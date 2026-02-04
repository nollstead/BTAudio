/*
 * BTAudio - ESP32 Bluetooth Audio Receiver
 * Supports A2DP audio streaming and SPP command interface
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "board.h"
#include "mongoose.h"
#include "mongoose_glue.h"
#include "ws2812.h"

#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "mongoose.h"

#include "a2dp_stream.h"
#include "audio_element.h"
#include "audio_hal.h"
#include "audio_pipeline.h"
#include "board_def.h"
#include "driver/i2c_master.h"
#include "driver/i2s.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "filter_resample.h"
#include "i2s_stream.h"
#include "nvs_flash.h"

static const char *TAG = "MAIN";
static const char *TAG_A2DP = "A2DP";
static const char *TAG_SPP = "SPP";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// WiFi Access Point configuration
#define AP_SSID "BTAudio"
#define AP_PASS "btaudio123" // Min 8 chars, or empty string for open network
#define AP_MAX_CONN 4

// BTAUDIO_VERSION is defined by CMake from version.txt
#ifndef BTAUDIO_VERSION
#define BTAUDIO_VERSION "unknown"
#endif
#define SPP_SERVER_NAME "BTAudio_SPP"
#define DEVICE_NAME "BTAudio"

// SPP command types
typedef enum {
    CMD_HELP,
    CMD_VERSION,
    CMD_HELLO,
    CMD_UNKNOWN,
} spp_cmd_t;

// Shared state
static audio_board_handle_t s_board_handle = NULL;
static ws2812_handle_t s_led = NULL;
static uint32_t s_spp_handle = 0;

// Forward declarations
static void init_nvs(void);
static void init_bluetooth(void);
static void a2dp_task(void *pvParameters);
static void spp_task(void *pvParameters);
static void mongoose_task(void *arg);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data);
static void init_wifi_ap(void);
static void monitorMemory(void *pvParameters);
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static void handle_spp_command(const char *cmd, size_t len);
static void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

// Unused test functions
__attribute__((unused)) static void ledTestTask(void *pvParameters);
__attribute__((unused)) void scan_i2c(void);

__attribute__((unused)) static void init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .max_files = 20,
        .format_if_mount_failed = true,
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    ESP_LOGI("SPIFFS", "SPIFFS mounted at /spiffs");
}

__attribute__((unused)) static void monitorMemory(void *pvParameters) {
    while (1) {
        ESP_LOGI("MEM", "Free heap: %d", esp_get_free_heap_size());
        ESP_LOGI("MEM", "Min free heap (since boot): %d", esp_get_minimum_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000)); // Log every 5 seconds
    }
}

/*
 * Initialize Non-Volatile Storage (NVS).
 *
 * NVS is a key-value storage system in flash memory used by ESP-IDF components
 * to persist data across reboots. Bluetooth requires NVS to store:
 *   - Paired device information
 *   - Link keys for bonded devices
 *   - Device name and other settings
 *
 * If the NVS partition is full or corrupted (ESP_ERR_NVS_NO_FREE_PAGES),
 * we erase it and reinitialize. This loses stored pairings but allows
 * the system to start fresh.
 */
static void init_nvs(void) {
    ESP_LOGI(TAG, "Initializing NVS");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Store firmware version in NVS
    nvs_handle_t nvs;
    err = nvs_open("btaudio", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        char stored_version[32] = {0};
        size_t len = sizeof(stored_version);

        // Check if version changed
        err = nvs_get_str(nvs, "fw_version", stored_version, &len);
        if (err != ESP_OK || strcmp(stored_version, BTAUDIO_VERSION) != 0) {
            nvs_set_str(nvs, "fw_version", BTAUDIO_VERSION);
            nvs_commit(nvs);
            ESP_LOGI(TAG, "Firmware version %s stored in NVS", BTAUDIO_VERSION);
        } else {
            ESP_LOGI(TAG, "Firmware version %s already in NVS", BTAUDIO_VERSION);
        }
        nvs_close(nvs);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    }
}

static void init_bluetooth(void) {
    ESP_LOGI(TAG, "Initializing Bluetooth");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_gap_set_device_name(DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
}

static void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    if (event == ESP_A2D_CONNECTION_STATE_EVT) {
        esp_a2d_connection_state_t state = param->conn_stat.state;
        if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(TAG_A2DP, "A2DP connected");
            ws2812_set(s_led, WS2812_GREEN);
        } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG_A2DP, "A2DP disconnected");
            ws2812_set(s_led, WS2812_BLUE);
            audio_hal_set_mute(s_board_handle->audio_hal, true);
        }
    } else if (event == ESP_A2D_AUDIO_STATE_EVT) {
        esp_a2d_audio_state_t state = param->audio_stat.state;
        if (state == ESP_A2D_AUDIO_STATE_STARTED) {
            ESP_LOGI(TAG_A2DP, "Audio streaming started");
            audio_hal_set_mute(s_board_handle->audio_hal, false);
        } else if (state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND ||
                   state == ESP_A2D_AUDIO_STATE_STOPPED) {
            ESP_LOGI(TAG_A2DP, "Audio streaming paused/stopped");
            audio_hal_set_mute(s_board_handle->audio_hal, true);
        }
    }
}

static void a2dp_task(void *pvParameters) {
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t bt_stream_reader, i2s_stream_writer;

    ESP_LOGI(TAG_A2DP, "Creating audio pipeline");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG_A2DP, "Initializing A2DP stream");
    a2dp_stream_config_t a2dp_config = {
        .type = AUDIO_STREAM_READER,
        .user_callback = {
            .user_a2d_cb = a2dp_callback,
        },
        .audio_hal = s_board_handle->audio_hal,
    };
    bt_stream_reader = a2dp_stream_init(&a2dp_config);

    ESP_LOGI(TAG_A2DP, "Registering pipeline elements");
    audio_pipeline_register(pipeline, bt_stream_reader, "bt");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    const char *link_tag[2] = {"bt", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);

    ESP_LOGI(TAG_A2DP, "Setting up event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG_A2DP, "Starting audio pipeline");
    audio_pipeline_run(pipeline);

    ESP_LOGI(TAG_A2DP, "Listening for pipeline events");
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_A2DP, "Event interface error: %d", ret);
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
            msg.source == (void *)bt_stream_reader && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(bt_stream_reader, &music_info);

            ESP_LOGI(TAG_A2DP, "Music info: sample_rate=%d, bits=%d, channels=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_set_music_info(i2s_stream_writer, music_info.sample_rates,
                                         music_info.channels, music_info.bits);
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits,
                               music_info.channels);
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
            msg.source == (void *)i2s_stream_writer && msg.cmd == AEL_MSG_CMD_REPORT_STATUS &&
            (((int)msg.data == AEL_STATUS_STATE_STOPPED) ||
             ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            ESP_LOGW(TAG_A2DP, "Stop event received");
            break;
        }
    }

    // Cleanup (only reached if pipeline stops)
    ESP_LOGI(TAG_A2DP, "Stopping audio pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);
    audio_pipeline_remove_listener(pipeline);
    audio_event_iface_destroy(evt);
    audio_pipeline_unregister(pipeline, bt_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(bt_stream_reader);
    audio_element_deinit(i2s_stream_writer);

    vTaskDelete(NULL);
}

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG_SPP, "SPP initialized");
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        break;

    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG_SPP, "SPP server started");
        break;

    case ESP_SPP_SRV_OPEN_EVT: {
        ESP_LOGI(TAG_SPP, "Client connected");
        s_spp_handle = param->srv_open.handle;
        const char *welcome = "Type 'help' for a list of available commands.\r\n";
        esp_spp_write(s_spp_handle, strlen(welcome), (uint8_t *)welcome);
        break;
    }

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG_SPP, "Client disconnected");
        s_spp_handle = 0;
        break;

    case ESP_SPP_DATA_IND_EVT:
        if (param->data_ind.len > 0) {
            handle_spp_command((const char *)param->data_ind.data, param->data_ind.len);
        }
        break;

    default:
        break;
    }
}

static spp_cmd_t parse_command(const char *cmd) {
    if (strcasecmp(cmd, "help") == 0)
        return CMD_HELP;
    if (strcasecmp(cmd, "version") == 0)
        return CMD_VERSION;
    if (strcasecmp(cmd, "hello") == 0)
        return CMD_HELLO;
    return CMD_UNKNOWN;
}

static void handle_spp_command(const char *cmd, size_t len) {
    // Create null-terminated copy and trim whitespace
    char buf[64];
    size_t copy_len = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, cmd, copy_len);
    buf[copy_len] = '\0';

    // Trim trailing whitespace/newlines
    while (copy_len > 0 &&
           (buf[copy_len - 1] == '\n' || buf[copy_len - 1] == '\r' || buf[copy_len - 1] == ' ')) {
        buf[--copy_len] = '\0';
    }

    ESP_LOGI(TAG_SPP, "Command: '%s'", buf);

    char response[128];

    switch (parse_command(buf)) {
    case CMD_HELP:
        snprintf(response, sizeof(response),
                 "Available commands:\r\n"
                 "  help    - Show this help\r\n"
                 "  version - Show firmware version\r\n"
                 "  hello   - Say hello\r\n");
        break;
    case CMD_VERSION:
        snprintf(response, sizeof(response), "BTAudio v%s\r\n", BTAUDIO_VERSION);
        break;
    case CMD_HELLO:
        snprintf(response, sizeof(response), "Hiya!\r\n");
        break;
    default:
        snprintf(response, sizeof(response), "Unknown command: %s\r\nType 'help' for commands\r\n",
                 buf);
        break;
    }

    if (s_spp_handle != 0) {
        esp_spp_write(s_spp_handle, strlen(response), (uint8_t *)response);
    }
}

static void spp_task(void *pvParameters) {
    ESP_LOGI(TAG_SPP, "Initializing SPP");

    ESP_ERROR_CHECK(esp_spp_register_callback(spp_callback));

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = false,
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

    // SPP runs via callbacks, task can idle or be deleted
    vTaskDelete(NULL);
}

static void mongoose_task(void *arg) {
    mongoose_init();
    ESP_LOGI("MONGOOSE", "Web server started on port 80");

    for (;;) {
        mongoose_poll();
    }

    vTaskDelete(NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI("WIFI", "Access Point started - SSID: %s", AP_SSID);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // Start Mongoose when AP is ready
        static bool mongoose_started = false;
        if (!mongoose_started) {
            mongoose_started = true;
            xTaskCreate(mongoose_task, "mongoose", 4096, NULL, 5, NULL);
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI("WIFI", "Client connected - MAC: " MACSTR, MAC2STR(event->mac));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI("WIFI", "Client disconnected - MAC: " MACSTR, MAC2STR(event->mac));
    }
}

__attribute__((unused)) static void init_wifi_ap(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wc = {
        .ap =
            {
                .ssid = AP_SSID,
                .ssid_len = strlen(AP_SSID),
                .password = AP_PASS,
                .max_connection = AP_MAX_CONN,
                .authmode = strlen(AP_PASS) > 0 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("WIFI", "WiFi AP started - connect to '%s' then browse to http://192.168.4.1",
             AP_SSID);
}

__attribute__((unused)) void scan_i2c(void) {
    i2c_port_t port = 0;
    int sda_pin = BOARD_I2C_SDA_PIN;
    int scl_pin = BOARD_I2C_SCL_PIN;

    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = port,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags =
            {
                .enable_internal_pullup = true,
            },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    while (1) {
        printf("\nI2C Scan:\n");
        for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
            if (i2c_master_probe(bus, addr, 10) == ESP_OK) {
                printf("Found: 7-bit=0x%02X   8-bit=0x%02X\n", addr, (uint8_t)(addr << 1));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

__attribute__((unused)) static void ledTestTask(void *pvParameters) {
    while (1) {
        ws2812_set(s_led, WS2812_YELLOW);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ws2812_set(s_led, WS2812_GREEN);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("A2DP", ESP_LOG_WARN);      // SPP: only WARN and ERROR
    esp_log_level_set("esp_image", ESP_LOG_WARN); // SPP: only WARN and ERROR

    // esp_log_level_set("SPP", ESP_LOG_WARN);    // SPP: only WARN and ERROR

    // Initialize LED
    ESP_LOGI(TAG, "Initializing LED on GPIO%d", BOARD_RGB_LED);
    esp_err_t err = ws2812_init(BOARD_RGB_LED, &s_led);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED init failed: %d", err);
        return;
    }
    ws2812_set(s_led, WS2812_YELLOW);

    // Initialize NVS
    init_nvs();

    // Initialize Bluetooth stack
    init_bluetooth();

    // Initialize Wi-Fi (for Mongoose web server) as a task
    // init_spiffs();       // Only needed if using SPIFFS for web server
    // init_wifi_ap();

    // Initialize audio board/codec
    ESP_LOGI(TAG, "Initializing audio board");
    s_board_handle = audio_board_init();
    audio_hal_set_mute(s_board_handle->audio_hal, true);  // Start muted until audio plays
    ws2812_set(s_led, WS2812_BLUE);

    // Start tasks
    xTaskCreate(a2dp_task, "a2dp", 4096, NULL, 5, NULL);
    xTaskCreate(spp_task, "spp", 3072, NULL, 5, NULL);
    // xTaskCreate(monitorMemory, "monitorMemory", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "BTAudio v%s started", BTAUDIO_VERSION);
}
