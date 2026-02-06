/*
 * bt_audio.c - Bluetooth Audio Module
 * Handles A2DP audio streaming and SPP command interface
 */

#include "bt_audio.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "a2dp_stream.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "esp_a2dp_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "i2s_stream.h"

static const char *TAG_BT = "BT";
static const char *TAG_A2DP = "A2DP";
static const char *TAG_SPP = "SPP";

#define SPP_SERVER_NAME "BTAudio_SPP"
#define DEVICE_NAME "BTAudio"

// SPP command types
typedef enum {
    CMD_HELP,
    CMD_VERSION,
    CMD_HELLO,
    CMD_UNKNOWN,
} spp_cmd_t;

// Module state (set via bt_audio_init)
static audio_board_handle_t s_board_handle = NULL;
static const char *s_version = "unknown";
static uint32_t s_spp_handle = 0;

// Forward declarations
static void init_bluetooth_stack(void);
static void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static void a2dp_task(void *pvParameters);
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static spp_cmd_t parse_command(const char *cmd);
static void handle_spp_command(const char *cmd, size_t len);
static void spp_task(void *pvParameters);

void bt_audio_init(audio_board_handle_t board_handle, const char *version) {
    s_board_handle = board_handle;
    s_version = version;

    init_bluetooth_stack();
}

void bt_audio_start_a2dp(void) {
    xTaskCreate(a2dp_task, "a2dp", 4096, NULL, 5, NULL);
}

void bt_audio_start_spp(void) {
    xTaskCreate(spp_task, "spp", 3072, NULL, 5, NULL);
}

static void init_bluetooth_stack(void) {
    ESP_LOGI(TAG_BT, "Initializing Bluetooth");

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
            audio_board_led_set(WS2812_GREEN);
        } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG_A2DP, "A2DP disconnected");
            audio_board_led_set(WS2812_BLUE);
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
        snprintf(response, sizeof(response), "BTAudio v%s\r\n", s_version);
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
