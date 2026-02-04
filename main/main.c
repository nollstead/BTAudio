/*
 * BTAudio - ESP32 Bluetooth Audio Receiver
 * Supports A2DP audio streaming and SPP command interface
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "board.h"
#include "bt_audio.h"
#include "mongoose.h"
#include "mongoose_glue.h"
#include "ws2812.h"

// WiFi/Mongoose includes (for WiFi AP functionality - currently disabled in app_main)
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"

#include "audio_hal.h"
#include "board_def.h"
#include "driver/i2c_master.h" // used by scan_i2c test function
#include "nvs_flash.h"

static const char *TAG = "MAIN";

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

// Shared state
static audio_board_handle_t s_board_handle = NULL;
static ws2812_handle_t s_led = NULL;

// Forward declarations
static void init_nvs(void);
static void mongoose_task(void *arg);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data);
static void init_wifi_ap(void);
static void monitorMemory(void *pvParameters);

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

    // Initialize audio board/codec
    ESP_LOGI(TAG, "Initializing audio board");
    s_board_handle = audio_board_init();
    audio_hal_set_mute(s_board_handle->audio_hal, true); // Start muted until audio plays
    ws2812_set(s_led, WS2812_BLUE);

    // Initialize Bluetooth and start tasks
    bt_audio_init(s_board_handle, s_led, BTAUDIO_VERSION);
    bt_audio_start_a2dp();
    bt_audio_start_spp();

    // Initialize Wi-Fi (for Mongoose web server) as a task
    // init_spiffs();       // Only needed if using SPIFFS for web server
    // init_wifi_ap();
    // xTaskCreate(monitorMemory, "monitorMemory", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "BTAudio v%s started", BTAUDIO_VERSION);
}
