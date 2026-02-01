#include "device_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "DEVICE_INFO";
static const char *NVS_NAMESPACE = "btaudio";

// System info

const char *device_get_version(void) {
    return BTAUDIO_VERSION;
}

uint32_t device_get_uptime_sec(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
}

uint32_t device_get_free_heap(void) {
    return esp_get_free_heap_size();
}

uint32_t device_get_min_free_heap(void) {
    return esp_get_minimum_free_heap_size();
}

// NVS helpers

static bool nvs_get_string(const char *key, char *buf, size_t buf_size) {
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    size_t len = buf_size;
    esp_err_t err = nvs_get_str(nvs, key, buf, &len);
    nvs_close(nvs);
    return err == ESP_OK;
}

static bool nvs_set_string(const char *key, const char *value) {
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing");
        return false;
    }
    esp_err_t err = nvs_set_str(nvs, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err == ESP_OK;
}

// WiFi credentials

bool device_get_wifi_ssid(char *buf, size_t buf_size) {
    return nvs_get_string("wifi_ssid", buf, buf_size);
}

bool device_set_wifi_ssid(const char *ssid) {
    return nvs_set_string("wifi_ssid", ssid);
}

bool device_get_wifi_pass(char *buf, size_t buf_size) {
    return nvs_get_string("wifi_pass", buf, buf_size);
}

bool device_set_wifi_pass(const char *pass) {
    return nvs_set_string("wifi_pass", pass);
}
