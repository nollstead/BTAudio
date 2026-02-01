#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// BTAUDIO_VERSION is defined by CMake from version.txt
#ifndef BTAUDIO_VERSION
#define BTAUDIO_VERSION "unknown"
#endif

// System info (read-only)
const char *device_get_version(void);
uint32_t device_get_uptime_sec(void);
uint32_t device_get_free_heap(void);
uint32_t device_get_min_free_heap(void);

// WiFi credentials (NVS-backed)
bool device_get_wifi_ssid(char *buf, size_t buf_size);
bool device_set_wifi_ssid(const char *ssid);
bool device_get_wifi_pass(char *buf, size_t buf_size);
bool device_set_wifi_pass(const char *pass);

#endif
