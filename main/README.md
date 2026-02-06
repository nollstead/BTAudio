# Main Application

This directory contains the main application code for BTAudio.

## Files

| File | Description |
|------|-------------|
| `main.c` | Application entry point, initialization sequence, task creation |
| `bt_audio.c` | Bluetooth A2DP streaming and SPP command interface |
| `bt_audio.h` | Bluetooth module public API |
| `device_info.c` | System info helpers (version, uptime, heap) and NVS utilities |
| `device_info.h` | Device info public API |

## Initialization Sequence

`app_main()` performs the following steps:

1. Initialize WS2812 status LED (yellow = initializing)
2. Initialize NVS (required for Bluetooth pairing storage)
3. Initialize audio board (ES8388 codec, BD37033, ADS1115)
4. Initialize Bluetooth stack
5. Start A2DP sink task
6. Start SPP server task
7. LED turns blue when ready for connection

## Bluetooth Module (bt_audio)

### A2DP Sink

Receives audio stream from paired device and outputs via I2S to the ES8388 codec.

```c
bt_audio_init(board_handle, led_handle, version_string);
bt_audio_start_a2dp();  // Creates A2DP task
```

### SPP Server

Provides a serial port profile for remote control/debugging. Connect via Bluetooth serial terminal.

```c
bt_audio_start_spp();  // Creates SPP task
```

Current commands:
- `help` - List available commands
- `version` - Show firmware version
- `hello` - Test response

## Device Info Module

Provides system information and NVS persistence helpers.

```c
const char *version = device_get_version();
uint32_t uptime = device_get_uptime_sec();
uint32_t free_heap = device_get_free_heap();

// WiFi credential storage (for future use)
device_set_wifi_ssid("MyNetwork");
device_set_wifi_pass("MyPassword");
```

## Task Overview

| Task | Stack | Priority | Description |
|------|-------|----------|-------------|
| a2dp | 4096 | 5 | A2DP audio pipeline |
| spp | 3072 | 5 | SPP command processing |
| adc_test | 4096 | 5 | ADS1115 test reads (temporary) |

## TODO

- [ ] Implement volume control via SPP commands
- [ ] Add EQ adjustment commands
- [ ] Integrate ADS1115 pot readings with BD37033 control
- [ ] Add connection status reporting
- [ ] Consider moving test tasks to separate module
