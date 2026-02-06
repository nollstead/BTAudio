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
| `adc_manager.c` | Unified 8-channel ADC interface over two ADS1115 chips |
| `adc_manager.h` | ADC manager public API |

## Initialization Sequence

`app_main()` performs the following steps:

1. Initialize NVS (required for Bluetooth pairing storage)
2. Initialize audio board (ES8388 codec, BD37033, ADS1115, WS2812 LED)
3. Initialize Bluetooth stack
4. Start A2DP sink task
5. Start SPP server task
6. LED turns blue when ready for connection

## Bluetooth Module (bt_audio)

### A2DP Sink

Receives audio stream from paired device and outputs via I2S to the ES8388 codec.

```c
bt_audio_init(board_handle, version_string);
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

## ADC Manager

Provides a unified 8-channel interface over two ADS1115 chips. Application code reads channels 0-7 without needing to know which physical chip or pin is involved.

### Channel Mapping

| Channel | Chip (Index) | Physical Pin | I2C Address |
|---------|-------------|-------------|-------------|
| 0-3 | 0 | A0-A3 | 0x48 |
| 4-7 | 1 | A0-A3 | 0x49 |

### Usage

```c
#include "adc_manager.h"

adc_manager_init();  // call after audio_board_init()

// Configure gain per chip (all 4 channels on that chip share the same PGA)
adc_manager_set_gain(0, ADS1115_GAIN_4_096);  // chip 0 → channels 0-3
adc_manager_set_gain(1, ADS1115_GAIN_4_096);  // chip 1 → channels 4-7

// Read any channel
int16_t raw;
float volts;
adc_manager_read(5, &raw, &volts);  // routes to chip 1, pin A1
```

### Per-Chip Settings

Gain and data rate are set by **chip index** (0 or 1), not by channel number. This matches the ADS1115 hardware — one PGA and one data rate per chip, shared across all 4 channels.

If only one chip is present, channels on the missing chip return `ESP_ERR_NOT_FOUND`. Use `adc_manager_channel_available(ch)` to check.

### Negative Value Clamping

The ADS1115 is a differential ADC that outputs signed 16-bit values. In single-ended mode, floating or grounded inputs can produce small negative readings due to ground reference offsets. By default, `adc_manager_read()` clamps negative values to zero, which is appropriate for most single-ended sensor applications.

```c
// Clamping is enabled by default — negative raw/volts values are returned as 0
adc_manager_read(ch, &raw, &volts);  // raw >= 0, volts >= 0.0

// Disable clamping for diagnostics (see true signed output)
adc_manager_set_clamp(false);
adc_manager_read(ch, &raw, &volts);  // raw may be negative

// Re-enable clamping
adc_manager_set_clamp(true);

// Query current setting
bool clamping = adc_manager_get_clamp();
```

## TODO

- [ ] Implement volume control via SPP commands
- [ ] Add EQ adjustment commands
- [ ] Integrate ADS1115 pot readings with BD37033 control
- [ ] Add connection status reporting
- [ ] Consider moving test tasks to separate module
