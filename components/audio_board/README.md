# Audio Board

Custom board definition for ESP-ADF (Espressif Audio Development Framework). This component integrates the ES8388 codec and manages initialization of all audio-related I2C peripherals.

## Overview

The audio_board component provides:
- Custom board configuration for ESP-ADF (similar to LyraT board definitions)
- ES8388 codec initialization via audio_hal
- I2C bus setup and sharing with other components
- Centralized initialization of BD37033 and ADS1115 devices
- Board-level accessor functions

## Structure

```
audio_board/
  CMakeLists.txt
  Kconfig.projbuild          # Board selection config
  include/
    board_pins_config.h      # GPIO pin definitions
  my_board_v1_0/
    board.c                  # Main board implementation
    board.h                  # Public board API
    board_def.h              # Board-specific defines
    board_pins_config.c      # Pin configuration
    CMakeLists.txt
```

## API Reference

### Initialization

| Function | Description |
|----------|-------------|
| `audio_board_init()` | Initialize entire audio system (codec, peripherals) |
| `audio_board_codec_init()` | Initialize ES8388 codec only |
| `audio_board_deinit(handle)` | Deinitialize and free resources |
| `audio_board_i2c_init()` | Initialize I2C bus |
| `audio_board_i2s_init()` | Initialize I2S interface |

### Accessors

| Function | Description |
|----------|-------------|
| `audio_board_get_handle()` | Get the audio board handle |
| `audio_board_bd37033_available()` | Check if BD37033 is present |
| `audio_board_get_ads1115(index)` | Get ADS1115 handle (0 or 1) |

### Utility

| Function | Description |
|----------|-------------|
| `audio_board_led_init()` | Initialize status LED display service |
| `board_codec_reset()` | Hardware reset the codec |

## Usage

### Basic Initialization

```c
#include "board.h"

void app_main(void) {
    // Initialize entire audio system
    audio_board_handle_t board = audio_board_init();

    // Get codec handle for direct control
    audio_hal_handle_t hal = board->audio_hal;
    audio_hal_set_volume(hal, 50);
}
```

### Accessing Peripherals

```c
// Check if BD37033 is connected
if (audio_board_bd37033_available()) {
    bd37033_set_volume(-10);  // -10 dB
    bd37033_set_bass(3);      // +3 dB boost
}

// Get ADS1115 for analog inputs
ads1115_handle_t adc = audio_board_get_ads1115(0);
if (adc) {
    int16_t raw;
    ads1115_read_single_ended(adc, 0, &raw);
}
```

## I2C Bus Sharing

ESP-ADF's `i2c_bus` component manages bus handles. When `audio_board_init()` initializes the ES8388, it creates the I2C bus. The audio_board then retrieves this handle and passes it to other I2C devices:

```c
// Inside board.c
i2c_master_bus_handle_t bus = i2c_bus_get_master_handle(I2C_NUM_0);
bd37033_init(bus, BD37033_I2C_ADDR_DEFAULT, 100000);
ads1115_init(bus, ADS1115_ADDR_GND, 100000, &ads1115_handle_1);
```

External code can also get the bus handle for additional devices:

```c
#include "i2c_bus.h"

i2c_master_bus_handle_t bus = i2c_bus_get_master_handle(I2C_NUM_0);
my_device_init(bus, MY_DEVICE_ADDR, 100000);
```

## Signal Chain

```
ESP32 I2S --> ES8388 DAC --> LOUT2/ROUT2 --> BD37033 Input --> BD37033 Output --> Speakers
                                                                              --> Headphones
```

The ES8388 LOUT2/ROUT2 outputs feed the BD37033 input. The BD37033 provides processed audio with volume control, EQ, and fader/balance.

## Configuration

Board selection is done via Kconfig. Run `idf.py menuconfig` and navigate to:
- Audio HAL -> Audio board -> Select "My Board v1.0"

## Pin Configuration

Pin assignments are defined in `board_pins_config.h`. See that file for the specific GPIO mappings for I2C, I2S, and other peripherals.

## TODO

- [ ] Document all GPIO pin assignments in this README
- [ ] Add headphone detect support
- [ ] Add PA (power amplifier) enable control
- [ ] Consider adding board revision detection
- [ ] Add audio_board_get_bd37033() accessor function
