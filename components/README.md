# Components

This directory contains custom ESP-IDF components for the BTAudio project. Each component is self-contained with its own source, headers, and CMakeLists.txt.

## Component Overview

| Component | Description | I2C Address |
|-----------|-------------|-------------|
| [ads1115](ads1115/) | 16-bit ADC for analog inputs (potentiometers, sensors) | 0x48-0x4B |
| [audio_board](audio_board/) | Custom board definition for ESP-ADF (ES8388 codec integration) | - |
| [bd37033](bd37033/) | Audio processor with volume, EQ, and fader control | 0x40 |
| [mongoose](mongoose/) | Embedded web server with HTTP, WebSocket, and OTA support | - |
| [ws2812](ws2812/) | Addressable RGB LED driver using RMT peripheral | - |

## Architecture

### I2C Bus Sharing

Multiple I2C devices share a single bus (I2C_NUM_0). The bus is created by ESP-ADF's `audio_hal` during codec initialization. Other components retrieve the existing bus handle:

```c
#include "i2c_bus.h"

i2c_master_bus_handle_t bus = i2c_bus_get_master_handle(I2C_NUM_0);
your_device_init(bus, DEVICE_ADDR, 100000);
```

### Signal Flow

```
ESP32 I2S --> ES8388 DAC --> BD37033 --> Speakers/Headphones
                              ^
                              |
                         ADS1115 (volume/tone pots)
```

The ES8388 codec provides I2S-to-analog conversion. The BD37033 handles volume, EQ, and routing. The ADS1115 reads analog controls.

## Adding a New Component

1. Create a directory under `components/` with:
   - `CMakeLists.txt` - component build configuration
   - `include/` - public headers
   - Source files (`.c`)
   - `README.md` - component documentation

2. For I2C devices, accept a bus handle in your init function rather than creating a new bus

3. Add the component to your main `CMakeLists.txt` PRIV_REQUIRES if needed

4. Initialize in `audio_board_init()` in `board.c` for automatic setup

## Component Documentation

Each component has its own README.md with:
- Overview and purpose
- API reference with function signatures
- Configuration options (enums, defines)
- Usage examples
- Troubleshooting tips
- TODO section for planned work

## TODO

- [ ] Add ES8388 codec component (currently in ESP-ADF)
- [ ] Consider abstracting I2C device pattern into helper
- [ ] Add component for rotary encoder input
