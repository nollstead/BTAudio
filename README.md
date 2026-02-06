# BTAudio

ESP32-based Bluetooth audio receiver with hardware volume/EQ control and web interface.

## Project Overview

BTAudio is a custom Bluetooth audio adapter built on the ESP32 platform using Espressif's Audio Development Framework (ESP-ADF). It receives audio via Bluetooth A2DP and outputs to speakers/headphones through an audio processing chain.

### Hardware Stack

| Component | Chip | Function |
|-----------|------|----------|
| MCU | ESP32-WROVER-E-N8R8 | Main controller, Bluetooth, WiFi (8MB flash, 8MB PSRAM) |
| Codec | ES8388 | I2S DAC/ADC, analog output |
| Audio Processor | BD37033FV | Volume, EQ, fader/balance |
| ADC | ADS1115 (x2) | Read analog controls (pots, sensors) |
| Status LED | WS2812 | RGB status indicator |

### Signal Flow

```
Bluetooth A2DP --> ESP32 I2S --> ES8388 DAC --> BD37033 --> Speakers/Headphones
                                                  ^
                                                  |
                                             ADS1115 (pots)
```

### Features

- **Bluetooth A2DP** - Standard Bluetooth audio streaming
- **SPP Command Interface** - Serial port profile for remote control/configuration
- **Hardware Volume/EQ** - BD37033 provides analog domain processing
- **Analog Controls** - Up to 8 potentiometers via dual ADS1115 ADCs
- **Web Interface** - Mongoose-based web UI (WiFi AP mode, currently disabled)
- **OTA Updates** - Firmware updates via web interface

## Project Structure

```
BTAudio/
├── main/                   # Application code
│   ├── main.c              # Entry point, initialization
│   ├── bt_audio.c/.h       # Bluetooth A2DP and SPP
│   └── device_info.c/.h    # System info and NVS helpers
├── components/             # Custom ESP-IDF components
│   ├── ads1115/            # 16-bit ADC driver
│   ├── audio_board/        # Custom ESP-ADF board definition
│   ├── bd37033/            # Audio processor driver
│   ├── mongoose/           # Web server
│   └── ws2812/             # RGB LED driver
├── version.txt             # Firmware version (used by build)
└── README.md               # This file
```

See [main/README.md](main/README.md) for application code details.
See [components/README.md](components/README.md) for component documentation.

## Build & Flash

### Prerequisites

- ESP-IDF v5.3+
- ESP-ADF (configured as IDF component or in `$ADF_PATH`)

### Build Commands

```bash
# Configure (first time or to change settings)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor
idf.py -p COM3 flash monitor
```

### Key Configuration

In `menuconfig`:
- **Audio HAL > Audio board** - Select "My Board v1.0"
- **Bluetooth** - Enabled by default
- <!-- TODO: Add other key config items -->

## Hardware Connections

### I2C Bus (shared)

| Signal | GPIO | Notes |
|--------|------|-------|
| SDA | 18 | Shared by ES8388, BD37033, ADS1115 |
| SCL | 23 | |

### I2S (to ES8388)

| Signal | GPIO |
|--------|------|
| MCLK | 0 |
| BCLK | 5 |
| WS (LRCLK) | 25 |
| DOUT | 26 |
| DIN | 19 |

### Other

| Signal | GPIO | Notes |
|--------|------|-------|
| RGB LED | 2 | WS2812 data line |
| PA Enable | 21 | Power amplifier enable |
| AUX Detect | 12 | Auxiliary input detection |

## Usage

### Bluetooth Pairing

1. Power on the device - LED shows yellow during init, blue when ready
2. Search for "BTAudio" on your phone/computer
3. Pair and connect
4. Audio streams automatically

### LED Status

| Color | Meaning |
|-------|---------|
| Yellow | Initializing |
| Blue | Ready, waiting for connection |
| Green | Connected and playing |
| <!-- TODO: Add other states --> | |

### SPP Commands

Connect via Bluetooth SPP (serial) to send commands:

<!-- TODO: Document SPP command protocol -->

```
# Example commands (placeholder)
VOL:50      # Set volume to 50%
BASS:+3     # Boost bass 3dB
```

## Development Notes

### Adding New I2C Devices

1. Create component in `components/`
2. Accept `i2c_master_bus_handle_t` in init (don't create new bus)
3. Initialize in `audio_board_init()` in `board.c`
4. See [components/README.md](components/README.md) for pattern

### Versioning

Version is stored in `version.txt` and embedded at build time via CMake. The version is:
- Stored in NVS on first boot
- Reported via SPP
- Displayed in web UI

### Memory Considerations

<!-- TODO: Document heap usage, task stack sizes, etc. -->

## TODO

<!-- High-level project TODOs - update as needed -->

- [x] Complete GPIO pin documentation
- [ ] Document SPP command protocol
- [ ] Enable and test WiFi AP mode
- [ ] Implement volume control via ADS1115 pots
- [ ] Add EQ preset support
- [ ] Web UI for configuration
- [ ] Document LED status states

## References

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [ESP-ADF Documentation](https://docs.espressif.com/projects/esp-adf/en/latest/)
- [ES8388 Datasheet](https://www.everest-semi.com/pdf/ES8388%20DS.pdf)
- [BD37033FV Datasheet](https://jlcpcb.com/api/file/downloadByFileSystemAccessId/8588884437430120448)
- [ADS1115 Datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf)

## License

<!-- TODO: Add license information -->
