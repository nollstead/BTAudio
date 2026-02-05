# BD37033 Audio Processor Driver

ESP-IDF driver for the Rohm BD37033FV 4-channel audio processor.

## Overview

The BD37033FV provides:
- Master volume control (0 to -79 dB)
- 4-channel fader/balance control
- 3-band parametric EQ (bass, mid, treble)
- Input selection (4 inputs)
- Input gain control
- Loudness compensation

Connected via I2C at address `0x40` (7-bit).

## Signal Chain

```
ESP32 I2S --> ES8388 DAC --> LOUT2/ROUT2 --> BD37033 Input --> BD37033 Output --> Speakers
                                                                              --> Headphones
```

The ES8388 LOUT2/ROUT2 outputs feed the BD37033 input. The BD37033 then provides processed audio with volume, EQ, and routing control.

## Initialization

```c
#include "bd37033.h"
#include "i2c_bus.h"

// Get the shared I2C bus (created by audio_hal for ES8388)
i2c_master_bus_handle_t bus = i2c_bus_get_master_handle(I2C_NUM_0);

// Initialize and configure defaults
bd37033_init(bus, BD37033_I2C_ADDR_DEFAULT, 100000);
bd37033_setup_defaults();  // Flat EQ, pass-through mode
```

## Register Addresses

| Register | Address | Description |
|----------|---------|-------------|
| `BD37033_INITIAL_SETUP` | 0x01 | Advanced switch mode, anti-alias |
| `BD37033_LPF_SETUP` | 0x02 | Subwoofer LPF configuration |
| `BD37033_MIXING_SETUP` | 0x03 | Input mixing mode |
| `BD37033_INPUT_SELECT` | 0x05 | Active input selection |
| `BD37033_INPUT_GAIN` | 0x06 | Input gain (-12 to +12 dB) |
| `BD37033_VOLUME_GAIN` | 0x20 | Master volume (0 to -79 dB) |
| `BD37033_FADER_1_FRONT` | 0x28 | Front left fader |
| `BD37033_FADER_2_FRONT` | 0x29 | Front right fader |
| `BD37033_FADER_1_REAR` | 0x2A | Rear left fader |
| `BD37033_FADER_2_REAR` | 0x2B | Rear right fader |
| `BD37033_BASS_SETUP` | 0x41 | Bass EQ frequency/Q |
| `BD37033_MIDDLE_SETUP` | 0x44 | Mid EQ frequency/Q |
| `BD37033_TREBLE_SETUP` | 0x47 | Treble EQ frequency/Q |
| `BD37033_BASS_GAIN` | 0x51 | Bass gain |
| `BD37033_MIDDLE_GAIN` | 0x54 | Mid gain |
| `BD37033_TREBLE_GAIN` | 0x57 | Treble gain |
| `BD37033_LOUDNESS_GAIN` | 0x75 | Loudness compensation |

## Default Configuration

`bd37033_setup_defaults()` configures basic pass-through operation with flat EQ:

| Register | Value | Description |
|----------|-------|-------------|
| INITIAL_SETUP | 0x24 | Advanced switch mode (reduces pop/click) |
| LPF_SETUP | 0x00 | Subwoofer LPF off |
| MIXING_SETUP | 0x61 | A_Single mode (A1->L, A2->R) |
| INPUT_SELECT | 0x00 | Input 1 |
| INPUT_GAIN | 0x00 | 0 dB input gain |
| VOLUME_GAIN | 0x00 | 0 dB (max volume) |
| FADER_* | 0x00 | All channels 0 dB |
| BASS/MID/TREBLE_GAIN | 0x00 | Flat EQ |
| LOUDNESS_GAIN | 0x00 | Loudness off |

## Important: Inverted Bit Polarity

The BD37033 uses **inverted logic** for ON/OFF bits: `0 = ON`, `1 = OFF`.

Example from INITIAL_SETUP (0x01) with value 0x24:
- D7=0 -> Advanced Switch **ON**
- D6=0 -> Anti Alias Filter **ON**

## Input Mixing Modes

The BD37033 has two stereo input pairs (A1/A2 and B1/B2). The lower bits of MIXING_SETUP control how they're used:

| D1-D0 | Mode | Description |
|-------|------|-------------|
| 00 | Mix | Combines both inputs (A1+B1->L, A2+B2->R) |
| 01 | A_Single | Uses only A inputs (A1->L, A2->R) |
| 10 | B_Single | Uses only B inputs (B1->L, B2->R) |
| 11 | Prohibited | Don't use |

**Current setup:** A_Single (0x61) - ES8388 LOUT2->A1, ROUT2->A2

To add a second input source later, wire it to B1/B2 and switch to Mix mode (0x60), or switch between A_Single/B_Single via `bd37033_select_input()`.

## Enumerations

### Channel Identifiers

```c
typedef enum {
    BD37033_CH_FRONT_LEFT,
    BD37033_CH_FRONT_RIGHT,
    BD37033_CH_REAR_LEFT,
    BD37033_CH_REAR_RIGHT,
} bd37033_channel_t;
```

### Input Selector

```c
typedef enum {
    BD37033_INPUT_1,
    BD37033_INPUT_2,
    BD37033_INPUT_3,
    BD37033_INPUT_4,
} bd37033_input_t;
```

### Loudness Mode

```c
typedef enum {
    BD37033_LOUDNESS_OFF,
    BD37033_LOUDNESS_ON,
} bd37033_loudness_t;
```

### Mute State

```c
typedef enum {
    BD37033_MUTE_OFF,
    BD37033_MUTE_ON,
} bd37033_mute_t;
```

## API Reference

### Initialization

| Function | Description |
|----------|-------------|
| `bd37033_init(bus, addr, clk_hz)` | Initialize driver on I2C bus |
| `bd37033_setup_defaults()` | Configure sensible defaults |
| `bd37033_deinit()` | Remove device from bus |

### Volume & Gain

| Function | Description |
|----------|-------------|
| `bd37033_set_volume(vol_db)` | Set master volume (0 to -79 dB) |
| `bd37033_set_input_gain(gain_db)` | Set input gain (-12 to +12 dB) |
| `bd37033_set_mute(mute)` | Enable/disable mute |

### Input & Routing

| Function | Description |
|----------|-------------|
| `bd37033_select_input(input)` | Select active input (1-4) |
| `bd37033_set_channel_attenuation(ch, att_db)` | Per-channel fader |

### Tone Control

| Function | Description |
|----------|-------------|
| `bd37033_set_bass(bass_db)` | Adjust bass (+/- dB) |
| `bd37033_set_treble(treble_db)` | Adjust treble (+/- dB) |
| `bd37033_set_loudness(mode)` | Enable/disable loudness |

### Utility

| Function | Description |
|----------|-------------|
| `bd37033_read_register(reg, &value)` | Read register value |
| `bd37033_test()` | Probe and verify device |

## Board Integration

Check if the BD37033 is present before using (may not be connected):

```c
#include "board.h"

if (audio_board_bd37033_available()) {
    bd37033_set_volume(-10);  // -10 dB
    bd37033_set_bass(3);      // +3 dB bass boost
}
```

## Gain Staging

For optimal dynamic range, set ES8388 output to 0dB and control volume via BD37033:

**ES8388 output registers:**
- `0x2E` (DACCONTROL24): LOUT1 volume
- `0x2F` (DACCONTROL25): ROUT1 volume
- `0x30` (DACCONTROL26): LOUT2 volume (to BD37033)
- `0x31` (DACCONTROL27): ROUT2 volume (to BD37033)

Values: `0x00` = -45dB (min), `0x1E` = 0dB, `0x21` = +4.5dB (max)

**Recommendation:** Set ES8388 output to 0dB (`0x1E`) and control volume via BD37033. This maximizes dynamic range and puts volume/EQ control in one place.

## TODO

- [ ] Implement `bd37033_set_middle()` for mid-range EQ
- [ ] Add EQ frequency/Q configuration functions
- [ ] Add subwoofer LPF configuration
- [ ] Implement balance/fader convenience functions
- [ ] Add register dump function for debugging

## References

- [BD37033FV Datasheet](https://jlcpcb.com/api/file/downloadByFileSystemAccessId/8588884437430120448) - Register addresses and bit definitions
- [liman324/BD37033FV](https://github.com/liman324/BD37033FV) - Arduino library reference
- ESP-ADF i2c_bus component - Bus sharing implementation
