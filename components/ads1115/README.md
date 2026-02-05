# ADS1115 16-bit ADC Driver

ESP-IDF driver for the Texas Instruments ADS1115 16-bit, 4-channel ADC with I2C interface.

## Features

- Handle-based API supporting multiple devices on the same bus
- Single-ended reads (4 channels vs GND)
- Differential reads (4 differential pairs)
- Configurable gain (PGA) for different voltage ranges
- Configurable data rate (8-860 SPS)
- Comparator with threshold alerts

## I2C Addresses

The ADS1115 address is set by the ADDR pin:

| ADDR Pin | 7-bit Address | Define |
|----------|---------------|--------|
| GND | 0x48 | `ADS1115_ADDR_GND` |
| VDD | 0x49 | `ADS1115_ADDR_VDD` |
| SDA | 0x4A | `ADS1115_ADDR_SDA` |
| SCL | 0x4B | `ADS1115_ADDR_SCL` |

With 4 possible addresses, you can have up to 4 ADS1115 chips on one I2C bus (16 single-ended channels).

## Initialization

```c
#include "ads1115.h"
#include "i2c_bus.h"

// Get the shared I2C bus (created by audio_hal for ES8388)
i2c_master_bus_handle_t bus = i2c_bus_get_master_handle(I2C_NUM_0);

// Initialize two ADS1115 devices
ads1115_handle_t adc1, adc2;
ads1115_init(bus, ADS1115_ADDR_GND, 100000, &adc1);  // 0x48
ads1115_init(bus, ADS1115_ADDR_VDD, 100000, &adc2);  // 0x49
```

The init function verifies connectivity by reading the config register. If the device doesn't respond, it returns an error and the handle is NULL.

## Gain (PGA) Settings

The Programmable Gain Amplifier controls the full-scale voltage range. Choose the smallest range that fits your signal for best resolution.

| Setting | Full Scale | LSB Size | Use Case |
|---------|------------|----------|----------|
| `ADS1115_GAIN_6_144` | +/-6.144V | 187.5 uV | High voltage signals |
| `ADS1115_GAIN_4_096` | +/-4.096V | 125 uV | 3.3V systems (recommended) |
| `ADS1115_GAIN_2_048` | +/-2.048V | 62.5 uV | Default, 0-2V signals |
| `ADS1115_GAIN_1_024` | +/-1.024V | 31.25 uV | Low voltage signals |
| `ADS1115_GAIN_0_512` | +/-0.512V | 15.625 uV | Small signals |
| `ADS1115_GAIN_0_256` | +/-0.256V | 7.8 uV | Precision measurements |

**Important:** The full-scale range is what the ADC can *measure*, not the maximum safe input. Inputs are protected up to VDD + 0.3V regardless of gain.

**For 3.3V potentiometers:** Use `ADS1115_GAIN_4_096`. This provides full range (0-3.3V fits within +/-4.096V) with good resolution (125 uV/bit).

```c
ads1115_set_gain(adc1, ADS1115_GAIN_4_096);
```

## Data Rate Settings

Lower rates = less noise (more averaging). Higher rates = faster reads.

| Setting | SPS | Conversion Time |
|---------|-----|-----------------|
| `ADS1115_RATE_8` | 8 | ~125 ms |
| `ADS1115_RATE_16` | 16 | ~62.5 ms |
| `ADS1115_RATE_32` | 32 | ~31.25 ms |
| `ADS1115_RATE_64` | 64 | ~15.6 ms |
| `ADS1115_RATE_128` | 128 | ~7.8 ms (default) |
| `ADS1115_RATE_250` | 250 | ~4 ms |
| `ADS1115_RATE_475` | 475 | ~2.1 ms |
| `ADS1115_RATE_860` | 860 | ~1.2 ms |

```c
ads1115_set_rate(adc1, ADS1115_RATE_128);
```

## Single-Ended Reads

Read one of four channels (AIN0-AIN3) measured against GND:

```c
int16_t raw;
float volts;

if (ads1115_read_single_ended(adc1, 0, &raw) == ESP_OK) {  // Channel 0
    volts = ads1115_compute_volts(adc1, raw);
    printf("Channel 0: %d counts (%.3f V)\n", raw, volts);
}
```

## Differential Reads

Measure the voltage *difference* between two pins (can be positive or negative):

```c
int16_t diff;
if (ads1115_read_differential(adc1, ADS1115_MUX_DIFF_0_1, &diff) == ESP_OK) {
    float diff_volts = ads1115_compute_volts(adc1, diff);
    printf("A0-A1: %d counts (%.3f V)\n", diff, diff_volts);
}
```

Available differential pairs:

| Setting | Measurement |
|---------|-------------|
| `ADS1115_MUX_DIFF_0_1` | AIN0 - AIN1 |
| `ADS1115_MUX_DIFF_0_3` | AIN0 - AIN3 |
| `ADS1115_MUX_DIFF_1_3` | AIN1 - AIN3 |
| `ADS1115_MUX_DIFF_2_3` | AIN2 - AIN3 |

## Comparator

The comparator monitors ADC results and asserts the ALERT pin when thresholds are crossed. Useful for hardware-triggered interrupts.

### Setup

```c
// Calculate threshold in counts: counts = voltage / volts_per_bit
// At GAIN_4_096: volts_per_bit = 4.096 / 32768 = 0.000125
// 2.0V threshold = 2.0 / 0.000125 = 16000 counts

ads1115_set_comparator_thresholds(adc1, 0, 16000);  // Alert when > 2.0V

ads1115_set_comparator_config(adc1,
    ADS1115_COMP_MODE_TRADITIONAL,  // Hysteresis mode
    ADS1115_COMP_POL_LOW,           // ALERT active low
    ADS1115_COMP_LAT_OFF,           // Non-latching
    ADS1115_COMP_QUE_1);            // Assert after 1 conversion
```

### Comparator Modes

| Mode | Behavior |
|------|----------|
| Traditional | ALERT asserts when value > hi_thresh, clears when < lo_thresh (hysteresis) |
| Window | ALERT asserts when value < lo_thresh OR value > hi_thresh |

Connect the ALERT pin to an ESP32 GPIO configured as input with interrupt.

## Board Integration

The ADS1115 devices are initialized in `audio_board_init()`:

```c
// Get handle from audio_board
ads1115_handle_t adc = audio_board_get_ads1115(0);  // First chip
if (adc) {
    int16_t raw;
    ads1115_read_single_ended(adc, 0, &raw);
}
```

## API Reference

### Initialization

| Function | Description |
|----------|-------------|
| `ads1115_init(bus, addr, clk_hz, &handle)` | Initialize device on I2C bus |
| `ads1115_deinit(handle)` | Free device resources |

### Configuration

| Function | Description |
|----------|-------------|
| `ads1115_set_gain(handle, gain)` | Set PGA gain |
| `ads1115_get_gain(handle)` | Get current gain |
| `ads1115_set_rate(handle, rate)` | Set data rate |
| `ads1115_get_rate(handle)` | Get current rate |

### Reading

| Function | Description |
|----------|-------------|
| `ads1115_read_single_ended(handle, channel, &value)` | Read channel vs GND |
| `ads1115_read_differential(handle, mux, &value)` | Read differential pair |
| `ads1115_compute_volts(handle, counts)` | Convert counts to voltage |

### Comparator

| Function | Description |
|----------|-------------|
| `ads1115_set_comparator_thresholds(handle, lo, hi)` | Set threshold values |
| `ads1115_set_comparator_config(handle, mode, pol, lat, que)` | Configure comparator |

### Utility

| Function | Description |
|----------|-------------|
| `ads1115_get_address(handle)` | Get device I2C address |

## Troubleshooting

**Reading wrong values:**
- Check potentiometer wiring: outer pins to 3.3V and GND, wiper (middle) to ADC input
- Verify with multimeter that the actual voltage matches what the ADC reports
- Floating (unconnected) inputs will read ~0.5-0.6V (random noise)

**Device not responding:**
- Verify I2C wiring (SDA/SCL)
- Check ADDR pin is connected correctly for desired address
- Use I2C scan to verify device is detected

**Noisy readings:**
- Use slower data rate (`ADS1115_RATE_8` for best noise rejection)
- Add decoupling capacitor (0.1uF) near VDD pin
- For high-impedance sources, add a small capacitor (1-10nF) on the input

## TODO

- [ ] Add continuous conversion mode (vs single-shot)
- [ ] Add ALERT pin interrupt handling example
- [ ] Implement multi-channel scan function
- [ ] Add calibration/offset support
- [ ] Consider adding averaging/filtering helper

## References

- [ADS1115 Datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf)
- [Adafruit ADS1X15 Arduino Library](https://github.com/adafruit/Adafruit_ADS1X15) - API inspiration
