# WS2812 RGB LED Driver

Simple driver for WS2812 (NeoPixel) addressable RGB LEDs using the ESP32 RMT peripheral.

## Features

- Handle-based API for multiple LED instances
- Predefined color palette
- Custom RGB color support
- Uses RMT peripheral for precise timing

## Board-Level Usage

In this project, the WS2812 LED is initialized and managed by the `audio_board` component. Application code uses the board-level accessor functions rather than the ws2812 handle directly:

```c
#include "board.h"

audio_board_led_set(WS2812_GREEN);       // Set predefined color
audio_board_led_set_rgb(64, 32, 0);      // Set custom RGB color
```

## Standalone Initialization

The ws2812 component can also be used independently with its handle-based API:

```c
#include "ws2812.h"

ws2812_handle_t led;
ws2812_init(GPIO_NUM_48, &led);  // Initialize LED on GPIO 48
```

## Predefined Colors

| Color | Name |
|-------|------|
| `WS2812_OFF` | Off (black) |
| `WS2812_RED` | Red |
| `WS2812_GREEN` | Green |
| `WS2812_BLUE` | Blue |
| `WS2812_WHITE` | White |
| `WS2812_YELLOW` | Yellow |
| `WS2812_CYAN` | Cyan |
| `WS2812_MAGENTA` | Magenta |
| `WS2812_ORANGE` | Orange |
| `WS2812_PURPLE` | Purple |

Colors are pre-dimmed to reasonable brightness levels.

## API Reference

### Initialization

| Function | Description |
|----------|-------------|
| `ws2812_init(gpio, &handle)` | Initialize LED on GPIO pin |
| `ws2812_deinit(handle)` | Free resources |

### Color Control

| Function | Description |
|----------|-------------|
| `ws2812_set(handle, color)` | Set predefined color |
| `ws2812_set_rgb(handle, r, g, b)` | Set custom RGB (0-255 each) |

### Utility

| Function | Description |
|----------|-------------|
| `ws2812_color_name(color)` | Get color name string (for logging) |

## Usage Examples

### Set Predefined Color

```c
ws2812_handle_t led;
ws2812_init(GPIO_NUM_48, &led);

ws2812_set(led, WS2812_GREEN);   // Status OK
ws2812_set(led, WS2812_RED);     // Error
ws2812_set(led, WS2812_BLUE);    // Processing
ws2812_set(led, WS2812_OFF);     // Turn off
```

### Set Custom Color

```c
// Set to 50% brightness white
ws2812_set_rgb(led, 128, 128, 128);

// Set to dim orange
ws2812_set_rgb(led, 64, 32, 0);
```

### Logging with Color Name

```c
ws2812_color_t status = WS2812_GREEN;
ws2812_set(led, status);
ESP_LOGI(TAG, "LED set to %s", ws2812_color_name(status));
// Output: LED set to green
```

### Multiple LEDs

```c
ws2812_handle_t led1, led2;
ws2812_init(GPIO_NUM_48, &led1);
ws2812_init(GPIO_NUM_38, &led2);

ws2812_set(led1, WS2812_RED);
ws2812_set(led2, WS2812_GREEN);
```

## Hardware Notes

- WS2812 LEDs require a 5V supply but can often be driven with 3.3V data
- For reliable operation with 3.3V logic, keep wire length short or use a level shifter
- Add a 300-500 ohm resistor in series with the data line to prevent ringing
- A 100uF capacitor across the LED power pins helps with power stability

## Wiring

```
ESP32 GPIO ----[330R]---- WS2812 DIN
ESP32 GND  -------------- WS2812 GND
5V/3.3V    -------------- WS2812 VDD
```

## TODO

- [ ] Add support for LED strips (multiple LEDs per handle)
- [ ] Add brightness scaling function
- [ ] Add fade/transition effects
- [ ] Add blink/pulse patterns

## References

- [WS2812B Datasheet](https://www.lcsc.com/datasheet/C4154872.pdf)
- [ESP-IDF RMT Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
