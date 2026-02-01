#ifndef WS2812_H
#define WS2812_H

#include "esp_err.h"
#include <stdint.h>

typedef struct ws2812 *ws2812_handle_t;

// Predefined colors (dimmed to reasonable brightness)
typedef enum {
    WS2812_OFF = 0,
    WS2812_RED,
    WS2812_GREEN,
    WS2812_BLUE,
    WS2812_WHITE,
    WS2812_YELLOW,
    WS2812_CYAN,
    WS2812_MAGENTA,
    WS2812_ORANGE,
    WS2812_PURPLE,
    WS2812_COLOR_COUNT
} ws2812_color_t;

/**
 * @brief Initialize a WS2812 LED on the specified GPIO
 * @param gpio_num GPIO pin connected to the LED data line
 * @param handle Pointer to store the handle
 * @return ESP_OK on success
 */
esp_err_t ws2812_init(int gpio_num, ws2812_handle_t *handle);

/**
 * @brief Set the LED to a predefined color
 * @param handle WS2812 handle from ws2812_init
 * @param color Predefined color from ws2812_color_t
 * @return ESP_OK on success
 */
esp_err_t ws2812_set(ws2812_handle_t handle, ws2812_color_t color);

/**
 * @brief Set the LED to a custom RGB color
 * @param handle WS2812 handle from ws2812_init
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @return ESP_OK on success
 */
esp_err_t ws2812_set_rgb(ws2812_handle_t handle, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Get the name of a predefined color (for logging)
 * @param color Predefined color from ws2812_color_t
 * @return Color name string
 */
const char *ws2812_color_name(ws2812_color_t color);

/**
 * @brief Deinitialize and free resources
 * @param handle WS2812 handle from ws2812_init
 * @return ESP_OK on success
 */
esp_err_t ws2812_deinit(ws2812_handle_t handle);

#endif // WS2812_H
