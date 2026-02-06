/*
 * Custom Board Definition for ESP-ADF
 * Based on LyraT v4.3 structure for ES8388 codec
 */

#ifndef _AUDIO_BOARD_H_
#define _AUDIO_BOARD_H_

#include "ads1115.h"
#include "audio_hal.h"
#include "board_def.h"
#include "board_pins_config.h"
#include "display_service.h"
#include "esp_peripherals.h"
#include "ws2812.h"
#include <stdbool.h>
//  #include "periph_sdcard.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Audio board handle
 */
struct audio_board_handle {
    audio_hal_handle_t audio_hal;
};

typedef struct audio_board_handle *audio_board_handle_t;

audio_board_handle_t audio_board_init(void);
audio_hal_handle_t audio_board_codec_init(void);
// esp_err_t audio_board_key_init(esp_periph_set_handle_t set);
audio_board_handle_t audio_board_get_handle(void);
bool audio_board_bd37033_available(void);
ads1115_handle_t audio_board_get_ads1115(int index);
esp_err_t audio_board_deinit(audio_board_handle_t audio_board);
void audio_board_i2c_init(void);
void audio_board_i2s_init(void);
void board_codec_reset(void);

esp_err_t audio_board_led_set(ws2812_color_t color);
esp_err_t audio_board_led_set_rgb(uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif

#endif
