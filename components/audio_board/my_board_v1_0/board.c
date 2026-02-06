/*
 * Custom Board Implementation for ESP-ADF
 * Uses ES8388 codec driver from ESP-ADF
 * Initializes BD37033 audio processor on shared I2C bus
 */

#include "board.h"
#include "ads1115.h"
#include "audio_mem.h"
#include "bd37033.h"
#include "driver/i2s.h"
#include "es8388.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "ws2812.h"
#include <periph_button.h>
#include <periph_touch.h>
#include <stdbool.h>

static const char *TAG = "MY_BOARD";

static audio_board_handle_t board_handle = NULL;
static bool bd37033_available = false;
static ads1115_handle_t ads1115_handle_1 = NULL;
static ads1115_handle_t ads1115_handle_2 = NULL;
static ws2812_handle_t led_handle = NULL;

audio_board_handle_t audio_board_init(void) {
    if (board_handle) {
        return board_handle;
    }

    board_handle = (audio_board_handle_t)audio_calloc(1, sizeof(struct audio_board_handle));
    AUDIO_MEM_CHECK(TAG, board_handle, return NULL);
    board_handle->audio_hal = audio_board_codec_init();

    // Initialize WS2812 status LED
    esp_err_t led_err = ws2812_init(BOARD_RGB_LED, &led_handle);
    if (led_err == ESP_OK) {
        ESP_LOGI(TAG, "WS2812 LED initialized on GPIO%d", BOARD_RGB_LED);
    } else {
        ESP_LOGW(TAG, "WS2812 LED init failed: %s (continuing without it)", esp_err_to_name(led_err));
    }

    // Initialize BD37033 audio processor using the shared I2C bus
    // The I2C bus was created by audio_hal_init() for ES8388
    i2c_master_bus_handle_t i2c_bus = i2c_bus_get_master_handle(I2C_NUM_0);
    if (i2c_bus != NULL) {
        esp_err_t err = bd37033_init(i2c_bus, BD37033_I2C_ADDR_DEFAULT, 100000);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "BD37033 initialized on shared I2C bus");
            bd37033_available = true;
            // Configure all registers for basic pass-through operation
            bd37033_setup_defaults();
        } else {
            ESP_LOGW(TAG, "BD37033 init failed: %s (continuing without it)", esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "Could not get I2C bus handle for BD37033");
    }

    // Initialize ADS1115 ADCs on shared I2C bus
    if (i2c_bus != NULL) {
        esp_err_t err = ads1115_init(i2c_bus, ADS1115_ADDR_GND, 100000, &ads1115_handle_1);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "ADS1115 #1 initialized at 0x%02X", ADS1115_ADDR_GND);
        } else {
            ESP_LOGW(TAG, "ADS1115 #1 init failed: %s (continuing without it)",
                     esp_err_to_name(err));
        }

        err = ads1115_init(i2c_bus, ADS1115_ADDR_VDD, 100000, &ads1115_handle_2);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "ADS1115 #2 initialized at 0x%02X", ADS1115_ADDR_VDD);
        } else {
            ESP_LOGW(TAG, "ADS1115 #2 init failed: %s (continuing without it)",
                     esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "Could not get I2C bus handle for ADS1115");
    }

    return board_handle;
}

audio_hal_handle_t audio_board_codec_init(void) {
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();

    // Your headphones are on LOUT2/ROUT2
    // LINE2 should route DAC to LOUT2/ROUT2
    audio_codec_cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;

    audio_hal_handle_t codec_hal =
        audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8388_DEFAULT_HANDLE);

    AUDIO_NULL_CHECK(TAG, codec_hal, return NULL); // Check before using the handle

    // Start DAC path, then unmute and set volume via the generic HAL
    audio_hal_ctrl_codec(codec_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
    audio_hal_set_mute(codec_hal, false);
    audio_hal_set_volume(codec_hal, 80);

    // ES8388 output volume register values:
    //   0x00 = -45dB (min), 0x12 = -6dB, 0x18 = -3dB, 0x1E = 0dB, 0x21 = +4.5dB (max)
    //
    // LOUT1/ROUT1: Line output (for powered speakers)
    es8388_write_reg(0x2E, 0x1E); // DACCONTROL24: LOUT1 volume = 0dB
    es8388_write_reg(0x2F, 0x1E); // DACCONTROL25: ROUT1 volume = 0dB
    //
    // LOUT2/ROUT2: Headphone output (ESP-ADF defaults to -30dB, override to 0dB)
    es8388_write_reg(0x30, 0x1E); // DACCONTROL26: LOUT2 volume = 0dB
    es8388_write_reg(0x31, 0x1E); // DACCONTROL27: ROUT2 volume = 0dB

    return codec_hal;
}

// ===== Codec HAL init (ES8388) =====

audio_board_handle_t audio_board_get_handle(void) {
    return board_handle;
}

bool audio_board_bd37033_available(void) {
    return bd37033_available;
}

esp_err_t audio_board_deinit(audio_board_handle_t audio_board) {

    esp_err_t ret = ESP_OK;
    ret = audio_hal_deinit(audio_board->audio_hal);
    audio_free(audio_board);
    board_handle = NULL;
    return ret;
}

ads1115_handle_t audio_board_get_ads1115(int index) {
    if (index == 0)
        return ads1115_handle_1;
    if (index == 1)
        return ads1115_handle_2;
    return NULL;
}

// ===== LED control =====

esp_err_t audio_board_led_set(ws2812_color_t color) {
    if (!led_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    return ws2812_set(led_handle, color);
}

esp_err_t audio_board_led_set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    if (!led_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    return ws2812_set_rgb(led_handle, red, green, blue);
}