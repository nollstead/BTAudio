/*
 * Custom Board Pin Configuration
 * Provides pin accessor functions for ESP-ADF
 */

#include "audio_error.h"
#include "audio_mem.h"
#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MY_BOARD_PINS";

esp_err_t get_i2c_pins(i2c_port_t port, i2c_config_t *i2c_config) {
    AUDIO_NULL_CHECK(TAG, i2c_config, return ESP_FAIL);

    memset(i2c_config, 0, sizeof(*i2c_config));
    i2c_config->mode = I2C_MODE_MASTER; // HARMLESS EVEN IF THE CALLER IGNORES IT

    if (port == I2C_NUM_0 || port == I2C_NUM_1) {
        i2c_config->sda_io_num = BOARD_I2C_SDA_PIN;
        i2c_config->scl_io_num = BOARD_I2C_SCL_PIN;
        i2c_config->sda_pullup_en = GPIO_PULLUP_DISABLE; // External pullups present
        i2c_config->scl_pullup_en = GPIO_PULLUP_DISABLE; // External pullups present
        // Optional: a reference clock; callers using the legacy api might read it
        i2c_config->master.clk_speed = 400000; // 400kHz
    } else {
        i2c_config->sda_io_num = -1;
        i2c_config->scl_io_num = -1;
        ESP_LOGE(TAG, "i2c port %d is not supported", port);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t get_i2s_pins(int port, board_i2s_pin_t *i2s_config) {
    AUDIO_NULL_CHECK(TAG, i2s_config, return ESP_FAIL);
    if (port == 0 || port == 1) {
        i2s_config->mck_io_num = BOARD_I2S_MCLK_PIN;
        i2s_config->bck_io_num = BOARD_I2S_BCLK_PIN;
        i2s_config->ws_io_num = BOARD_I2S_LRCLK_PIN;
        i2s_config->data_out_num = BOARD_I2S_DOUT_PIN;
        i2s_config->data_in_num = BOARD_I2S_DIN_PIN;
    } else {
        memset(i2s_config, -1, sizeof(board_i2s_pin_t));
        ESP_LOGE(TAG, "i2s port %d is not supported", port);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t get_spi_pins(spi_bus_config_t *spi_config,
                       spi_device_interface_config_t *spi_device_interface_config) {
    AUDIO_NULL_CHECK(TAG, spi_config, return ESP_FAIL);
    AUDIO_NULL_CHECK(TAG, spi_device_interface_config, return ESP_FAIL);

    spi_config->mosi_io_num = -1;
    spi_config->miso_io_num = -1;
    spi_config->sclk_io_num = -1;
    spi_config->quadwp_io_num = -1;
    spi_config->quadhd_io_num = -1;

    spi_device_interface_config->spics_io_num = -1;

    ESP_LOGW(TAG, "SPI interface is not supported");
    return ESP_OK;
}

// int8_t get_sdcard_intr_gpio(void) {
//     return SDCARD_INTR_GPIO;
// }

// int8_t get_sdcard_open_file_num_max(void) {
//     return SDCARD_OPEN_FILE_NUM_MAX;
// }

// int8_t get_sdcard_power_ctrl_gpio(void) {
//     return SDCARD_PWR_CTRL;
// }

// int8_t get_auxin_detect_gpio(void) {
//     return AUXIN_DETECT_GPIO;
// }

int8_t get_headphone_detect_gpio(void) {
    return HEADPHONE_DETECT;
}

int8_t get_pa_enable_gpio(void) {
    return -1;
}

int8_t get_adc_detect_gpio(void) {
    return -1;
}

int8_t get_es7243_mclk_gpio(void) {
    return -1;
}

int8_t get_input_rec_id(void) {
    return -1;
}

int8_t get_input_mode_id(void) {
    return BUTTON_MODE_ID;
}

int8_t get_input_set_id(void) {
    return BUTTON_SET_ID;
}

int8_t get_input_play_id(void) {
    return BUTTON_PLAY_ID;
}

int8_t get_input_volup_id(void) {
    return BUTTON_VOLUP_ID;
}

int8_t get_input_voldown_id(void) {
    return BUTTON_VOLDOWN_ID;
}

// int8_t get_input_color_id(void) {
//     return -1;
// }

// int8_t get_reset_codec_gpio(void) {
//     return -1;
// }

int8_t get_reset_board_gpio(void) {
    return -1;
}

// int8_t get_green_led_gpio(void) {
//     return GREEN_LED_GPIO;
// }

// int8_t get_blue_led_gpio(void) {
//     return -1;
// }