/*
 * Custom Board Pin Definitions
 *
 * IMPORTANT: Update these pin definitions to match your custom board wiring!
 * Current defaults are based on LyraT v4.3 - change as needed.
 */

#ifndef _AUDIO_BOARD_DEFINITION_H_
#define _AUDIO_BOARD_DEFINITION_H_

#include "driver/gpio.h"

// #define LyraT

/*
 * =============================================================================
 * I2C Configuration (for ES8388 codec control)
 * =============================================================================
 */

/*
 * ES8388 I2C Address Override
 * CE/AD0 pin tied to 3.3V = address 0x22 (8-bit) / 0x11 (7-bit)
 * Default ESP-ADF uses 0x20 (CE=0), we need 0x22 (CE=1)
 */
// #define ES8388_ADDR 0x22

#define BOARD_I2C_SDA_PIN GPIO_NUM_18
#define BOARD_I2C_SCL_PIN GPIO_NUM_23

/*
 * =============================================================================
 * I2S Configuration (for audio data)
 * =============================================================================
 */
#define BOARD_I2S_MCLK_PIN GPIO_NUM_0
#define BOARD_I2S_BCLK_PIN GPIO_NUM_5
#define BOARD_I2S_LRCLK_PIN GPIO_NUM_25
#define BOARD_I2S_DOUT_PIN GPIO_NUM_26
#ifdef LyraT
#define BOARD_I2S_DIN_PIN GPIO_NUM_35
#else
#define BOARD_I2S_DIN_PIN GPIO_NUM_19
#endif

#define BOARD_RGB_LED GPIO_NUM_2
/*
 * =============================================================================
 * Audio Codec Configuration
 * =============================================================================
 */
#define FUNC_AUDIO_CODEC_EN (1)
#define PA_ENABLE_GPIO GPIO_NUM_21
#define HEADPHONE_DETECT -1
#define AUXIN_DETECT_GPIO GPIO_NUM_12
#define CODEC_ADC_I2S_PORT ((i2s_port_t)0)
#define CODEC_ADC_BITS_PER_SAMPLE ((i2s_data_bit_width_t)16)
#define CODEC_ADC_SAMPLE_RATE (48000)
#define RECORD_HARDWARE_AEC (false)
#define BOARD_PA_GAIN (10)

/*
 * =============================================================================
 * SD Card Configuration (set to -1 if not used)
 * =============================================================================
 */
// #define FUNC_SDCARD_EN (1)
// #define SDCARD_OPEN_FILE_NUM_MAX 5
// #define SDCARD_INTR_GPIO GPIO_NUM_34

#define ESP_SD_PIN_CLK -1
#define ESP_SD_PIN_CMD -1
#define ESP_SD_PIN_D0 -1
#define ESP_SD_PIN_D1 -1
#define ESP_SD_PIN_D2 -1
#define ESP_SD_PIN_D3 -1

/*
 * =============================================================================
 * LED Configuration (set to -1 if not used)
 * =============================================================================
 */
// #define FUNC_SYS_LEN_EN (0)
// #define GREEN_LED_GPIO (-1)

/*
 * =============================================================================
 * Button Configuration
 * Since you don't need buttons, these are set to -1 (disabled)
 * =============================================================================
 */
// #define FUNC_BUTTON_EN (1)
// #define INPUT_KEY_NUM 6
#define BUTTON_REC_ID -1
#define BUTTON_MODE_ID -1
#define BUTTON_SET_ID -1
#define BUTTON_PLAY_ID -1
#define BUTTON_VOLUP_ID -1
#define BUTTON_VOLDOWN_ID -1

/*
 * =============================================================================
 * ES8388 Codec Driver Handle
 * This uses the ESP-ADF provided ES8388 driver
 * =============================================================================
 */
extern audio_hal_func_t AUDIO_CODEC_ES8388_DEFAULT_HANDLE;

#define AUDIO_CODEC_DEFAULT_CONFIG()                                                               \
    {                                                                                              \
        .adc_input = AUDIO_HAL_ADC_INPUT_LINE1,                                                    \
        .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,                                                    \
        .codec_mode = AUDIO_HAL_CODEC_MODE_BOTH,                                                   \
        .i2s_iface =                                                                               \
            {                                                                                      \
                .mode = AUDIO_HAL_MODE_SLAVE,                                                      \
                .fmt = AUDIO_HAL_I2S_NORMAL,                                                       \
                .samples = AUDIO_HAL_48K_SAMPLES,                                                  \
                .bits = AUDIO_HAL_BIT_LENGTH_16BITS,                                               \
            },                                                                                     \
    };

#endif
