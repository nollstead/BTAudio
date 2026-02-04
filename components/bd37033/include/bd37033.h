#ifndef BD37033_H
#define BD37033_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file bd37033.h
 * @brief ESP-IDF driver interface for the BD37033FV audio processor.
 *
 * This header declares the public API for configuring and controlling
 * the BD37033FV via I2C on ESP32-class devices.
 *
 * Uses the new i2c_master API (ESP-IDF 5.x+).
 */

/* Register addresses (from datasheet page 11) */
#define BD37033_INITIAL_SETUP   0x01
#define BD37033_LPF_SETUP       0x02
#define BD37033_MIXING_SETUP    0x03
#define BD37033_INPUT_SELECT    0x05
#define BD37033_INPUT_GAIN      0x06
#define BD37033_VOLUME_GAIN     0x20
#define BD37033_FADER_1_FRONT   0x28
#define BD37033_FADER_2_FRONT   0x29
#define BD37033_FADER_1_REAR    0x2A
#define BD37033_FADER_2_REAR    0x2B
#define BD37033_FADER_SUB_1     0x2C
#define BD37033_FADER_SUB_2     0x30
#define BD37033_BASS_SETUP      0x41
#define BD37033_MIDDLE_SETUP    0x44
#define BD37033_TREBLE_SETUP    0x47
#define BD37033_BASS_GAIN       0x51
#define BD37033_MIDDLE_GAIN     0x54
#define BD37033_TREBLE_GAIN     0x57
#define BD37033_LOUDNESS_GAIN   0x75

/**
 * @brief I2C address of the BD37033FV (7-bit).
 */
#define BD37033_I2C_ADDR_DEFAULT 0x40

/**
 * @brief BD37033 channel identifiers for fader/balance operations.
 */
typedef enum {
    BD37033_CH_FRONT_LEFT = 0,
    BD37033_CH_FRONT_RIGHT,
    BD37033_CH_REAR_LEFT,
    BD37033_CH_REAR_RIGHT,
} bd37033_channel_t;

/**
 * @brief BD37033 input selector.
 *
 * The actual mapping depends on your board's wiring to the BD37033FV inputs.
 */
typedef enum {
    BD37033_INPUT_1 = 0,
    BD37033_INPUT_2,
    BD37033_INPUT_3,
    BD37033_INPUT_4,
} bd37033_input_t;

/**
 * @brief BD37033 loudness mode.
 */
typedef enum {
    BD37033_LOUDNESS_OFF = 0,
    BD37033_LOUDNESS_ON,
} bd37033_loudness_t;

/**
 * @brief BD37033 mute state.
 */
typedef enum {
    BD37033_MUTE_OFF = 0,
    BD37033_MUTE_ON,
} bd37033_mute_t;

/**
 * @brief Initialize the BD37033 driver using an existing I2C bus.
 *
 * Use this when sharing the I2C bus with other devices (e.g., ES8388).
 * The bus handle is not deleted on deinit.
 *
 * @param bus_handle Existing I2C master bus handle.
 * @param i2c_addr 7-bit I2C address of the BD37033.
 * @param clk_speed_hz I2C clock speed in Hz.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr, uint32_t clk_speed_hz);

/**
 * @brief Configure chip with sensible defaults for basic operation.
 *
 * Sets up all registers for pass-through audio with flat EQ.
 * Call this after bd37033_init() to get sound output.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_setup_defaults(void);

/**
 * @brief Deinitialize the BD37033 driver.
 *
 * Removes the device from the bus.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_deinit(void);

/**
 * @brief Set the master volume.
 *
 * Typical range is 0 dB down to -79 dB (will be clamped).
 *
 * @param vol_db Volume in dB (0 = max, negative values = attenuation).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_set_volume(int vol_db);

/**
 * @brief Set input gain for the selected input.
 *
 * @param gain_db Gain in dB (-12 to +12, will be clamped).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_set_input_gain(int gain_db);

/**
 * @brief Select the active input.
 *
 * @param input Input selector value.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_select_input(bd37033_input_t input);

/**
 * @brief Set mute state.
 *
 * @param mute Mute on/off.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_set_mute(bd37033_mute_t mute);

/**
 * @brief Set per-channel fader/attenuation.
 *
 * @param ch Channel identifier.
 * @param att_db Attenuation in dB (0 = no attenuation, positive values = more attenuation).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_set_channel_attenuation(bd37033_channel_t ch, int att_db);

/**
 * @brief Set bass tone control.
 *
 * @param bass_db Bass adjustment in dB (positive = boost, negative = cut).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_set_bass(int bass_db);

/**
 * @brief Set treble tone control.
 *
 * @param treble_db Treble adjustment in dB (positive = boost, negative = cut).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_set_treble(int treble_db);

/**
 * @brief Configure loudness.
 *
 * @param mode Loudness on/off.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_set_loudness(bd37033_loudness_t mode);

/**
 * @brief Read a register value.
 *
 * @param reg Register address.
 * @param value Pointer to store the read value.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033_read_register(uint8_t reg, uint8_t *value);

/**
 * @brief Test the BD37033 by probing and writing to registers.
 *
 * @return ESP_OK if all tests pass, or an error code on failure.
 */
esp_err_t bd37033_test(void);

#ifdef __cplusplus
}
#endif

#endif /* BD37033_H */
