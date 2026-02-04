#ifndef BD37033FV_H
#define BD37033FV_H

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file bd37033fv.h
 * @brief ESP-IDF driver interface for the BD37033FV audio processor.
 *
 * This header declares the public API for configuring and controlling
 * the BD37033FV via I2C on ESP32-class devices.
 */

/**
 * @brief I2C address of the BD37033FV (7-bit).
 *
 * The BD37033FV typically uses 0x44 (depending on hardware configuration).
 * If your hardware differs, you can override this at init time if needed.
 */
#define BD37033FV_I2C_ADDR_DEFAULT 0x44

/**
 * @brief BD37033FV channel identifiers for fader/balance operations.
 */
typedef enum {
    BD37033FV_CH_FRONT_LEFT = 0,
    BD37033FV_CH_FRONT_RIGHT,
    BD37033FV_CH_REAR_LEFT,
    BD37033FV_CH_REAR_RIGHT,
} bd37033fv_channel_t;

/**
 * @brief BD37033FV input selector.
 *
 * The actual mapping depends on your board’s wiring to the BD37033FV inputs.
 */
typedef enum {
    BD37033FV_INPUT_1 = 0,
    BD37033FV_INPUT_2,
    BD37033FV_INPUT_3,
    BD37033FV_INPUT_4,
} bd37033fv_input_t;

/**
 * @brief BD37033FV loudness mode.
 */
typedef enum {
    BD37033FV_LOUDNESS_OFF = 0,
    BD37033FV_LOUDNESS_ON,
} bd37033fv_loudness_t;

/**
 * @brief BD37033FV mute state.
 */
typedef enum {
    BD37033FV_MUTE_OFF = 0,
    BD37033FV_MUTE_ON,
} bd37033fv_mute_t;

/**
 * @brief I2C configuration for BD37033FV driver.
 *
 * All hardware-specific details are provided by the application at init time.
 */
typedef struct {
    i2c_port_t port;       /*!< I2C port (e.g. I2C_NUM_0 or I2C_NUM_1) */
    gpio_num_t sda;        /*!< SDA GPIO */
    gpio_num_t scl;        /*!< SCL GPIO */
    uint32_t clk_speed_hz; /*!< I2C clock speed (e.g. 100000 or 400000) */
    uint8_t i2c_addr;      /*!< 7-bit I2C address; use BD37033FV_I2C_ADDR_DEFAULT if unsure */
} bd37033fv_i2c_config_t;

/**
 * @brief Initialize the BD37033FV driver and underlying I2C bus (if needed).
 *
 * This function:
 *  - Configures and installs the I2C driver for the specified port (if not already installed).
 *  - Stores the configuration internally for subsequent register accesses.
 *  - Performs the basic BD37033FV initialization sequence.
 *
 * @param cfg Pointer to I2C configuration structure.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_init(const bd37033fv_i2c_config_t *cfg);

/**
 * @brief Deinitialize the BD37033FV driver.
 *
 * This does not automatically uninstall the I2C driver, to avoid interfering
 * with other devices on the same bus. It only clears internal state.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_deinit(void);

/**
 * @brief Set the master volume.
 *
 * Typical range for BD37033FV is around 0 dB down to -79 dB (implementation
 * will clamp to the device’s valid range).
 *
 * @param vol_db Volume in dB (0 = max, negative values = attenuation).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_set_volume(int vol_db);

/**
 * @brief Set input gain for the selected input.
 *
 * @param gain_db Gain in dB (range depends on BD37033FV spec; will be clamped).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_set_input_gain(int gain_db);

/**
 * @brief Select the active input.
 *
 * @param input Input selector value.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_select_input(bd37033fv_input_t input);

/**
 * @brief Set mute state.
 *
 * @param mute Mute on/off.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_set_mute(bd37033fv_mute_t mute);

/**
 * @brief Set per-channel fader/attenuation.
 *
 * @param ch Channel identifier.
 * @param att_db Attenuation in dB (0 = no attenuation, positive values = more attenuation).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_set_channel_attenuation(bd37033fv_channel_t ch, int att_db);

/**
 * @brief Set bass tone control.
 *
 * @param bass_db Bass adjustment in dB (positive = boost, negative = cut).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_set_bass(int bass_db);

/**
 * @brief Set treble tone control.
 *
 * @param treble_db Treble adjustment in dB (positive = boost, negative = cut).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_set_treble(int treble_db);

/**
 * @brief Configure loudness.
 *
 * @param mode Loudness on/off.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_set_loudness(bd37033fv_loudness_t mode);

/**
 * @brief Commit any pending configuration if the driver batches writes.
 *
 * For a simple implementation this may be a no-op, but it is provided
 * for future extensibility.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_update(void);

#ifdef __cplusplus
}
#endif

#endif /* BD37033FV_H */