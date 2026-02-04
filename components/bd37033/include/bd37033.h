#ifndef BD37033FV_H
#define BD37033FV_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BD37033FV_INITIAL_SETUP 0x01
#define BD37033FV_LPF_SETUP 0x02
#define BD37033FV_MIXING_SETUP 0x03
#define BD37033FV_INPUT_SELECT 0x05
#define BD37033FV_INPUT_GAIN 0x06
#define BD37033FV_VOLUME_GAIN 0x20
#define BD37033FV_FADER_1_FRONT 0x28
#define BD37033FV_FADER_2_FRONT 0x29
#define BD37033FV_FADER_1_REAR 0x2A
#define BD37033FV_FADER_2_REAR 0x2B
#define BD37033FV_FADER_SUB_1 0x2C
#define BD37033FV_FADER_SUB_2 0x30
#define BD37033FV_BASS_SETUP 0x41
#define BD37033FV_MIDDLE_SETUP 0x44
#define BD37033FV_TREBLE_SETUP 0x47
#define BD37033FV_BASS_GAIN 0x51
#define BD37033FV_MIDDLE_GAIN 0x54
#define BD37033FV_TREBLE_GAIN 0x57
#define BD37033FV_LOUDNESS_GAIN 0x75

/**
 * @file bd37033fv.h
 * @brief ESP-IDF driver interface for the BD37033FV audio processor.
 *
 * This header declares the public API for configuring and controlling
 * the BD37033FV via I2C on ESP32-class devices.
 *
 * Uses the new i2c_master API (ESP-IDF 5.x+).
 */

/**
 * @brief I2C address of the BD37033FV (7-bit).
 *
 */
#define BD37033FV_I2C_ADDR_DEFAULT 0x40

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
 * The actual mapping depends on your board's wiring to the BD37033FV inputs.
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
 * @brief I2C configuration for BD37033FV driver (new i2c_master API).
 */
typedef struct {
    i2c_port_num_t port;   /*!< I2C port number */
    gpio_num_t sda;        /*!< SDA GPIO */
    gpio_num_t scl;        /*!< SCL GPIO */
    uint32_t clk_speed_hz; /*!< I2C clock speed (e.g. 100000 or 400000) */
    uint8_t i2c_addr;      /*!< 7-bit I2C address; use BD37033FV_I2C_ADDR_DEFAULT if unsure */
} bd37033fv_i2c_config_t;

/**
 * @brief Initialize the BD37033FV driver and create a new I2C bus.
 *
 * Use this when BD37033FV is the only device on the bus.
 *
 * @param cfg Pointer to I2C configuration structure.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_init(const bd37033fv_i2c_config_t *cfg);

/**
 * @brief Initialize the BD37033FV driver using an existing I2C bus.
 *
 * Use this when sharing the I2C bus with other devices (e.g., ES8388).
 * The bus handle is not deleted on deinit.
 *
 * @param bus_handle Existing I2C master bus handle.
 * @param i2c_addr 7-bit I2C address of the BD37033FV.
 * @param clk_speed_hz I2C clock speed in Hz.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_init_with_bus(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr,
                                  uint32_t clk_speed_hz);

/**
 * @brief Deinitialize the BD37033FV driver.
 *
 * Removes the device from the bus and deletes the bus handle.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_deinit(void);

/**
 * @brief Set the master volume.
 *
 * Typical range for BD37033FV is around 0 dB down to -79 dB (implementation
 * will clamp to the device's valid range).
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

/**
 * @brief Read a register value.
 *
 * @param reg Register address.
 * @param value Pointer to store the read value.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bd37033fv_read_register(uint8_t reg, uint8_t *value);

/**
 * @brief Test the BD37033FV by writing and reading back register values.
 *
 * This function writes test values to registers and reads them back
 * to verify I2C communication is working correctly.
 *
 * @return ESP_OK if all tests pass, or an error code on failure.
 */
esp_err_t bd37033fv_test(void);

#ifdef __cplusplus
}
#endif

#endif /* BD37033FV_H */
