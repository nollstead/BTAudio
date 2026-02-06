#ifndef ADC_MANAGER_H
#define ADC_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ads1115.h"
#include "esp_err.h"
#include <stdint.h>

/*
 * ADC Manager - Unified interface for multiple ADS1115 chips
 *
 * Presents 8 single-ended channels (0-7) mapped across two ADS1115 devices:
 *   Channels 0-3 → Chip 0 (ADS1115 at 0x48, pins A0-A3)
 *   Channels 4-7 → Chip 1 (ADS1115 at 0x49, pins A0-A3)
 *
 * Per-chip settings (gain, data rate) are configured by chip index (0 or 1)
 * because these settings are hardware-level and affect all 4 channels on
 * that chip simultaneously.
 */

#define ADC_CHANNEL_COUNT 8
#define ADC_CHIP_COUNT    2
#define ADC_CHANNELS_PER_CHIP 4

/**
 * @brief Initialize the ADC manager.
 *
 * Retrieves ADS1115 handles from the audio board and sets up channel mapping.
 * Call this after audio_board_init() has completed.
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no ADS1115 devices are available.
 */
esp_err_t adc_manager_init(void);

/**
 * @brief Read a single-ended ADC channel (0-7).
 *
 * Automatically routes to the correct ADS1115 chip and physical pin.
 *
 * @param channel  Virtual channel number (0-7).
 * @param[out] raw    Pointer to receive the raw 16-bit signed ADC value (may be NULL).
 * @param[out] volts  Pointer to receive the voltage in volts (may be NULL).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t adc_manager_read(uint8_t channel, int16_t *raw, float *volts);

/**
 * @brief Get the number of available channels.
 *
 * Returns the total number of usable channels based on how many chips
 * initialized successfully (0, 4, or 8).
 *
 * @return Number of available channels.
 */
uint8_t adc_manager_channel_count(void);

/**
 * @brief Check if a specific channel is available.
 *
 * @param channel  Virtual channel number (0-7).
 * @return true if the channel's underlying chip is initialized.
 */
bool adc_manager_channel_available(uint8_t channel);

/**
 * @brief Set the gain (PGA) for a specific chip.
 *
 * This affects all 4 channels on the chip simultaneously — it is a
 * hardware limitation of the ADS1115 (one PGA per chip, not per channel).
 *
 *   chip 0 → channels 0-3
 *   chip 1 → channels 4-7
 *
 * @param chip  Chip index (0 or 1).
 * @param gain  Gain setting.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t adc_manager_set_gain(uint8_t chip, ads1115_gain_t gain);

/**
 * @brief Get the gain (PGA) setting for a specific chip.
 *
 * @param chip  Chip index (0 or 1).
 * @return Current gain setting, or ADS1115_GAIN_2_048 on error.
 */
ads1115_gain_t adc_manager_get_gain(uint8_t chip);

/**
 * @brief Set the data rate for a specific chip.
 *
 * This affects all 4 channels on the chip simultaneously.
 *
 *   chip 0 → channels 0-3
 *   chip 1 → channels 4-7
 *
 * @param chip  Chip index (0 or 1).
 * @param rate  Data rate setting.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t adc_manager_set_rate(uint8_t chip, ads1115_rate_t rate);

/**
 * @brief Get the data rate setting for a specific chip.
 *
 * @param chip  Chip index (0 or 1).
 * @return Current data rate setting, or ADS1115_RATE_128 on error.
 */
ads1115_rate_t adc_manager_get_rate(uint8_t chip);

/**
 * @brief Enable or disable clamping of negative ADC values to zero.
 *
 * When enabled (the default), any negative raw count from the ADS1115 is
 * replaced with zero before being returned by adc_manager_read().
 * Disable for diagnostics when you need to see the true signed output.
 *
 * @param enabled  true to clamp negatives to zero, false to pass through.
 */
void adc_manager_set_clamp(bool enabled);

/**
 * @brief Get the current clamp setting.
 *
 * @return true if negative clamping is enabled.
 */
bool adc_manager_get_clamp(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_MANAGER_H */
