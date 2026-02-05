#ifndef ADS1115_H
#define ADS1115_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

/* I2C addresses (7-bit) - determined by ADDR pin wiring */
#define ADS1115_ADDR_GND 0x48 /* ADDR pin -> GND */
#define ADS1115_ADDR_VDD 0x49 /* ADDR pin -> VDD */
#define ADS1115_ADDR_SDA 0x4A /* ADDR pin -> SDA */
#define ADS1115_ADDR_SCL 0x4B /* ADDR pin -> SCL */

/* Register pointer addresses */
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01
#define ADS1115_REG_LO_THRESH  0x02
#define ADS1115_REG_HI_THRESH  0x03

/* Config register bit positions */
#define ADS1115_CFG_OS_BIT        15
#define ADS1115_CFG_MUX_SHIFT     12
#define ADS1115_CFG_MUX_MASK      (0x07 << ADS1115_CFG_MUX_SHIFT)
#define ADS1115_CFG_PGA_SHIFT      9
#define ADS1115_CFG_PGA_MASK      (0x07 << ADS1115_CFG_PGA_SHIFT)
#define ADS1115_CFG_MODE_BIT       8
#define ADS1115_CFG_DR_SHIFT       5
#define ADS1115_CFG_DR_MASK       (0x07 << ADS1115_CFG_DR_SHIFT)
#define ADS1115_CFG_COMP_QUE_SHIFT 0
#define ADS1115_CFG_COMP_QUE_MASK (0x03 << ADS1115_CFG_COMP_QUE_SHIFT)

/*
 * Programmable Gain Amplifier (PGA) settings
 *
 * The PGA controls the full-scale input voltage range of the ADC.
 * Choose a gain setting where your maximum expected input voltage is
 * below the full-scale range, but as close as possible for best resolution.
 *
 * IMPORTANT: The full-scale range is the maximum voltage the ADC can measure,
 * NOT the maximum safe input voltage. The ADS1115 inputs are protected up to
 * VDD + 0.3V regardless of gain setting.
 *
 * For a 3.3V system with inputs from 0-3.3V (e.g., potentiometers):
 *   - GAIN_6_144 works but wastes ~half the ADC range (only uses ~0-3.3V of 0-6.144V)
 *   - GAIN_4_096 is optimal: full 3.3V range fits, maximizing resolution
 *
 * Resolution per LSB at each gain setting:
 *   GAIN_6_144: 187.5 uV/bit  (coarsest, for high voltage signals)
 *   GAIN_4_096: 125.0 uV/bit  (good for 3.3V full-scale)
 *   GAIN_2_048: 62.5 uV/bit   (default, for 0-2V signals)
 *   GAIN_1_024: 31.25 uV/bit
 *   GAIN_0_512: 15.625 uV/bit
 *   GAIN_0_256: 7.8125 uV/bit (finest, for small differential signals)
 */
typedef enum {
    ADS1115_GAIN_6_144 = 0x00, /* +/-6.144V range, 187.5uV/bit */
    ADS1115_GAIN_4_096 = 0x01, /* +/-4.096V range, 125uV/bit   */
    ADS1115_GAIN_2_048 = 0x02, /* +/-2.048V range, 62.5uV/bit (default) */
    ADS1115_GAIN_1_024 = 0x03, /* +/-1.024V range, 31.25uV/bit */
    ADS1115_GAIN_0_512 = 0x04, /* +/-0.512V range, 15.625uV/bit */
    ADS1115_GAIN_0_256 = 0x05, /* +/-0.256V range, 7.8125uV/bit */
} ads1115_gain_t;

/*
 * Data Rate settings (samples per second)
 *
 * Lower data rates provide better noise rejection (more averaging),
 * while higher rates allow faster sampling but with more noise.
 *
 * Conversion time = 1/rate + overhead (~1ms)
 *   RATE_8:   ~126ms per conversion (best noise rejection)
 *   RATE_128: ~8ms per conversion (default, good balance)
 *   RATE_860: ~1.2ms per conversion (fastest, most noise)
 */
typedef enum {
    ADS1115_RATE_8   = 0x00, /* 8 SPS,   ~125ms conversion */
    ADS1115_RATE_16  = 0x01, /* 16 SPS,  ~62.5ms conversion */
    ADS1115_RATE_32  = 0x02, /* 32 SPS,  ~31.25ms conversion */
    ADS1115_RATE_64  = 0x03, /* 64 SPS,  ~15.6ms conversion */
    ADS1115_RATE_128 = 0x04, /* 128 SPS, ~7.8ms conversion (default) */
    ADS1115_RATE_250 = 0x05, /* 250 SPS, ~4ms conversion */
    ADS1115_RATE_475 = 0x06, /* 475 SPS, ~2.1ms conversion */
    ADS1115_RATE_860 = 0x07, /* 860 SPS, ~1.2ms conversion */
} ads1115_rate_t;

/* Input multiplexer settings */
typedef enum {
    ADS1115_MUX_DIFF_0_1   = 0x00, /* AIN0 - AIN1 (default) */
    ADS1115_MUX_DIFF_0_3   = 0x01, /* AIN0 - AIN3 */
    ADS1115_MUX_DIFF_1_3   = 0x02, /* AIN1 - AIN3 */
    ADS1115_MUX_DIFF_2_3   = 0x03, /* AIN2 - AIN3 */
    ADS1115_MUX_SINGLE_0   = 0x04, /* AIN0 vs GND */
    ADS1115_MUX_SINGLE_1   = 0x05, /* AIN1 vs GND */
    ADS1115_MUX_SINGLE_2   = 0x06, /* AIN2 vs GND */
    ADS1115_MUX_SINGLE_3   = 0x07, /* AIN3 vs GND */
} ads1115_mux_t;

/* Comparator mode */
typedef enum {
    ADS1115_COMP_MODE_TRADITIONAL = 0, /* Traditional comparator with hysteresis */
    ADS1115_COMP_MODE_WINDOW      = 1, /* Window comparator */
} ads1115_comp_mode_t;

/* Comparator polarity */
typedef enum {
    ADS1115_COMP_POL_LOW  = 0, /* ALERT pin active low (default) */
    ADS1115_COMP_POL_HIGH = 1, /* ALERT pin active high */
} ads1115_comp_pol_t;

/* Comparator latching */
typedef enum {
    ADS1115_COMP_LAT_OFF = 0, /* Non-latching (default) */
    ADS1115_COMP_LAT_ON  = 1, /* Latching - ALERT stays until data read */
} ads1115_comp_lat_t;

/* Comparator queue (number of conversions before ALERT asserts) */
typedef enum {
    ADS1115_COMP_QUE_1    = 0x00, /* Assert after 1 conversion */
    ADS1115_COMP_QUE_2    = 0x01, /* Assert after 2 conversions */
    ADS1115_COMP_QUE_4    = 0x02, /* Assert after 4 conversions */
    ADS1115_COMP_QUE_NONE = 0x03, /* Disable comparator (default) */
} ads1115_comp_que_t;

/* Opaque handle for an ADS1115 instance */
typedef struct ads1115_dev *ads1115_handle_t;

/**
 * @brief Initialize an ADS1115 device on an existing I2C bus.
 *
 * Adds the device to the bus, verifies connectivity by reading the
 * config register, and returns a handle for subsequent operations.
 *
 * @param bus_handle    Existing I2C master bus handle.
 * @param i2c_addr      7-bit I2C address (ADS1115_ADDR_GND .. ADS1115_ADDR_SCL).
 * @param clk_speed_hz  I2C clock speed in Hz for this device.
 * @param[out] handle   Pointer to receive the allocated device handle.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr,
                       uint32_t clk_speed_hz, ads1115_handle_t *handle);

/**
 * @brief Deinitialize an ADS1115 device.
 *
 * Removes the device from the I2C bus and frees the handle.
 *
 * @param handle  Device handle returned by ads1115_init().
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_deinit(ads1115_handle_t handle);

/**
 * @brief Set the programmable gain amplifier (PGA) setting.
 *
 * The gain determines the full-scale input voltage range. Choose the smallest
 * range that accommodates your maximum expected input voltage for best resolution.
 *
 * @param handle  Device handle.
 * @param gain    Gain setting (see ads1115_gain_t documentation).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_set_gain(ads1115_handle_t handle, ads1115_gain_t gain);

/**
 * @brief Get the current PGA gain setting.
 *
 * @param handle  Device handle.
 * @return Current gain setting.
 */
ads1115_gain_t ads1115_get_gain(ads1115_handle_t handle);

/**
 * @brief Read a single-ended ADC value from the specified channel.
 *
 * Performs a single-shot conversion on one of the four input channels
 * (AIN0-AIN3) measured against GND. Blocks until conversion completes.
 *
 * @param handle   Device handle.
 * @param channel  Channel number (0-3).
 * @param[out] value  Pointer to receive the raw 16-bit signed ADC value.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_read_single_ended(ads1115_handle_t handle, uint8_t channel, int16_t *value);

/**
 * @brief Convert a raw ADC value to voltage.
 *
 * Uses the current gain setting to calculate the actual voltage.
 *
 * @param handle  Device handle.
 * @param counts  Raw ADC value from ads1115_read_single_ended().
 * @return Voltage in volts.
 */
float ads1115_compute_volts(ads1115_handle_t handle, int16_t counts);

/**
 * @brief Set the data rate (samples per second).
 *
 * Lower rates provide better noise rejection, higher rates allow faster reads.
 *
 * @param handle  Device handle.
 * @param rate    Data rate setting.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_set_rate(ads1115_handle_t handle, ads1115_rate_t rate);

/**
 * @brief Get the current data rate setting.
 *
 * @param handle  Device handle.
 * @return Current data rate setting.
 */
ads1115_rate_t ads1115_get_rate(ads1115_handle_t handle);

/**
 * @brief Read a differential ADC value.
 *
 * Performs a single-shot conversion measuring the voltage difference
 * between two input pins. Result can be positive or negative.
 *
 * @param handle  Device handle.
 * @param mux     Differential pair (ADS1115_MUX_DIFF_0_1, etc.).
 * @param[out] value  Pointer to receive the raw 16-bit signed ADC value.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_read_differential(ads1115_handle_t handle, ads1115_mux_t mux, int16_t *value);

/**
 * @brief Get the I2C address of this device.
 *
 * Useful for logging when managing multiple ADS1115 devices.
 *
 * @param handle  Device handle.
 * @return 7-bit I2C address, or 0 on error.
 */
uint8_t ads1115_get_address(ads1115_handle_t handle);

/**
 * @brief Configure the comparator thresholds.
 *
 * The comparator monitors the ADC result and asserts the ALERT pin when
 * the value crosses the threshold(s). Use this to trigger hardware interrupts
 * when a voltage exceeds or falls below a set point.
 *
 * In traditional mode: ALERT asserts when value > hi_thresh, clears when < lo_thresh.
 * In window mode: ALERT asserts when value < lo_thresh OR value > hi_thresh.
 *
 * Thresholds are raw ADC counts (same scale as read functions return).
 * To convert voltage to counts: counts = voltage / volts_per_bit
 *   where volts_per_bit depends on gain setting.
 *
 * @param handle     Device handle.
 * @param lo_thresh  Low threshold in raw ADC counts.
 * @param hi_thresh  High threshold in raw ADC counts.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_set_comparator_thresholds(ads1115_handle_t handle,
                                            int16_t lo_thresh, int16_t hi_thresh);

/**
 * @brief Configure comparator behavior.
 *
 * @param handle    Device handle.
 * @param mode      Traditional (hysteresis) or window mode.
 * @param polarity  ALERT pin active low or high.
 * @param latching  Whether ALERT latches until data is read.
 * @param queue     Number of conversions exceeding threshold before ALERT.
 *                  Use ADS1115_COMP_QUE_NONE to disable the comparator.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_set_comparator_config(ads1115_handle_t handle,
                                        ads1115_comp_mode_t mode,
                                        ads1115_comp_pol_t polarity,
                                        ads1115_comp_lat_t latching,
                                        ads1115_comp_que_t queue);

#ifdef __cplusplus
}
#endif

#endif /* ADS1115_H */
