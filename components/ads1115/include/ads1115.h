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

/* Programmable gain amplifier (PGA) settings */
typedef enum {
    ADS1115_GAIN_6_144 = 0x00, /* +/-6.144V (2/3x gain) */
    ADS1115_GAIN_4_096 = 0x01, /* +/-4.096V (1x gain)   */
    ADS1115_GAIN_2_048 = 0x02, /* +/-2.048V (default)   */
    ADS1115_GAIN_1_024 = 0x03, /* +/-1.024V             */
    ADS1115_GAIN_0_512 = 0x04, /* +/-0.512V             */
    ADS1115_GAIN_0_256 = 0x05, /* +/-0.256V             */
} ads1115_gain_t;

/* Data rate settings (samples per second) */
typedef enum {
    ADS1115_RATE_8   = 0x00,
    ADS1115_RATE_16  = 0x01,
    ADS1115_RATE_32  = 0x02,
    ADS1115_RATE_64  = 0x03,
    ADS1115_RATE_128 = 0x04, /* default */
    ADS1115_RATE_250 = 0x05,
    ADS1115_RATE_475 = 0x06,
    ADS1115_RATE_860 = 0x07,
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

#ifdef __cplusplus
}
#endif

#endif /* ADS1115_H */
