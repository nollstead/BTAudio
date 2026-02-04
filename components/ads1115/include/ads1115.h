#ifndef ADS1115_H
#define ADS1115_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @brief Initialize the ADS1115 driver using an existing I2C bus.
 *
 * Use this when sharing the I2C bus with other devices (e.g., ES8388).
 * The bus handle is not deleted on deinit.
 *
 * @param bus_handle Existing I2C master bus handle.
 * @param i2c_addr 7-bit I2C address of the ADS1115.
 * @param clk_speed_hz I2C clock speed in Hz.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ads1115_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr, uint32_t clk_speed_hz);

#ifdef __cplusplus
}
#endif

#endif /* ADS1115_H */