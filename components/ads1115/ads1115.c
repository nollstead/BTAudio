#include "ads1115.h"
#include "esp_log.h"

static const char *TAG = "ADS1115";
/* ---------------------------------------------------------------------------
 * Internal driver state
 * ---------------------------------------------------------------------------
 */
typedef struct {
    bool initialized;
    uint8_t i2c_addr;
    uint32_t clk_speed_hz;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} ads1115_state_t;

static ads1115_state_t g_state = {0};

esp_err_t ads1115_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr,
                       uint32_t clk_speed_hz) {

    if (!bus_handle)
        return ESP_ERR_INVALID_ARG;

    memset(&g_state, 0, sizeof(g_state));
    g_state.bus_handle = bus_handle;
    g_state.i2c_addr = i2c_addr;
    g_state.clk_speed_hz = clk_speed_hz;

    // Add adc1115 device to the existing bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = clk_speed_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &g_state.dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Initialized at I2C addr 0x%02X", i2c_addr);
    return ESP_OK;
}
