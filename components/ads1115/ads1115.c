#include "ads1115.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ADS1115";

/*
 * Power-on default for the config register: 0x8583
 * OS=1, MUX=000(diff 0-1), PGA=010(+/-2.048V), MODE=1(single-shot),
 * DR=100(128SPS), COMP_MODE=0, COMP_POL=0, COMP_LAT=0, COMP_QUE=11(disabled)
 */
#define ADS1115_CONFIG_DEFAULT 0x8583

/* Internal device state (one per instance) */
struct ads1115_dev {
    bool initialized;
    uint8_t i2c_addr;
    uint32_t clk_speed_hz;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    uint16_t config; /* cached config register */
};

/* ---------------------------------------------------------------------------
 * Low-level I2C helpers (ADS1115 uses 16-bit big-endian registers)
 * ---------------------------------------------------------------------------
 */
static esp_err_t ads1115_write_reg(ads1115_handle_t dev, uint8_t reg, uint16_t value) {
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value >> 8),   /* MSB first */
        (uint8_t)(value & 0xFF), /* LSB */
    };
    esp_err_t err = i2c_master_transmit(dev->dev_handle, buf, sizeof(buf), pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[0x%02X] write reg 0x%02X failed: %s",
                 dev->i2c_addr, reg, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t ads1115_read_reg(ads1115_handle_t dev, uint8_t reg, uint16_t *value) {
    uint8_t buf[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(
        dev->dev_handle, &reg, 1, buf, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[0x%02X] read reg 0x%02X failed: %s",
                 dev->i2c_addr, reg, esp_err_to_name(err));
        return err;
    }
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Initialization
 * ---------------------------------------------------------------------------
 */
esp_err_t ads1115_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr,
                       uint32_t clk_speed_hz, ads1115_handle_t *handle) {
    if (!bus_handle || !handle)
        return ESP_ERR_INVALID_ARG;

    *handle = NULL;

    struct ads1115_dev *dev = calloc(1, sizeof(*dev));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate device state");
        return ESP_ERR_NO_MEM;
    }

    dev->bus_handle = bus_handle;
    dev->i2c_addr = i2c_addr;
    dev->clk_speed_hz = clk_speed_hz;

    /* Add device to I2C bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = clk_speed_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[0x%02X] failed to add device to I2C bus: %s",
                 i2c_addr, esp_err_to_name(err));
        free(dev);
        return err;
    }

    /* Verify connectivity by reading the config register */
    uint16_t config;
    err = ads1115_read_reg(dev, ADS1115_REG_CONFIG, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[0x%02X] connectivity check failed - cannot read config register",
                 i2c_addr);
        i2c_master_bus_rm_device(dev->dev_handle);
        free(dev);
        return err;
    }

    /* OS bit (bit 15) should be 1 when device is idle */
    if (!(config & (1 << ADS1115_CFG_OS_BIT))) {
        ESP_LOGW(TAG, "[0x%02X] config=0x%04X (OS bit not set, unexpected)", i2c_addr, config);
    }

    dev->config = config;
    dev->initialized = true;

    ESP_LOGI(TAG, "Initialized at 0x%02X (config=0x%04X)", i2c_addr, config);

    *handle = dev;
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * De-initialization
 * ---------------------------------------------------------------------------
 */
esp_err_t ads1115_deinit(ads1115_handle_t handle) {
    if (!handle)
        return ESP_ERR_INVALID_ARG;

    if (handle->dev_handle) {
        i2c_master_bus_rm_device(handle->dev_handle);
    }

    ESP_LOGI(TAG, "Deinitialized device at 0x%02X", handle->i2c_addr);
    free(handle);
    return ESP_OK;
}
