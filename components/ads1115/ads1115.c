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
    uint16_t config;       /* cached config register */
    ads1115_gain_t gain;   /* cached gain setting */
    ads1115_rate_t rate;   /* cached data rate setting */
    /* Comparator settings (cached) */
    ads1115_comp_mode_t comp_mode;
    ads1115_comp_pol_t comp_pol;
    ads1115_comp_lat_t comp_lat;
    ads1115_comp_que_t comp_que;
};

/* Conversion time in milliseconds for each data rate (approximate) */
static const uint16_t rate_conv_time_ms[] = {
    125, /* RATE_8 */
    63,  /* RATE_16 */
    32,  /* RATE_32 */
    16,  /* RATE_64 */
    8,   /* RATE_128 */
    4,   /* RATE_250 */
    3,   /* RATE_475 */
    2,   /* RATE_860 */
};

/* Full-scale voltage for each gain setting (in microvolts for precision) */
static const uint32_t gain_fs_uv[] = {
    6144000, /* GAIN_6_144: +/-6.144V */
    4096000, /* GAIN_4_096: +/-4.096V */
    2048000, /* GAIN_2_048: +/-2.048V */
    1024000, /* GAIN_1_024: +/-1.024V */
    512000,  /* GAIN_0_512: +/-0.512V */
    256000,  /* GAIN_0_256: +/-0.256V */
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
    dev->gain = (config & ADS1115_CFG_PGA_MASK) >> ADS1115_CFG_PGA_SHIFT;
    dev->rate = (config & ADS1115_CFG_DR_MASK) >> ADS1115_CFG_DR_SHIFT;
    dev->comp_que = ADS1115_COMP_QUE_NONE; /* disabled by default */
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

/* ---------------------------------------------------------------------------
 * Gain control
 * ---------------------------------------------------------------------------
 */
esp_err_t ads1115_set_gain(ads1115_handle_t handle, ads1115_gain_t gain) {
    if (!handle || !handle->initialized)
        return ESP_ERR_INVALID_STATE;
    if (gain > ADS1115_GAIN_0_256)
        return ESP_ERR_INVALID_ARG;

    handle->gain = gain;
    /* Gain is applied when starting a conversion, so just cache it */
    return ESP_OK;
}

ads1115_gain_t ads1115_get_gain(ads1115_handle_t handle) {
    if (!handle || !handle->initialized)
        return ADS1115_GAIN_2_048; /* return default on error */
    return handle->gain;
}

/* ---------------------------------------------------------------------------
 * Single-ended ADC read (single-shot mode)
 * ---------------------------------------------------------------------------
 */
esp_err_t ads1115_read_single_ended(ads1115_handle_t handle, uint8_t channel, int16_t *value) {
    if (!handle || !handle->initialized)
        return ESP_ERR_INVALID_STATE;
    if (channel > 3 || !value)
        return ESP_ERR_INVALID_ARG;

    /*
     * Build config register for single-shot conversion:
     * - OS = 1 (start conversion)
     * - MUX = 0x04 + channel (single-ended AINx vs GND)
     * - PGA = current gain setting
     * - MODE = 1 (single-shot)
     * - DR = 128 SPS (default)
     * - COMP_QUE = 0b11 (disable comparator)
     */
    uint16_t config = (1 << ADS1115_CFG_OS_BIT) |                         /* Start conversion */
                      ((ADS1115_MUX_SINGLE_0 + channel) << ADS1115_CFG_MUX_SHIFT) |
                      (handle->gain << ADS1115_CFG_PGA_SHIFT) |
                      (1 << ADS1115_CFG_MODE_BIT) |                        /* Single-shot */
                      (handle->rate << ADS1115_CFG_DR_SHIFT) |
                      (handle->comp_mode << 4) |                           /* COMP_MODE */
                      (handle->comp_pol << 3) |                            /* COMP_POL */
                      (handle->comp_lat << 2) |                            /* COMP_LAT */
                      (handle->comp_que << ADS1115_CFG_COMP_QUE_SHIFT);

    esp_err_t err = ads1115_write_reg(handle, ADS1115_REG_CONFIG, config);
    if (err != ESP_OK)
        return err;

    /*
     * Wait for conversion to complete.
     * Timeout is based on data rate setting with margin.
     */
    uint16_t status;
    int timeout_ms = rate_conv_time_ms[handle->rate] + 20;
    while (timeout_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(2));
        timeout_ms -= 2;

        err = ads1115_read_reg(handle, ADS1115_REG_CONFIG, &status);
        if (err != ESP_OK)
            return err;

        if (status & (1 << ADS1115_CFG_OS_BIT)) {
            /* Conversion complete */
            break;
        }
    }

    if (timeout_ms <= 0) {
        ESP_LOGE(TAG, "[0x%02X] conversion timeout", handle->i2c_addr);
        return ESP_ERR_TIMEOUT;
    }

    /* Read the conversion result */
    uint16_t raw;
    err = ads1115_read_reg(handle, ADS1115_REG_CONVERSION, &raw);
    if (err != ESP_OK)
        return err;

    *value = (int16_t)raw;
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Voltage conversion
 * ---------------------------------------------------------------------------
 */
float ads1115_compute_volts(ads1115_handle_t handle, int16_t counts) {
    if (!handle || !handle->initialized)
        return 0.0f;

    /* Get full-scale range in microvolts, convert to volts per LSB */
    uint32_t fs_uv = gain_fs_uv[handle->gain];
    float volts_per_bit = (float)fs_uv / 32768.0f / 1000000.0f;

    return counts * volts_per_bit;
}

/* ---------------------------------------------------------------------------
 * Data rate control
 * ---------------------------------------------------------------------------
 */
esp_err_t ads1115_set_rate(ads1115_handle_t handle, ads1115_rate_t rate) {
    if (!handle || !handle->initialized)
        return ESP_ERR_INVALID_STATE;
    if (rate > ADS1115_RATE_860)
        return ESP_ERR_INVALID_ARG;

    handle->rate = rate;
    return ESP_OK;
}

ads1115_rate_t ads1115_get_rate(ads1115_handle_t handle) {
    if (!handle || !handle->initialized)
        return ADS1115_RATE_128;
    return handle->rate;
}

/* ---------------------------------------------------------------------------
 * Differential read (single-shot mode)
 * ---------------------------------------------------------------------------
 */
esp_err_t ads1115_read_differential(ads1115_handle_t handle, ads1115_mux_t mux, int16_t *value) {
    if (!handle || !handle->initialized)
        return ESP_ERR_INVALID_STATE;
    if (mux > ADS1115_MUX_DIFF_2_3 || !value)
        return ESP_ERR_INVALID_ARG;

    uint16_t config = (1 << ADS1115_CFG_OS_BIT) |
                      (mux << ADS1115_CFG_MUX_SHIFT) |
                      (handle->gain << ADS1115_CFG_PGA_SHIFT) |
                      (1 << ADS1115_CFG_MODE_BIT) |
                      (handle->rate << ADS1115_CFG_DR_SHIFT) |
                      (handle->comp_mode << 4) |
                      (handle->comp_pol << 3) |
                      (handle->comp_lat << 2) |
                      (handle->comp_que << ADS1115_CFG_COMP_QUE_SHIFT);

    esp_err_t err = ads1115_write_reg(handle, ADS1115_REG_CONFIG, config);
    if (err != ESP_OK)
        return err;

    uint16_t status;
    int timeout_ms = rate_conv_time_ms[handle->rate] + 20;
    while (timeout_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(2));
        timeout_ms -= 2;

        err = ads1115_read_reg(handle, ADS1115_REG_CONFIG, &status);
        if (err != ESP_OK)
            return err;

        if (status & (1 << ADS1115_CFG_OS_BIT))
            break;
    }

    if (timeout_ms <= 0) {
        ESP_LOGE(TAG, "[0x%02X] conversion timeout", handle->i2c_addr);
        return ESP_ERR_TIMEOUT;
    }

    uint16_t raw;
    err = ads1115_read_reg(handle, ADS1115_REG_CONVERSION, &raw);
    if (err != ESP_OK)
        return err;

    *value = (int16_t)raw;
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Get I2C address
 * ---------------------------------------------------------------------------
 */
uint8_t ads1115_get_address(ads1115_handle_t handle) {
    if (!handle)
        return 0;
    return handle->i2c_addr;
}

/* ---------------------------------------------------------------------------
 * Comparator configuration
 * ---------------------------------------------------------------------------
 */
esp_err_t ads1115_set_comparator_thresholds(ads1115_handle_t handle,
                                            int16_t lo_thresh, int16_t hi_thresh) {
    if (!handle || !handle->initialized)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = ads1115_write_reg(handle, ADS1115_REG_LO_THRESH, (uint16_t)lo_thresh);
    if (err != ESP_OK)
        return err;

    err = ads1115_write_reg(handle, ADS1115_REG_HI_THRESH, (uint16_t)hi_thresh);
    return err;
}

esp_err_t ads1115_set_comparator_config(ads1115_handle_t handle,
                                        ads1115_comp_mode_t mode,
                                        ads1115_comp_pol_t polarity,
                                        ads1115_comp_lat_t latching,
                                        ads1115_comp_que_t queue) {
    if (!handle || !handle->initialized)
        return ESP_ERR_INVALID_STATE;

    handle->comp_mode = mode;
    handle->comp_pol = polarity;
    handle->comp_lat = latching;
    handle->comp_que = queue;

    /* Settings are applied on next conversion */
    return ESP_OK;
}
