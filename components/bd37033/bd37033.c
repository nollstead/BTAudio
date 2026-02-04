/**
 * ============================================================================
 *  BD37033FV ESP-IDF Driver (New i2c_master API)
 *  ---------------------------------------------
 *  High-level control driver for the Rohm BD37033FV audio processor.
 *
 *  This file implements:
 *    - I2C register read/write using new i2c_master API
 *    - Initialization sequence
 *    - Volume control
 *    - Input selection
 *    - Input gain
 *    - Per-channel attenuation (fader/balance)
 *    - Bass / Treble tone control
 *    - Loudness control
 *
 *  No hardware details are hard-coded. All I2C pins, port numbers, and bus
 *  speeds are supplied by the application via bd37033fv_init().
 *
 * ============================================================================
 */

#include "bd37033.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BD37033FV";

/* ---------------------------------------------------------------------------
 * Internal driver state
 * ---------------------------------------------------------------------------
 */
typedef struct {
    bool initialized;
    bool owns_bus; // true if we created the bus, false if using shared bus
    bd37033fv_i2c_config_t cfg;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} bd37033fv_state_t;

static bd37033fv_state_t g_state = {0};

/* ---------------------------------------------------------------------------
 * Low-level I2C write helper (new API)
 * ---------------------------------------------------------------------------
 */
static esp_err_t bd37033fv_write_reg(uint8_t reg, uint8_t value) {
    if (!g_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {reg, value};

    esp_err_t err = i2c_master_transmit(g_state.dev_handle, data, sizeof(data), pdMS_TO_TICKS(20));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed reg=0x%02X val=0x%02X err=%s", reg, value,
                 esp_err_to_name(err));
    }

    return err;
}

/* ---------------------------------------------------------------------------
 * Low-level I2C read helper (new API)
 * ---------------------------------------------------------------------------
 */
static esp_err_t bd37033fv_read_reg(uint8_t reg, uint8_t *value) {
    if (!g_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        i2c_master_transmit_receive(g_state.dev_handle, &reg, 1, value, 1, pdMS_TO_TICKS(20));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed reg=0x%02X err=%s", reg, esp_err_to_name(err));
    }

    return err;
}

/* ---------------------------------------------------------------------------
 * Register addresses are defined in bd37033.h (from datasheet page 11)
 * ---------------------------------------------------------------------------
 */

/* ---------------------------------------------------------------------------
 * Initialization (new i2c_master API) - creates its own bus
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_init(const bd37033fv_i2c_config_t *cfg) {
    if (!cfg)
        return ESP_ERR_INVALID_ARG;

    memset(&g_state, 0, sizeof(g_state));
    g_state.cfg = *cfg;
    g_state.owns_bus = true;

    // Create I2C master bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = cfg->port,
        .sda_io_num = cfg->sda,
        .scl_io_num = cfg->scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags =
            {
                .enable_internal_pullup = true,
            },
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &g_state.bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    // Add BD37033FV device to the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = cfg->i2c_addr,
        .scl_speed_hz = cfg->clk_speed_hz,
    };

    err = i2c_master_bus_add_device(g_state.bus_handle, &dev_cfg, &g_state.dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(err));
        i2c_del_master_bus(g_state.bus_handle);
        return err;
    }

    g_state.initialized = true;

    ESP_LOGI(TAG, "Initialized at I2C addr 0x%02X (owns bus)", cfg->i2c_addr);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Initialization with existing bus handle (for sharing bus with other devices)
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_init_with_bus(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr,
                                  uint32_t clk_speed_hz) {
    if (!bus_handle)
        return ESP_ERR_INVALID_ARG;

    memset(&g_state, 0, sizeof(g_state));
    g_state.bus_handle = bus_handle;
    g_state.cfg.i2c_addr = i2c_addr;
    g_state.cfg.clk_speed_hz = clk_speed_hz;
    g_state.owns_bus = false;

    // Add BD37033FV device to the existing bus
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

    g_state.initialized = true;

    ESP_LOGI(TAG, "Initialized at I2C addr 0x%02X (shared bus)", i2c_addr);
    return ESP_OK;
}

esp_err_t bd37033fv_deinit(void) {
    if (g_state.initialized) {
        if (g_state.dev_handle) {
            i2c_master_bus_rm_device(g_state.dev_handle);
        }
        // Only delete the bus if we created it
        if (g_state.owns_bus && g_state.bus_handle) {
            i2c_del_master_bus(g_state.bus_handle);
        }
    }
    memset(&g_state, 0, sizeof(g_state));
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Volume control
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_volume(int vol_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp to device range: 0 to -79 dB
    if (vol_db > 0)
        vol_db = 0;
    if (vol_db < -79)
        vol_db = -79;

    uint8_t reg_val = (uint8_t)(-vol_db); // 0 = 0dB, 79 = -79dB

    return bd37033fv_write_reg(BD37033FV_VOLUME_GAIN, reg_val);
}

/* ---------------------------------------------------------------------------
 * Input selection
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_select_input(bd37033fv_input_t input) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint8_t val = (uint8_t)input & 0x03;
    return bd37033fv_write_reg(BD37033FV_INPUT_SELECT, val);
}

/* ---------------------------------------------------------------------------
 * Input gain
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_input_gain(int gain_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp: -12 to +12 dB
    if (gain_db < -12)
        gain_db = -12;
    if (gain_db > 12)
        gain_db = 12;

    uint8_t val = (uint8_t)(gain_db + 12); // map -12..+12 -> 0..24

    return bd37033fv_write_reg(BD37033FV_INPUT_GAIN, val);
}

/* ---------------------------------------------------------------------------
 * Per-channel fader attenuation
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_channel_attenuation(bd37033fv_channel_t ch, int att_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp: 0 to 79 dB attenuation
    if (att_db < 0)
        att_db = 0;
    if (att_db > 79)
        att_db = 79;

    uint8_t reg;
    switch (ch) {
    case BD37033FV_CH_FRONT_LEFT:
        reg = BD37033FV_FADER_1_FRONT;
        break;
    case BD37033FV_CH_FRONT_RIGHT:
        reg = BD37033FV_FADER_2_FRONT;
        break;
    case BD37033FV_CH_REAR_LEFT:
        reg = BD37033FV_FADER_1_REAR;
        break;
    case BD37033FV_CH_REAR_RIGHT:
        reg = BD37033FV_FADER_2_REAR;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return bd37033fv_write_reg(reg, (uint8_t)att_db);
}

/* ---------------------------------------------------------------------------
 * Bass / Treble
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_bass(int bass_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp: -14 to +14 dB
    if (bass_db < -14)
        bass_db = -14;
    if (bass_db > 14)
        bass_db = 14;

    uint8_t val = (uint8_t)(bass_db + 14); // map -14..+14 -> 0..28

    return bd37033fv_write_reg(BD37033FV_BASS_GAIN, val);
}

esp_err_t bd37033fv_set_treble(int treble_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp: -14 to +14 dB
    if (treble_db < -14)
        treble_db = -14;
    if (treble_db > 14)
        treble_db = 14;

    uint8_t val = (uint8_t)(treble_db + 14);

    return bd37033fv_write_reg(BD37033FV_TREBLE_GAIN, val);
}

/* ---------------------------------------------------------------------------
 * Loudness
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_loudness(bd37033fv_loudness_t mode) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint8_t val = (mode == BD37033FV_LOUDNESS_ON) ? 1 : 0;

    return bd37033fv_write_reg(BD37033FV_LOUDNESS_GAIN, val);
}

/* ---------------------------------------------------------------------------
 * Mute (sets volume to max attenuation)
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_mute(bd37033fv_mute_t mute) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Mute by setting volume to max attenuation
    if (mute == BD37033FV_MUTE_ON) {
        return bd37033fv_write_reg(BD37033FV_VOLUME_GAIN, 0x7F);
    }
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Update (currently no-op)
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_update(void) {
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Read register (public)
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_read_register(uint8_t reg, uint8_t *value) {
    return bd37033fv_read_reg(reg, value);
}

/* ---------------------------------------------------------------------------
 * Test function - probes device and tests basic write operations
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_test(void) {
    if (!g_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== BD37033FV Test ===");
    ESP_LOGI(TAG, "I2C addr: 0x%02X", g_state.cfg.i2c_addr);

    // Test 1: Probe the device (just check if it ACKs its address)
    ESP_LOGI(TAG, "Probing device...");
    esp_err_t err = i2c_master_probe(g_state.bus_handle, g_state.cfg.i2c_addr, pdMS_TO_TICKS(50));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "  Device ACKed at 0x%02X", g_state.cfg.i2c_addr);
    } else {
        ESP_LOGE(TAG, "  Device not responding: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Test 2: Try writing single bytes to different addresses
    // Note: BD37033FV may be write-only - reads may not work
    ESP_LOGI(TAG, "Testing writes (device may be write-only):");

    uint8_t test_addrs[] = {
        0x01, 0x02, 0x03, 0x05, 0x06, 0x20, 0x28, 0x29, 0x2A, 0x2B,
        0x2C, 0x30, 0x41, 0x44, 0x47, 0x51, 0x54, 0x57, 0x75}; // Try a few addresses
    for (int i = 0; i < sizeof(test_addrs); i++) {
        uint8_t data[2] = {test_addrs[i], 0x00};
        err = i2c_master_transmit(g_state.dev_handle, data, 2, pdMS_TO_TICKS(50));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  Write to 0x%02X: OK", test_addrs[i]);
        } else {
            ESP_LOGW(TAG, "  Write to 0x%02X: %s", test_addrs[i], esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "=== Test Complete ===");
    ESP_LOGI(TAG, "Note: If probe passed but reads return 0xFF, device is likely write-only.");
    return ESP_OK;
}
