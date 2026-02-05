/**
 * ============================================================================
 *  BD37033 ESP-IDF Driver (New i2c_master API)
 *  -------------------------------------------
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
 * ============================================================================
 */

#include "bd37033.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BD37033";

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
} bd37033_state_t;

static bd37033_state_t g_state = {0};

/* ---------------------------------------------------------------------------
 * Low-level I2C write helper
 * ---------------------------------------------------------------------------
 */
static esp_err_t bd37033_write_reg(uint8_t reg, uint8_t value) {
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
 * Low-level I2C read helper
 * ---------------------------------------------------------------------------
 */
static esp_err_t bd37033_read_reg(uint8_t reg, uint8_t *value) {
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
 * Initialization (uses existing I2C bus handle)
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr,
                       uint32_t clk_speed_hz) {
    if (!bus_handle)
        return ESP_ERR_INVALID_ARG;

    memset(&g_state, 0, sizeof(g_state));
    g_state.bus_handle = bus_handle;
    g_state.i2c_addr = i2c_addr;
    g_state.clk_speed_hz = clk_speed_hz;

    // Add BD37033 device to the existing bus
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

    ESP_LOGI(TAG, "Initialized at I2C addr 0x%02X", i2c_addr);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Configure chip defaults for basic operation
 * Based on Arduino library initialization sequence
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_setup_defaults(void) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = ESP_OK;

    // INITIAL_SETUP (0x01): 0x24 = 0b00100100
    // D7=0: Advanced Switch ON, D6=0: Anti Alias Filter ON
    // D5-D4=10: Switch time, D3=0/D2=1: fixed, D1-D0=00: Mute time
    err |= bd37033_write_reg(BD37033_INITIAL_SETUP, 0x24);

    // LPF_SETUP (0x02): Subwoofer LPF off, flat response
    err |= bd37033_write_reg(BD37033_LPF_SETUP, 0x00);

    // MIXING_SETUP (0x03): 0x61 = 0b01100001
    // D1-D0=01: A_Single mode (A1→left, A2→right)
    // Other options: 0x60=Mix(both), 0x62=B_Single(B1-B2)
    err |= bd37033_write_reg(BD37033_MIXING_SETUP, 0x61);

    // INPUT_SELECT (0x05): Select input 1
    err |= bd37033_write_reg(BD37033_INPUT_SELECT, 0x00);

    // INPUT_GAIN (0x06): 0 dB input gain, no mute
    err |= bd37033_write_reg(BD37033_INPUT_GAIN, 0x00);

    // VOLUME_GAIN (0x20): 0 dB (max volume)
    err |= bd37033_write_reg(BD37033_VOLUME_GAIN, 0x00);

    // Faders: All at 0 dB (no attenuation)
    err |= bd37033_write_reg(BD37033_FADER_1_FRONT, 0x00);
    err |= bd37033_write_reg(BD37033_FADER_2_FRONT, 0x00);
    err |= bd37033_write_reg(BD37033_FADER_1_REAR, 0x00);
    err |= bd37033_write_reg(BD37033_FADER_2_REAR, 0x00);
    err |= bd37033_write_reg(BD37033_FADER_SUB_1, 0x00);
    err |= bd37033_write_reg(BD37033_FADER_SUB_2, 0x00);

    // EQ Setup: All flat (Q=1.0, center frequencies default)
    err |= bd37033_write_reg(BD37033_BASS_SETUP, 0x00);
    err |= bd37033_write_reg(BD37033_MIDDLE_SETUP, 0x00);
    err |= bd37033_write_reg(BD37033_TREBLE_SETUP, 0x00);

    // EQ Gains: All 0 dB (flat)
    err |= bd37033_write_reg(BD37033_BASS_GAIN, 0x00);
    err |= bd37033_write_reg(BD37033_MIDDLE_GAIN, 0x00);
    err |= bd37033_write_reg(BD37033_TREBLE_GAIN, 0x00);

    // Loudness: Off
    err |= bd37033_write_reg(BD37033_LOUDNESS_GAIN, 0x00);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure defaults");
    } else {
        ESP_LOGI(TAG, "Defaults configured successfully");
    }

    return err;
}

esp_err_t bd37033_deinit(void) {
    if (g_state.initialized) {
        if (g_state.dev_handle) {
            i2c_master_bus_rm_device(g_state.dev_handle);
        }
    }
    memset(&g_state, 0, sizeof(g_state));
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Volume control
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_set_volume(int vol_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp to device range: 0 to -79 dB
    if (vol_db > 0)
        vol_db = 0;
    if (vol_db < -79)
        vol_db = -79;

    uint8_t reg_val = (uint8_t)(-vol_db); // 0 = 0dB, 79 = -79dB

    return bd37033_write_reg(BD37033_VOLUME_GAIN, reg_val);
}

/* ---------------------------------------------------------------------------
 * Input selection
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_select_input(bd37033_input_t input) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint8_t val = (uint8_t)input & 0x03;
    return bd37033_write_reg(BD37033_INPUT_SELECT, val);
}

/* ---------------------------------------------------------------------------
 * Input gain
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_set_input_gain(int gain_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp: -12 to +12 dB
    if (gain_db < -12)
        gain_db = -12;
    if (gain_db > 12)
        gain_db = 12;

    uint8_t val = (uint8_t)(gain_db + 12); // map -12..+12 -> 0..24

    return bd37033_write_reg(BD37033_INPUT_GAIN, val);
}

/* ---------------------------------------------------------------------------
 * Per-channel fader attenuation
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_set_channel_attenuation(bd37033_channel_t ch, int att_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp: 0 to 79 dB attenuation
    if (att_db < 0)
        att_db = 0;
    if (att_db > 79)
        att_db = 79;

    uint8_t reg;
    switch (ch) {
    case BD37033_CH_FRONT_LEFT:
        reg = BD37033_FADER_1_FRONT;
        break;
    case BD37033_CH_FRONT_RIGHT:
        reg = BD37033_FADER_2_FRONT;
        break;
    case BD37033_CH_REAR_LEFT:
        reg = BD37033_FADER_1_REAR;
        break;
    case BD37033_CH_REAR_RIGHT:
        reg = BD37033_FADER_2_REAR;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return bd37033_write_reg(reg, (uint8_t)att_db);
}

/* ---------------------------------------------------------------------------
 * Bass / Treble
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_set_bass(int bass_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp: -14 to +14 dB
    if (bass_db < -14)
        bass_db = -14;
    if (bass_db > 14)
        bass_db = 14;

    uint8_t val = (uint8_t)(bass_db + 14); // map -14..+14 -> 0..28

    return bd37033_write_reg(BD37033_BASS_GAIN, val);
}

esp_err_t bd37033_set_treble(int treble_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Clamp: -14 to +14 dB
    if (treble_db < -14)
        treble_db = -14;
    if (treble_db > 14)
        treble_db = 14;

    uint8_t val = (uint8_t)(treble_db + 14);

    return bd37033_write_reg(BD37033_TREBLE_GAIN, val);
}

/* ---------------------------------------------------------------------------
 * Loudness
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_set_loudness(bd37033_loudness_t mode) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint8_t val = (mode == BD37033_LOUDNESS_ON) ? 1 : 0;

    return bd37033_write_reg(BD37033_LOUDNESS_GAIN, val);
}

/* ---------------------------------------------------------------------------
 * Mute (sets volume to max attenuation)
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_set_mute(bd37033_mute_t mute) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    // Mute by setting volume to max attenuation
    if (mute == BD37033_MUTE_ON) {
        return bd37033_write_reg(BD37033_VOLUME_GAIN, 0x7F);
    }
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Read register (public)
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_read_register(uint8_t reg, uint8_t *value) {
    return bd37033_read_reg(reg, value);
}

/* ---------------------------------------------------------------------------
 * Test function - probes device and tests basic write operations
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033_test(void) {
    if (!g_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== BD37033 Test ===");
    ESP_LOGI(TAG, "I2C addr: 0x%02X", g_state.i2c_addr);

    // Test 1: Probe the device (just check if it ACKs its address)
    ESP_LOGI(TAG, "Probing device...");
    esp_err_t err = i2c_master_probe(g_state.bus_handle, g_state.i2c_addr, pdMS_TO_TICKS(50));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "  Device ACKed at 0x%02X", g_state.i2c_addr);
    } else {
        ESP_LOGE(TAG, "  Device not responding: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Test 2: Try writing single bytes to different registers
    ESP_LOGI(TAG, "Testing writes (device may be write-only):");

    uint8_t test_addrs[] = {
        0x01, 0x02, 0x03, 0x05, 0x06, 0x20, 0x28, 0x29, 0x2A, 0x2B,
        0x2C, 0x30, 0x41, 0x44, 0x47, 0x51, 0x54, 0x57, 0x75};
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
    return ESP_OK;
}
