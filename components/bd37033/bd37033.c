/**
 * ============================================================================
 *  BD37033FV ESP‑IDF Driver
 *  -------------------------------------
 *  High‑level control driver for the Rohm BD37033FV audio processor.
 *
 *  This file implements:
 *    - I2C register write helpers
 *    - Initialization sequence
 *    - Volume control
 *    - Input selection
 *    - Input gain
 *    - Per‑channel attenuation (fader/balance)
 *    - Bass / Treble tone control
 *    - Loudness control
 *
 *  No hardware details are hard‑coded. All I2C pins, port numbers, and bus
 *  speeds are supplied by the application via bd37033fv_init().
 *
 * ============================================================================
 *
 *  Wiring Example (ESP32‑WROOM‑32E)
 *  --------------------------------
 *  The following wiring is ONLY an example for your test setup. The driver
 *  does not depend on these values.
 *
 *      ESP32‑WROOM‑32E        BD37033FV
 *      ----------------        ----------
 *      GPIO21 (pin 33)  --->  SDA
 *      GPIO22 (pin 36)  --->  SCL
 *      3V3               ---> VCC
 *      GND               ---> GND
 *
 *
 *  Example Usage
 *  -------------
 *  #include "bd37033fv.h"
 *
 *  void app_main(void)
 *  {
 *      bd37033fv_i2c_config_t cfg = {
 *          .port = I2C_NUM_0,
 *          .sda = GPIO_NUM_21,   // SDA on IO21 (module pin 33)
 *          .scl = GPIO_NUM_22,   // SCL on IO22 (module pin 36)
 *          .clk_speed_hz = 400000,
 *          .i2c_addr = BD37033FV_I2C_ADDR_DEFAULT
 *      };
 *
 *      bd37033fv_init(&cfg);
 *
 *      bd37033fv_set_volume(-20);                 // -20 dB
 *      bd37033fv_select_input(BD37033FV_INPUT_1); // Select input 1
 *      bd37033fv_set_input_gain(6);               // +6 dB input gain
 *      bd37033fv_set_bass(4);                     // +4 dB bass
 *      bd37033fv_set_treble(-2);                  // -2 dB treble
 *      bd37033fv_set_loudness(BD37033FV_LOUDNESS_ON);
 *
 *      bd37033fv_set_channel_attenuation(
 *          BD37033FV_CH_REAR_LEFT, 10);           // 10 dB attenuation
 *  }
 *
 *
 *  Notes
 *  -----
 *  • All dB values are clamped to the BD37033FV’s valid ranges.
 *  • All register writes are synchronous.
 *  • The driver does not uninstall the I2C driver on deinit().
 *
 * ============================================================================
 */

#include "bd37033.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BD37033FV";

/* ---------------------------------------------------------------------------
 * Internal driver state
 * ---------------------------------------------------------------------------
 */
typedef struct {
    bool initialized;
    bd37033fv_i2c_config_t cfg;
} bd37033fv_state_t;

static bd37033fv_state_t g_state = {0};

/* ---------------------------------------------------------------------------
 * Low‑level I2C write helper
 * ---------------------------------------------------------------------------
 */
static esp_err_t bd37033fv_write_reg(uint8_t reg, uint8_t value) {
    if (!g_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {reg, value};

    esp_err_t err = i2c_master_write_to_device(g_state.cfg.port, g_state.cfg.i2c_addr, data,
                                               sizeof(data), pdMS_TO_TICKS(20));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed reg=0x%02X val=0x%02X err=%d", reg, value, err);
    }

    return err;
}

/* ---------------------------------------------------------------------------
 * Register map (partial — extend as needed)
 * ---------------------------------------------------------------------------
 */
#define REG_INPUT_SELECT 0x00
#define REG_INPUT_GAIN 0x01
#define REG_VOLUME 0x02
#define REG_BASS 0x03
#define REG_TREBLE 0x04
#define REG_LOUDNESS 0x05
#define REG_CH_ATT_BASE 0x10 // 0x10–0x13 for FL/FR/RL/RR

/* ---------------------------------------------------------------------------
 * Initialization
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_init(const bd37033fv_i2c_config_t *cfg) {
    if (!cfg)
        return ESP_ERR_INVALID_ARG;

    memset(&g_state, 0, sizeof(g_state));
    g_state.cfg = *cfg;

    /* Install I2C driver if not already installed */
    i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                         .sda_io_num = cfg->sda,
                         .scl_io_num = cfg->scl,
                         .sda_pullup_en = GPIO_PULLUP_ENABLE,
                         .scl_pullup_en = GPIO_PULLUP_ENABLE,
                         .master.clk_speed = cfg->clk_speed_hz};

    esp_err_t err = i2c_param_config(cfg->port, &conf);
    if (err != ESP_OK)
        return err;

    err = i2c_driver_install(cfg->port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    g_state.initialized = true;

    /* Basic startup defaults */
    bd37033fv_write_reg(REG_VOLUME, 0x00); // 0 dB
    bd37033fv_write_reg(REG_BASS, 0x00);
    bd37033fv_write_reg(REG_TREBLE, 0x00);
    bd37033fv_write_reg(REG_LOUDNESS, 0x00);

    return ESP_OK;
}

esp_err_t bd37033fv_deinit(void) {
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

    /* Clamp to device range: 0 to -79 dB */
    if (vol_db > 0)
        vol_db = 0;
    if (vol_db < -79)
        vol_db = -79;

    uint8_t reg_val = (uint8_t)(-vol_db); // 0 = 0dB, 79 = -79dB

    return bd37033fv_write_reg(REG_VOLUME, reg_val);
}

/* ---------------------------------------------------------------------------
 * Input selection
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_select_input(bd37033fv_input_t input) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint8_t val = (uint8_t)input & 0x03;
    return bd37033fv_write_reg(REG_INPUT_SELECT, val);
}

/* ---------------------------------------------------------------------------
 * Input gain
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_input_gain(int gain_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    /* Example clamp: -12 to +12 dB */
    if (gain_db < -12)
        gain_db = -12;
    if (gain_db > 12)
        gain_db = 12;

    uint8_t val = (uint8_t)(gain_db + 12); // map -12..+12 → 0..24

    return bd37033fv_write_reg(REG_INPUT_GAIN, val);
}

/* ---------------------------------------------------------------------------
 * Per‑channel attenuation
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_channel_attenuation(bd37033fv_channel_t ch, int att_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    if (ch < BD37033FV_CH_FRONT_LEFT || ch > BD37033FV_CH_REAR_RIGHT)
        return ESP_ERR_INVALID_ARG;

    /* Clamp: 0 to 79 dB attenuation */
    if (att_db < 0)
        att_db = 0;
    if (att_db > 79)
        att_db = 79;

    uint8_t reg = REG_CH_ATT_BASE + (uint8_t)ch;
    uint8_t val = (uint8_t)att_db;

    return bd37033fv_write_reg(reg, val);
}

/* ---------------------------------------------------------------------------
 * Bass / Treble
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_bass(int bass_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    /* Clamp: -14 to +14 dB */
    if (bass_db < -14)
        bass_db = -14;
    if (bass_db > 14)
        bass_db = 14;

    uint8_t val = (uint8_t)(bass_db + 14); // map -14..+14 → 0..28

    return bd37033fv_write_reg(REG_BASS, val);
}

esp_err_t bd37033fv_set_treble(int treble_db) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    /* Clamp: -14 to +14 dB */
    if (treble_db < -14)
        treble_db = -14;
    if (treble_db > 14)
        treble_db = 14;

    uint8_t val = (uint8_t)(treble_db + 14);

    return bd37033fv_write_reg(REG_TREBLE, val);
}

/* ---------------------------------------------------------------------------
 * Loudness
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_set_loudness(bd37033fv_loudness_t mode) {
    if (!g_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint8_t val = (mode == BD37033FV_LOUDNESS_ON) ? 1 : 0;

    return bd37033fv_write_reg(REG_LOUDNESS, val);
}

/* ---------------------------------------------------------------------------
 * Update (currently no‑op)
 * ---------------------------------------------------------------------------
 */
esp_err_t bd37033fv_update(void) {
    return ESP_OK;
}
