#include "adc_manager.h"
#include "board.h"
#include "esp_log.h"

static const char *TAG = "ADC_MGR";

static ads1115_handle_t chips[ADC_CHIP_COUNT] = {NULL};
static bool initialized = false;
static bool clamp_negative = true;

esp_err_t adc_manager_init(void) {
    chips[0] = audio_board_get_ads1115(0);
    chips[1] = audio_board_get_ads1115(1);

    int count = 0;
    for (int i = 0; i < ADC_CHIP_COUNT; i++) {
        if (chips[i]) {
            ESP_LOGI(TAG, "Chip %d (0x%02X): channels %d-%d",
                     i, ads1115_get_address(chips[i]),
                     i * ADC_CHANNELS_PER_CHIP,
                     i * ADC_CHANNELS_PER_CHIP + ADC_CHANNELS_PER_CHIP - 1);
            count++;
        } else {
            ESP_LOGW(TAG, "Chip %d: not available (channels %d-%d offline)",
                     i, i * ADC_CHANNELS_PER_CHIP,
                     i * ADC_CHANNELS_PER_CHIP + ADC_CHANNELS_PER_CHIP - 1);
        }
    }

    initialized = true;

    if (count == 0) {
        ESP_LOGE(TAG, "No ADC chips available");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Initialized: %d channels available", count * ADC_CHANNELS_PER_CHIP);
    return ESP_OK;
}

esp_err_t adc_manager_read(uint8_t channel, int16_t *raw, float *volts) {
    if (!initialized)
        return ESP_ERR_INVALID_STATE;
    if (channel >= ADC_CHANNEL_COUNT)
        return ESP_ERR_INVALID_ARG;

    uint8_t chip_idx = channel / ADC_CHANNELS_PER_CHIP;
    uint8_t pin = channel % ADC_CHANNELS_PER_CHIP;

    if (!chips[chip_idx])
        return ESP_ERR_NOT_FOUND;

    int16_t counts;
    esp_err_t err = ads1115_read_single_ended(chips[chip_idx], pin, &counts);
    if (err != ESP_OK)
        return err;

    if (clamp_negative && counts < 0)
        counts = 0;

    if (raw)
        *raw = counts;
    if (volts)
        *volts = ads1115_compute_volts(chips[chip_idx], counts);

    return ESP_OK;
}

uint8_t adc_manager_channel_count(void) {
    if (!initialized)
        return 0;
    uint8_t count = 0;
    for (int i = 0; i < ADC_CHIP_COUNT; i++) {
        if (chips[i])
            count += ADC_CHANNELS_PER_CHIP;
    }
    return count;
}

bool adc_manager_channel_available(uint8_t channel) {
    if (!initialized || channel >= ADC_CHANNEL_COUNT)
        return false;
    return chips[channel / ADC_CHANNELS_PER_CHIP] != NULL;
}

esp_err_t adc_manager_set_gain(uint8_t chip, ads1115_gain_t gain) {
    if (!initialized)
        return ESP_ERR_INVALID_STATE;
    if (chip >= ADC_CHIP_COUNT)
        return ESP_ERR_INVALID_ARG;
    if (!chips[chip])
        return ESP_ERR_NOT_FOUND;

    return ads1115_set_gain(chips[chip], gain);
}

ads1115_gain_t adc_manager_get_gain(uint8_t chip) {
    if (!initialized || chip >= ADC_CHIP_COUNT || !chips[chip])
        return ADS1115_GAIN_2_048;
    return ads1115_get_gain(chips[chip]);
}

esp_err_t adc_manager_set_rate(uint8_t chip, ads1115_rate_t rate) {
    if (!initialized)
        return ESP_ERR_INVALID_STATE;
    if (chip >= ADC_CHIP_COUNT)
        return ESP_ERR_INVALID_ARG;
    if (!chips[chip])
        return ESP_ERR_NOT_FOUND;

    return ads1115_set_rate(chips[chip], rate);
}

ads1115_rate_t adc_manager_get_rate(uint8_t chip) {
    if (!initialized || chip >= ADC_CHIP_COUNT || !chips[chip])
        return ADS1115_RATE_128;
    return ads1115_get_rate(chips[chip]);
}

void adc_manager_set_clamp(bool enabled) {
    clamp_negative = enabled;
    ESP_LOGI(TAG, "Negative value clamping %s", enabled ? "enabled" : "disabled");
}

bool adc_manager_get_clamp(void) {
    return clamp_negative;
}
