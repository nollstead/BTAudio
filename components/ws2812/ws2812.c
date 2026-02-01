#include "ws2812.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include <stdlib.h>

// WS2812 timing (in RMT ticks at 10MHz resolution = 100ns per tick)
#define WS2812_T0H_TICKS 4 // 0.4us
#define WS2812_T0L_TICKS 8 // 0.8us
#define WS2812_T1H_TICKS 8 // 0.8us
#define WS2812_T1L_TICKS 4 // 0.4us
#define WS2812_RESET_US 280

// Default brightness (0-255), keeps LEDs from being blinding
#define DEFAULT_BRIGHTNESS 32

// Predefined color values {R, G, B}
static const uint8_t color_table[][3] = {
    [WS2812_OFF] = {0, 0, 0},
    [WS2812_RED] = {DEFAULT_BRIGHTNESS, 0, 0},
    [WS2812_GREEN] = {0, DEFAULT_BRIGHTNESS, 0},
    [WS2812_BLUE] = {0, 0, DEFAULT_BRIGHTNESS},
    [WS2812_WHITE] = {DEFAULT_BRIGHTNESS, DEFAULT_BRIGHTNESS, DEFAULT_BRIGHTNESS},
    [WS2812_YELLOW] = {DEFAULT_BRIGHTNESS, DEFAULT_BRIGHTNESS, 0},
    [WS2812_CYAN] = {0, DEFAULT_BRIGHTNESS, DEFAULT_BRIGHTNESS},
    [WS2812_MAGENTA] = {DEFAULT_BRIGHTNESS, 0, DEFAULT_BRIGHTNESS},
    [WS2812_ORANGE] = {DEFAULT_BRIGHTNESS, DEFAULT_BRIGHTNESS / 4, 0},
    [WS2812_PURPLE] = {DEFAULT_BRIGHTNESS / 2, 0, DEFAULT_BRIGHTNESS},
};

static const char *color_names[] = {
    [WS2812_OFF] = "Off",       [WS2812_RED] = "Red",         [WS2812_GREEN] = "Green",
    [WS2812_BLUE] = "Blue",     [WS2812_WHITE] = "White",     [WS2812_YELLOW] = "Yellow",
    [WS2812_CYAN] = "Cyan",     [WS2812_MAGENTA] = "Magenta", [WS2812_ORANGE] = "Orange",
    [WS2812_PURPLE] = "Purple",
};

struct ws2812 {
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
};

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                            const void *primary_data, size_t data_size,
                            rmt_encode_state_t *ret_state) {
    ws2812_encoder_t *ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws2812_encoder->state) {
    case 0:
        encoded_symbols += ws2812_encoder->bytes_encoder->encode(
            ws2812_encoder->bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        // fall through
    case 1:
        encoded_symbols += ws2812_encoder->copy_encoder->encode(
            ws2812_encoder->copy_encoder, channel, &ws2812_encoder->reset_code,
            sizeof(ws2812_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = RMT_ENCODING_RESET;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t ws2812_encoder_del(rmt_encoder_t *encoder) {
    ws2812_encoder_t *ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(ws2812_encoder->bytes_encoder);
    rmt_del_encoder(ws2812_encoder->copy_encoder);
    free(ws2812_encoder);
    return ESP_OK;
}

static esp_err_t ws2812_encoder_reset(rmt_encoder_t *encoder) {
    ws2812_encoder_t *ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(ws2812_encoder->bytes_encoder);
    rmt_encoder_reset(ws2812_encoder->copy_encoder);
    ws2812_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_new(rmt_encoder_handle_t *ret_encoder) {
    ws2812_encoder_t *ws2812_encoder = calloc(1, sizeof(ws2812_encoder_t));
    if (!ws2812_encoder) {
        return ESP_ERR_NO_MEM;
    }

    ws2812_encoder->base.encode = ws2812_encode;
    ws2812_encoder->base.del = ws2812_encoder_del;
    ws2812_encoder->base.reset = ws2812_encoder_reset;

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 =
            {
                .level0 = 1,
                .duration0 = WS2812_T0H_TICKS,
                .level1 = 0,
                .duration1 = WS2812_T0L_TICKS,
            },
        .bit1 =
            {
                .level0 = 1,
                .duration0 = WS2812_T1H_TICKS,
                .level1 = 0,
                .duration1 = WS2812_T1L_TICKS,
            },
        .flags.msb_first = 1,
    };

    esp_err_t ret = rmt_new_bytes_encoder(&bytes_encoder_config, &ws2812_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        free(ws2812_encoder);
        return ret;
    }

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &ws2812_encoder->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(ws2812_encoder->bytes_encoder);
        free(ws2812_encoder);
        return ret;
    }

    ws2812_encoder->reset_code = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = WS2812_RESET_US * 10 / 2,
        .level1 = 0,
        .duration1 = WS2812_RESET_US * 10 / 2,
    };

    *ret_encoder = &ws2812_encoder->base;
    return ESP_OK;
}

esp_err_t ws2812_init(int gpio_num, ws2812_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct ws2812 *dev = calloc(1, sizeof(struct ws2812));
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = 10 * 1000 * 1000,
        .trans_queue_depth = 4,
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &dev->channel);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    ret = ws2812_encoder_new(&dev->encoder);
    if (ret != ESP_OK) {
        rmt_del_channel(dev->channel);
        free(dev);
        return ret;
    }

    ret = rmt_enable(dev->channel);
    if (ret != ESP_OK) {
        rmt_del_encoder(dev->encoder);
        rmt_del_channel(dev->channel);
        free(dev);
        return ret;
    }

    *handle = dev;
    return ESP_OK;
}

esp_err_t ws2812_set_rgb(ws2812_handle_t handle, uint8_t red, uint8_t green, uint8_t blue) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // WS2812 expects GRB order
    uint8_t pixel[3] = {green, red, blue};
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    esp_err_t ret =
        rmt_transmit(handle->channel, handle->encoder, pixel, sizeof(pixel), &tx_config);
    if (ret != ESP_OK) {
        return ret;
    }

    return rmt_tx_wait_all_done(handle->channel, portMAX_DELAY);
}

esp_err_t ws2812_set(ws2812_handle_t handle, ws2812_color_t color) {
    if (color >= WS2812_COLOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    return ws2812_set_rgb(handle, color_table[color][0], color_table[color][1],
                          color_table[color][2]);
}

const char *ws2812_color_name(ws2812_color_t color) {
    if (color >= WS2812_COLOR_COUNT) {
        return "Unknown";
    }
    return color_names[color];
}

esp_err_t ws2812_deinit(ws2812_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    rmt_disable(handle->channel);
    rmt_del_encoder(handle->encoder);
    rmt_del_channel(handle->channel);
    free(handle);

    return ESP_OK;
}
