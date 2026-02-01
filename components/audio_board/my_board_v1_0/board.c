/*
 * Custom Board Implementation for ESP-ADF
 * Uses ES8388 codec driver from ESP-ADF
 */

#include "board.h"
#include "audio_mem.h"
#include "es8388.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include <periph_button.h>
#include <periph_touch.h>

static const char *TAG = "MY_BOARD";

static audio_board_handle_t board_handle = NULL;

audio_board_handle_t audio_board_init(void) {
    if (board_handle) {
        return board_handle;
    }

    board_handle = (audio_board_handle_t)audio_calloc(1, sizeof(struct audio_board_handle));
    AUDIO_MEM_CHECK(TAG, board_handle, return NULL);
    board_handle->audio_hal = audio_board_codec_init();

    return board_handle;
}

audio_hal_handle_t audio_board_codec_init(void) {
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();

    // Your headphones are on LOUT2/ROUT2
    // LINE2 should route DAC to LOUT2/ROUT2
    audio_codec_cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;

    audio_hal_handle_t codec_hal =
        audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8388_DEFAULT_HANDLE);

    AUDIO_NULL_CHECK(TAG, codec_hal, return NULL); // Check before using the handle

    // Start DAC path, then unmute and set volume via the generic HAL
    audio_hal_ctrl_codec(codec_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
    audio_hal_set_mute(codec_hal, false);
    audio_hal_set_volume(codec_hal, 80);

    // ESP-ADF sets LOUT2/ROUT2 to -30dB by default. Our headphone jack uses these outputs.
    // Set LOUT2/ROUT2 volume to 0dB (0x1E) for proper output level.
    es8388_write_reg(0x30, 0x1E);  // DACCONTROL26: LOUT2 volume = 0dB
    es8388_write_reg(0x31, 0x1E);  // DACCONTROL27: ROUT2 volume = 0dB

    return codec_hal;
}

// ===== Codec HAL init (ES8388) =====

audio_board_handle_t audio_board_get_handle(void) {
    return board_handle;
}

esp_err_t audio_board_deinit(audio_board_handle_t audio_board) {

    esp_err_t ret = ESP_OK;
    ret = audio_hal_deinit(audio_board->audio_hal);
    audio_free(audio_board);
    board_handle = NULL;
    return ret;
}

// ===== Optional / stubbed features =====

// display_service_handle_t audio_board_led_init(void) {
//     // No LEDs on this board
//     return NULL;
// }