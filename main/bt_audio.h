/*
 * bt_audio.h - Bluetooth Audio Module
 * Handles A2DP audio streaming and SPP command interface
 */

#ifndef BT_AUDIO_H
#define BT_AUDIO_H

#include "audio_hal.h"
#include "board.h"

/**
 * Initialize the Bluetooth audio module.
 * Must be called after audio_board_init().
 *
 * @param board_handle  Audio board handle for codec control
 * @param version       Firmware version string (for SPP version command)
 */
void bt_audio_init(audio_board_handle_t board_handle, const char *version);

/**
 * Start the A2DP audio streaming task.
 * Creates a FreeRTOS task that handles Bluetooth audio reception.
 */
void bt_audio_start_a2dp(void);

/**
 * Start the SPP (Serial Port Profile) command interface task.
 * Creates a FreeRTOS task that handles serial commands over Bluetooth.
 */
void bt_audio_start_spp(void);

#endif // BT_AUDIO_H
