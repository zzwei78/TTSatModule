/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef VOICE_PACKET_HANDLER_H
#define VOICE_PACKET_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include "freertos/queue.h"
#include "esp_err.h"

/* Maximum raw data size for AT^AUDPCM commands */
#define MAX_RAW_DATA_SIZE 200

/* Maximum AMR frames per AT^AUDPCM command (1 or 3) */
#define VOICE_MAX_FRAMES_PER_CMD     3

/**
 * @brief Structure to hold raw voice data packet
 */
typedef struct {
    uint8_t data[MAX_RAW_DATA_SIZE];
    size_t data_len;
} voice_raw_data_t;

/**
 * @brief Voice data output callback (uplink: TT -> BLE)
 *
 * Invoked when encoded voice data (AT^AUDPCM format) is ready to send.
 *
 * @param data AT^AUDPCM command string
 * @param len Length of the data
 * @param user_data User-provided context
 */
typedef void (*voice_data_output_callback_t)(const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Voice data downlink callback (downlink: BLE -> TT via MUX ch9)
 *
 * Invoked when decoded PCM data is ready to send to TT module.
 *
 * @param data PCM audio data
 * @param len Length of the data
 * @param user_data User-provided context
 */
typedef void (*voice_data_downlink_callback_t)(const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Register output callback for uplink (TT -> BLE)
 *
 * @param callback Callback function
 * @param user_data User context
 * @return 0 on success, -1 on error
 */
int voice_packet_register_output_callback(voice_data_output_callback_t callback, void *user_data);

/**
 * @brief Register downlink callback for downlink (BLE -> TT)
 *
 * @param callback Callback function
 * @param user_data User context
 * @return 0 on success, -1 on error
 */
int voice_packet_register_downlink_callback(voice_data_downlink_callback_t callback, void *user_data);

/**
 * @brief Start voice processing tasks
 *
 * Creates and starts voice encode/decode tasks:
 * - Encode task on CPU0: PCM -> AMRNB -> Base64 -> AT^AUDPCM
 * - Decode task on CPU1: AT^AUDPCM -> Base64 -> AMRNB -> PCM
 *
 * @return Task handle on success, NULL on failure
 */
TaskHandle_t voice_tasks_start(void);

/**
 * @brief Stop voice processing tasks
 *
 * Stops all voice tasks and prints statistics.
 */
void voice_tasks_stop(void);

/**
 * @brief Enqueue downlink voice packet (BLE -> TT)
 *
 * Processes AT^AUDPCM command by queuing for decode task.
 *
 * @param data Received data buffer (AT^AUDPCM="base64_data")
 * @param data_len Length of received data
 * @return 0 on success, -1 on error
 */
int voice_downlink_enqueue(uint8_t *data, size_t data_len);

#endif /* VOICE_PACKET_HANDLER_H */
