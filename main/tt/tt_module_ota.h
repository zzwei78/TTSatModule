/*
 * tt_module_ota.h - Tiantong Module OTA Update Interface (Streaming)
 *
 * Real-time streaming OTA: BLE data → 128B buffer → XMODEM → TT module.
 * No flash partition needed (TT firmware 2-3MB too large for 4MB flash).
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef TT_MODULE_OTA_H
#define TT_MODULE_OTA_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* OTA Update States */
typedef enum {
    TT_OTA_STATE_IDLE = 0,
    TT_OTA_STATE_PREPARING,         /* Stopping MUX, resetting, AT+UPDATE */
    TT_OTA_STATE_WAITING_READY,     /* Waiting for TT "ready" / "baud ok" / 'C' */
    TT_OTA_STATE_UPLOADING,         /* XMODEM data transfer */
    TT_OTA_STATE_VERIFYING,         /* Waiting for "done" */
    TT_OTA_STATE_COMPLETED,
    TT_OTA_STATE_FAILED
} tt_ota_state_t;

/* OTA Update Result (detailed failure reason) */
typedef enum {
    TT_OTA_RESULT_OK = 0,
    TT_OTA_RESULT_ERROR_BATTERY,        /* Battery too low */
    TT_OTA_RESULT_ERROR_READY_TIMEOUT,  /* "ready" timeout */
    TT_OTA_RESULT_ERROR_BAUD_TIMEOUT,   /* "baud ok" timeout */
    TT_OTA_RESULT_ERROR_CRC_TIMEOUT,    /* 'C' (XMODEM start) timeout */
    TT_OTA_RESULT_ERROR_XMODEM,         /* XMODEM packet failed (NAK > max retries) */
    TT_OTA_RESULT_ERROR_CAN,            /* TT module sent CAN (abort) */
    TT_OTA_RESULT_ERROR_MD5,            /* "md5 failed" from TT */
    TT_OTA_RESULT_ERROR_DONE_TIMEOUT,   /* "done" timeout */
    TT_OTA_RESULT_ERROR_UART,           /* UART communication error */
    TT_OTA_RESULT_ERROR_NOT_READY,      /* feed() called before prep complete */
    TT_OTA_RESULT_ERROR_PARTITION,      /* Partition read error */
} tt_ota_result_t;

/* OTA Update Progress Callback */
typedef void (*tt_ota_progress_cb_t)(tt_ota_state_t state, int progress);

/* Minimum battery voltage for TT OTA (mV) */
#define TT_OTA_MIN_BATTERY_MV   3500

/**
 * @brief Initialize TT Module OTA subsystem
 */
esp_err_t tt_module_ota_init(void);

/**
 * @brief Deinitialize TT Module OTA subsystem
 */
esp_err_t tt_module_ota_deinit(void);

/**
 * @brief Begin TT OTA — prepare TT module for XMODEM transfer
 *
 * Starts a background task that:
 * 1. Stops TT communication (tt_module_stop)
 * 2. Sends AT+UPDATE at 115200
 * 3. Waits 3s, hardware reset TT
 * 4. Waits for "ready"
 * 5. Switches baudrate to 921600
 * 6. Sends "loadx", waits for 'C'
 *
 * Returns immediately. Caller should poll tt_module_ota_is_ready() or
 * wait for progress callback reporting UPLOADING state.
 *
 * @param total_size Total firmware size (from APP)
 * @param progress_cb Progress callback
 * @return ESP_OK if started, error if already in progress or battery low
 */
esp_err_t tt_module_ota_begin(size_t total_size, tt_ota_progress_cb_t progress_cb);

/**
 * @brief Check if TT module is ready for data (prep complete)
 *
 * @return true if XMODEM transfer is active and ready for firmware data
 */
bool tt_module_ota_is_ready(void);

/**
 * @brief Feed firmware data to TT module via XMODEM
 *
 * Accumulates data into 128-byte XMODEM packets and sends them.
 * Called from BLE OTA data handler for each received packet.
 *
 * @param data Firmware data bytes
 * @param len Data length
 * @return ESP_OK on success, error if XMODEM failed or not ready
 */
esp_err_t tt_module_ota_feed(const uint8_t *data, size_t len);

/**
 * @brief Finish TT OTA — flush remaining data, send EOT, wait for done
 *
 * Called after all firmware data has been fed.
 *
 * @return ESP_OK on success, error on failure
 */
esp_err_t tt_module_ota_finish(void);

/**
 * @brief Cancel ongoing OTA update
 */
esp_err_t tt_module_ota_cancel(void);

/**
 * @brief Get current OTA state
 */
tt_ota_state_t tt_module_ota_get_state(void);

/**
 * @brief Get detailed failure result (valid when state == FAILED)
 */
tt_ota_result_t tt_module_ota_get_result(void);

/**
 * @brief Check if OTA update is in progress
 */
bool tt_module_ota_is_in_progress(void);

#endif /* TT_MODULE_OTA_H */
