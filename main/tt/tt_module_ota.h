/*
 * tt_module_ota.h - Tiantong Module OTA Update Interface
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
    TT_OTA_STATE_IDLE = 0,           // Idle, not updating
    TT_OTA_STATE_PREPARING,          // Preparing for update (stopping services, etc.)
    TT_OTA_STATE_WAITING_READY,      // Waiting for modem to be ready
    TT_OTA_STATE_UPLOADING,          // Uploading firmware via XMODEM
    TT_OTA_STATE_VERIFYING,          // Verifying firmware
    TT_OTA_STATE_COMPLETED,          // Update completed successfully
    TT_OTA_STATE_FAILED              // Update failed
} tt_ota_state_t;

/* OTA Update Result */
typedef enum {
    TT_OTA_RESULT_OK = 0,            // Success
    TT_OTA_RESULT_ERROR_UART,        // UART error
    TT_OTA_RESULT_ERROR_TIMEOUT,     // Timeout
    TT_OTA_RESULT_ERROR_MODEM,       // Modem not ready
    TT_OTA_RESULT_ERROR_TRANSMISSION,// Transmission error
    TT_OTA_RESULT_ERROR_VERIFY       // Verification error
} tt_ota_result_t;

/* OTA Update Progress Callback */
typedef void (*tt_ota_progress_cb_t)(tt_ota_state_t state, int progress);

/**
 * @brief Initialize TT Module OTA Update
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_ota_init(void);

/**
 * @brief Deinitialize TT Module OTA Update
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_ota_deinit(void);

/**
 * @brief Start OTA update for Tiantong Module
 *
 * This function will:
 * 1. Stop normal communication (MUX mode, AT command task)
 * 2. Reset modem to normal AT mode
 * 3. Send AT+UPDATE command to enter update mode
 * 4. Change baudrate to high speed (3000000)
 * 5. Read firmware from ota_tt partition and send via XMODEM protocol
 * 6. Verify and complete
 *
 * @param progress_cb Optional callback for progress updates
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_ota_start_update(tt_ota_progress_cb_t progress_cb);

/**
 * @brief Cancel ongoing OTA update
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_ota_cancel(void);

/**
 * @brief Get current OTA state
 *
 * @return tt_ota_state_t Current state
 */
tt_ota_state_t tt_module_ota_get_state(void);

/**
 * @brief Check if OTA update is in progress
 *
 * @return true if updating, false otherwise
 */
bool tt_module_ota_is_in_progress(void);

/**
 * @brief Start streaming OTA update for Tiantong Module
 *
 * This function will:
 * 1. Stop normal communication (MUX mode, AT command task)
 * 2. Reset modem to normal AT mode
 * 3. Send AT+UPDATE command to enter update mode
 * 4. Change baudrate to high speed (3000000)
 * 5. Wait for firmware data to be streamed via tt_module_ota_write_data()
 *
 * @param total_size Total firmware size to expect
 * @param progress_cb Optional callback for progress updates
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_ota_start_streaming(size_t total_size, tt_ota_progress_cb_t progress_cb);

/**
 * @brief Write firmware data during streaming OTA
 *
 * This function writes data chunks directly to the TT module via XMODEM.
 * Each chunk can be up to 4KB in size for efficiency.
 *
 * @param data Firmware data buffer
 * @param len Data length (max 4096 bytes)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_ota_write_data(const uint8_t *data, size_t len);

/**
 * @brief Finalize streaming OTA update
 *
 * This function sends EOT and verifies completion.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_ota_finalize_streaming(void);

#endif /* TT_MODULE_OTA_H */
