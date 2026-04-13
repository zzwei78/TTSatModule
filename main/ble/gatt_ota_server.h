/*
 * gatt_ota_server.h - GATT OTA Server for Firmware Update
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef GATT_OTA_SERVER_H
#define GATT_OTA_SERVER_H

#include <stdint.h>
#include "host/ble_hs.h"

/* OTA Service UUIDs */
#define BLE_SVC_OTA_UUID16           0xABF8
#define BLE_SVC_OTA_CHR_CONTROL_UUID16   0xABF9  // Control characteristic (start/abort)
#define BLE_SVC_OTA_CHR_DATA_UUID16      0xABFA  // Data characteristic (firmware data)
#define BLE_SVC_OTA_CHR_STATUS_UUID16    0xABFB  // Status characteristic (read status)

/* OTA Control Commands */
#define OTA_CMD_START_MCU           0x01  // Start MCU firmware update
#define OTA_CMD_START_TT            0x02  // Start Tiantong module firmware update
#define OTA_CMD_ABORT               0x03  // Abort current OTA operation

/* OTA Status Codes */
#define OTA_STATUS_IDLE             0x00
#define OTA_STATUS_WRITING          0x01
#define OTA_STATUS_VERIFYING        0x02
#define OTA_STATUS_SUCCESS          0x03
#define OTA_STATUS_FAILED           0x04

/* OTA Response Codes */
#define OTA_RESP_OK                 0x00
#define OTA_RESP_ERROR              0x01
#define OTA_RESP_INVALID_CMD        0x02
#define OTA_RESP_INVALID_STATE      0x03
#define OTA_RESP_SIZE_MISMATCH      0x04
#define OTA_RESP_CRC_MISMATCH       0x05
#define OTA_RESP_INVALID_PACKET     0x06  // Invalid packet format
#define OTA_RESP_SEQ_ERROR          0x07  // Sequence number error
#define OTA_RESP_PACKET_CRC_ERROR   0x08  // Packet CRC error

/**
 * @brief Initialize the GATT OTA server
 *
 * @return 0 on success, negative error code otherwise
 */
int gatt_ota_server_init(void);

/**
 * @brief Send OTA status notification to client
 *
 * @param conn_handle BLE connection handle
 * @param status Status code
 * @param progress Progress percentage (0-100)
 * @return 0 on success, negative error code otherwise
 */
int gatt_ota_server_send_status(uint16_t conn_handle, uint8_t status, uint8_t progress);

/**
 * @brief Enable OTA service
 */
void gatt_ota_server_enable(void);

/**
 * @brief Disable OTA service
 */
void gatt_ota_server_disable(void);

/**
 * @brief Check if OTA is in progress
 *
 * @return true if OTA is in progress, false otherwise
 */
bool gatt_ota_is_in_progress(void);

/**
 * @brief Clean up OTA state on disconnect
 *
 * Only clears OTA state if the disconnected connection matches the active OTA connection.
 * This is important for multi-connection support where DEBUG connections may
 * disconnect without affecting an ongoing OTA operation on the PRIMARY connection.
 *
 * Call this when connection is lost to reset OTA state and abort partition operation.
 * This allows the client to reconnect and start a new OTA operation.
 *
 * @param conn_handle BLE connection handle that disconnected
 */
void gatt_ota_server_cleanup_on_disconnect(uint16_t conn_handle);

#endif /* GATT_OTA_SERVER_H */
