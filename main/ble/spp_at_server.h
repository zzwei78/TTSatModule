/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef SPP_AT_SERVER_H
#define SPP_AT_SERVER_H

#include <stdint.h>
#include "freertos/queue.h"
#include "host/ble_hs.h"

// SPP AT Service UUIDs
#define BLE_SVC_SPP_AT_UUID16       0xABF2
#define BLE_SVC_SPP_AT_CHR_UUID16   0xABF3

/**
 * @brief Initialize the SPP AT server
 * 
 * This function initializes the SPP AT service for AT command pass-through.
 * 
 * @return 0 on success, negative error code otherwise
 */
int spp_at_server_init(void);

/**
 * @brief Send AT command response via BLE GATT notifications
 *
 * This function sends an AT command response to the specified connection.
 *
 * @param conn_handle BLE connection handle
 * @param data AT command response data
 * @param len Length of response data
 * @return 0 on success, negative error code otherwise
 */
int spp_at_server_send_response(uint16_t conn_handle, const uint8_t *data, uint16_t len);

/**
 * @brief Send AT command response (non-blocking, queues for later sending)
 *
 * This function queues the response for later sending by spp_at_cmd_task.
 * It is safe to call from uart_rx_task or other high-priority contexts.
 *
 * @param conn_handle BLE connection handle
 * @param data AT command response data
 * @param len Length of response data
 * @return 0 on success (queued), negative error code otherwise
 */
int spp_at_server_send_response_async(uint16_t conn_handle, const uint8_t *data, uint16_t len);

/**
 * @brief Clean up AT server state on BLE disconnect
 *
 * Only clears state if the disconnected connection matches the active connection.
 * This is important for multi-connection support where DEBUG connections may
 * disconnect without affecting the PRIMARY connection's AT service.
 *
 * @param conn_handle BLE connection handle that disconnected
 */
void spp_at_server_cleanup_on_disconnect(uint16_t conn_handle);

#endif /* SPP_AT_SERVER_H */