/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef BLE_GATT_SERVER_H
#define BLE_GATT_SERVER_H

#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// ============================================================
// BLE Constants
// ============================================================

// MTU size
#define SPP_GATT_MTU_SIZE   512

// BLE connection parameters (unit: 1.25ms)
#define BLE_CONN_INTERVAL_UNIT_MS    1.25f
#define BLE_VOICE_CONN_ITVL_MIN      8   // 10ms
#define BLE_VOICE_CONN_ITVL_MAX      10  // 12.5ms
#define BLE_DEFAULT_CONN_ITVL_MIN    24  // 30ms
#define BLE_DEFAULT_CONN_ITVL_MAX    40  // 50ms
#define BLE_CONN_LATENCY_NONE        0
#define BLE_SUPERVISION_TIMEOUT_400  400  // 4s in 10ms units

// Timeouts (milliseconds)
#define CONN_SUBS_MUTEX_TIMEOUT_MS   100
#define SYS_CMD_MUTEX_TIMEOUT_MS     100
#define QUEUE_TIMEOUT_SHORT_MS       10
#define REBOOT_DELAY_MS              500
#define MCU_REBOOT_DELAY_MS          2000

// Buffer sizes
#define VOICE_BUFFER_SIZE            256
#define AUDIO_FRAME_SIZE             320  // 20ms @ 8kHz, 16-bit
#define MAX_LOG_DATA_SIZE            256
#define SYSTEM_CMD_BUFFER_SIZE       256
#define SPP_AT_MAX_CMD_SIZE          256
#define SPP_AT_RESP_BUFFER_SIZE      512

// Queue sizes
#define SPP_AT_CMD_QUEUE_SIZE        4

// Task stack sizes
#define SPP_AT_TASK_STACK_SIZE       8192
#define SYS_CMD_TASK_STACK_SIZE      8192

// Task priorities
#define SPP_AT_TASK_PRIORITY         5
#define SYS_CMD_TASK_PRIORITY        5

// Error codes
#define ATT_ERR_INSUFFICIENT_RES      BLE_ATT_ERR_INSUFFICIENT_RESOURCES

// ============================================================
// Function Prototypes
// ============================================================

void ble_spp_server_host_task(void *param);
int gatt_svr_init(void);
int ble_gatt_server_init(void);

/**
 * @brief Safe GATT notification send helper with automatic mbuf cleanup
 *
 * This function ensures that the mbuf is always freed, even if the
 * notification fails. This prevents memory leaks in error paths.
 *
 * @param conn_handle Connection handle
 * @param attr_handle Attribute handle
 * @param data Data to send
 * @param len Data length
 * @return 0 on success, non-zero error code on failure
 */
int ble_gatts_send_safe_notify(uint16_t conn_handle, uint16_t attr_handle,
                                const uint8_t *data, uint16_t len);

/**
 * @brief Safe fragmented GATT notification send helper
 *
 * This function sends large data payloads in multiple fragments to respect
 * the MTU size limit. Each fragment is sent as a separate notification.
 * The mbuf is always freed, even if sending fails.
 *
 * @param conn_handle Connection handle
 * @param attr_handle Attribute handle
 * @param data Data to send
 * @param len Total data length
 * @param mtu Maximum transmission unit (payload size per notification)
 * @return 0 on success, non-zero error code on failure
 */
int ble_gatts_send_safe_notify_fragmented(uint16_t conn_handle, uint16_t attr_handle,
                                          const uint8_t *data, uint16_t len, uint16_t mtu);

#endif /* BLE_GATT_SERVER_H */
