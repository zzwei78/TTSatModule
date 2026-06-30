/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef BLE_GATT_SERVER_H
#define BLE_GATT_SERVER_H

#include "esp_log.h"
#include "esp_err.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// ============================================================
// BLE Constants
// ============================================================

// MTU size
#define SPP_GATT_MTU_SIZE   512

// Device name (stored in NVS, configurable via System service)
#define BLE_DEVICE_NAME_DEFAULT    "TTCat"
#define BLE_DEVICE_NAME_MAX_LEN    29

// BLE connection parameters (unit: 1.25ms)
#define BLE_CONN_INTERVAL_UNIT_MS    1.25f
#define BLE_VOICE_CONN_ITVL_MIN      8   // 10ms
#define BLE_VOICE_CONN_ITVL_MAX      10  // 12.5ms
#define BLE_DEFAULT_CONN_ITVL_MIN    24  // 30ms
#define BLE_DEFAULT_CONN_ITVL_MAX    40  // 50ms
#define BLE_CONN_LATENCY_NONE        0
#define BLE_SUPERVISION_TIMEOUT_400  400  // 4s in 10ms units

/* Powersave mode: BLE connected but idle (Light Sleep)
 * Effective wakeup every ~1.5-3s, saves ~10x power vs normal mode.
 * BT spec requires: supervision_timeout > itvl_max*(latency+1)*2
 *   7000ms > 1000ms * 3 * 2 = 6000ms ✓ */
#define BLE_POWERSAVE_CONN_ITVL_MIN  400  // 500ms
#define BLE_POWERSAVE_CONN_ITVL_MAX  800  // 1000ms
#define BLE_POWERSAVE_CONN_LATENCY   2    // skip 2 intervals → effective 1.5-3s
#define BLE_POWERSAVE_SUPER_TIMEOUT  700  // 7s (> 6s spec minimum)

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

// ============================================================
// BLE Device Name (NVS-backed, configurable via System service)
// ============================================================

/**
 * @brief Initialize BLE device name from NVS at boot.
 *
 * Reads the custom name from NVS namespace "ble_config" (key "dev_name").
 * Falls back to BLE_DEVICE_NAME_DEFAULT ("TTCat") if no custom name is stored.
 * Applies the name to the GAP service via ble_svc_gap_device_name_set().
 */
void ble_device_name_init(void);

/**
 * @brief Set the BLE device name, persist to NVS, and refresh advertising.
 *
 * @param name Pointer to UTF-8 name bytes (need not be null-terminated).
 *             Pass NULL or len=0 to restore the default name.
 * @param len Length of name in bytes. Valid range: 0..BLE_DEVICE_NAME_MAX_LEN (29).
 *            0 restores the default name and erases the NVS entry.
 * @return ESP_OK on success,
 *         ESP_ERR_INVALID_ARG if len is out of range,
 *         ESP_FAIL / other NVS error on persistence failure.
 */
esp_err_t ble_device_name_set(const char *name, uint8_t len);

/**
 * @brief Get the current BLE device name (null-terminated).
 *
 * The returned pointer is valid until the next call to ble_device_name_set()
 * or ble_device_name_init(). Caller must NOT free it.
 *
 * @return Pointer to the current device name string.
 */
const char *ble_device_name_get(void);

// ============================================================
// BLE Connection Interval Management
// ============================================================

/**
 * @brief Request powersave connection parameters (500ms-1s, latency 2)
 *
 * Call before entering Light Sleep to reduce BLE power ~10x.
 * iOS/Android compatible. APP receives param update request and
 * typically accepts automatically.
 *
 * @return 0 on success, non-zero on error (no connection or update rejected)
 */
int ble_gatt_server_request_powersave(void);

/**
 * @brief Request normal connection parameters (30-50ms, latency 0)
 *
 * Call after waking from Light Sleep or when activity resumes
 * to restore responsive BLE latency.
 *
 * @return 0 on success, non-zero on error
 */
int ble_gatt_server_request_normal(void);

#endif /* BLE_GATT_SERVER_H */
