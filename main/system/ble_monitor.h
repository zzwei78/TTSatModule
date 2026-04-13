/**
 * @file ble_monitor.h
 * @brief BLE Health Monitor
 *
 * Monitors BLE stack health by periodically checking its status.
 * Triggers system reboot if BLE stack becomes unresponsive.
 *
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: MIT
 */

#ifndef BLE_MONITOR_H
#define BLE_MONITOR_H

#include "esp_err.h"
#include <stdbool.h>

/* ========== Configuration ========== */
#define BLE_MONITOR_INTERVAL_MS       10000   // Check interval: 10 seconds
#define BLE_MONITOR_CHECK_TIMEOUT_MS  5000    // Single check timeout: 5 seconds
#define BLE_MONITOR_MAX_FAILURES      3       // Max consecutive failures before reboot

/* ========== Low Memory Threshold ========== */
#define BLE_MONITOR_LOW_HEAP_BYTES    10240   // 10KB minimum free heap

/* ========== Enable/Disable Monitor ========== */
#define BLE_MONITOR_ENABLED           1       // Set to 0 to disable

/* ========== Statistics ========== */
typedef struct {
    uint32_t check_count;           // Total health checks performed
    uint32_t failure_count;         // Total failures detected
    uint32_t recovery_count;        // System reboots triggered
    uint32_t last_failure_reason;   // Last failure reason code
    uint32_t last_failure_timestamp;// Last failure timestamp (seconds since boot)
} ble_monitor_stats_t;

/* ========== Failure Reason Codes ========== */
typedef enum {
    BLE_MONITOR_FAILURE_NONE = 0,
    BLE_MONITOR_FAILURE_HOST_DISABLED,     // ble_hs_is_enabled() returned false
    BLE_MONITOR_FAILURE_LOW_MEMORY,        // Free heap below threshold
    BLE_MONITOR_FAILURE_BLE_ADDR_READ,     // Failed to read BLE address
    BLE_MONITOR_FAILURE_STACK_LOW,         // BLE task stack watermark too low
    BLE_MONITOR_FAILURE_CHECK_TIMEOUT,     // Health check timed out
    BLE_MONITOR_FAILURE_UNKNOWN,           // Unknown error
} ble_monitor_failure_t;

/* ========== APIs ========== */

/**
 * @brief Start BLE health monitor task
 *
 * Creates a background task that periodically checks BLE stack health.
 * If consecutive failures exceed BLE_MONITOR_MAX_FAILURES, system will reboot.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_monitor_start(void);

/**
 * @brief Stop BLE health monitor task
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_monitor_stop(void);

/**
 * @brief Check if BLE monitor is running
 *
 * @return true if running, false otherwise
 */
bool ble_monitor_is_running(void);

/**
 * @brief Get BLE monitor statistics
 *
 * @param stats Pointer to store statistics
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if stats is NULL
 */
esp_err_t ble_monitor_get_stats(ble_monitor_stats_t *stats);

/**
 * @brief Clear BLE monitor statistics
 *
 * @return ESP_OK on success
 */
esp_err_t ble_monitor_clear_stats(void);

/**
 * @brief Manually trigger a health check (for testing)
 *
 * @return true if BLE is healthy, false otherwise
 */
bool ble_monitor_check_health(void);

#endif /* BLE_MONITOR_H */
