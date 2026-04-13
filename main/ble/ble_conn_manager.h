/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef BLE_CONN_MANAGER_H
#define BLE_CONN_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ble_conn_manager.h
 * @brief BLE Multi-Connection Manager for Debug Support
 *
 * Features:
 * - Support up to 2 simultaneous BLE connections
 * - Role-based access control (Primary vs Debug)
 * - Automatic role assignment based on connection order
 * - Connection state tracking and persistence
 * - Log routing control (Debug connection gets logs)
 *
 * Connection Roles:
 * - PRIMARY: First connection, full access (Voice, AT, OTA, System, Log)
 * - DEBUG: Second connection, restricted access (Log + specific System commands)
 *
 * Connection Rules:
 * - First connection → PRIMARY
 * - Second connection → DEBUG
 * - If PRIMARY disconnects: DEBUG keeps DEBUG role, next connection becomes PRIMARY
 * - If DEBUG disconnects: PRIMARY keeps PRIMARY role, next connection becomes DEBUG
 */

// ============================================================
// Configuration
// ============================================================

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
    #define BLE_MAX_CONNECTIONS  2
#else
    #define BLE_MAX_CONNECTIONS  1
#endif

// ============================================================
// Data Types
// ============================================================

/**
 * @brief Connection role enumeration
 */
typedef enum {
    BLE_CONN_ROLE_NONE = 0,       ///< No role / unassigned
    BLE_CONN_ROLE_PRIMARY,        ///< Primary connection (full access)
    BLE_CONN_ROLE_DEBUG,          ///< Debug connection (restricted access)
} ble_conn_role_t;

/**
 * @brief Connection state structure
 */
typedef struct {
    uint16_t conn_handle;         ///< BLE connection handle (0 if not connected)
    ble_conn_role_t role;         ///< Connection role
    bool is_connected;            ///< Connection state
    uint8_t addr[6];              ///< Peer address
    uint8_t addr_type;            ///< Peer address type (0=public, 1=random)
} ble_conn_state_t;

// ============================================================
// Connection Management API
// ============================================================

/**
 * @brief Initialize connection manager
 *
 * Must be called before any other connection manager functions.
 *
 * @return ESP_OK on success
 */
esp_err_t ble_conn_manager_init(void);

/**
 * @brief Add a new connection
 *
 * Assigns role automatically based on connection order and current state:
 * - If no PRIMARY exists → assign PRIMARY
 * - If PRIMARY exists but no DEBUG → assign DEBUG
 * - If both exist → reuse the role of disconnected connection
 *
 * Role persistence:
 * - If PRIMARY disconnects: DEBUG keeps DEBUG role
 * - If DEBUG disconnects: PRIMARY keeps PRIMARY role
 *
 * @param conn_handle BLE connection handle
 * @param addr Peer address (6 bytes)
 * @param addr_type Peer address type
 * @return Assigned role, or BLE_CONN_ROLE_NONE if failed
 */
ble_conn_role_t ble_conn_manager_add_connection(uint16_t conn_handle,
                                                const uint8_t *addr,
                                                uint8_t addr_type);

/**
 * @brief Remove a connection
 *
 * Handles role persistence:
 * - If PRIMARY disconnects: DEBUG connection keeps DEBUG role
 * - If DEBUG disconnects: PRIMARY connection keeps PRIMARY role
 *
 * @param conn_handle BLE connection handle
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not found
 */
esp_err_t ble_conn_manager_remove_connection(uint16_t conn_handle);

/**
 * @brief Get connection role by handle
 *
 * @param conn_handle BLE connection handle
 * @return Connection role (BLE_CONN_ROLE_NONE if not found)
 */
ble_conn_role_t ble_conn_manager_get_role(uint16_t conn_handle);

/**
 * @brief Check if a connection is primary
 *
 * @param conn_handle BLE connection handle
 * @return true if primary connection
 */
bool ble_conn_manager_is_primary(uint16_t conn_handle);

/**
 * @brief Check if a connection is debug
 *
 * @param conn_handle BLE connection handle
 * @return true if debug connection
 */
bool ble_conn_manager_is_debug(uint16_t conn_handle);

/**
 * @brief Get current connection count
 *
 * @return Number of active connections (0 to BLE_MAX_CONNECTIONS)
 */
int ble_conn_manager_get_connection_count(void);

/**
 * @brief Check if maximum connections reached
 *
 * @return true if max connections reached
 */
bool ble_conn_manager_is_max_connections(void);

/**
 * @brief Get primary connection handle
 *
 * @return Connection handle (0 if no primary connection)
 */
uint16_t ble_conn_manager_get_primary_handle(void);

/**
 * @brief Get debug connection handle
 *
 * @return Connection handle (0 if no debug connection)
 */
uint16_t ble_conn_manager_get_debug_handle(void);

/**
 * @brief Check if debug connection exists
 *
 * This is used by syslog to determine log routing.
 *
 * @return true if debug connection exists
 */
bool ble_conn_manager_has_debug(void);

/**
 * @brief Get connection state by handle
 *
 * @param conn_handle BLE connection handle
 * @param state Pointer to store connection state
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not found
 */
esp_err_t ble_conn_manager_get_state(uint16_t conn_handle, ble_conn_state_t *state);

/**
 * @brief Print connection information (for debugging)
 *
 * Logs current connection states to console.
 */
void ble_conn_manager_print_info(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CONN_MANAGER_H */
