/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef GATT_LOG_SERVER_H
#define GATT_LOG_SERVER_H

#include "freertos/queue.h"
#include "host/ble_hs.h"
#include "syslog.h"

// Log Service UUIDs
#define BLE_SVC_LOG_UUID16          0xABF4
#define BLE_SVC_LOG_CHR_UUID16      0xABF5

// Maximum log data size
#define MAX_LOG_DATA_SIZE           256

/**
 * @brief Initialize the GATT log server
 * 
 * This function initializes the GATT log service for log reporting.
 * 
 * @return 0 on success, negative error code otherwise
 */
int gatt_log_server_init(void);

/**
 * @brief Send log message via BLE GATT notifications
 *
 * This function sends a log message to all connected clients
 * that have subscribed to notifications.
 *
 * @param conn_handle Connection handle
 * @param level Log level
 * @param log_data Log message string
 * @param log_len Length of log message
 * @return 0 on success, negative error code otherwise
 */
int gatt_log_server_send_log(uint16_t conn_handle, syslog_level_t level, const char *log_data, size_t log_len);

/**
 * @brief Send formatted log message with level
 *
 * This function formats a log message with timestamp and log level,
 * then sends it via BLE GATT notifications.
 *
 * @param level Log level
 * @param format Format string
 * @param ... Variable arguments for format string
 * @return 0 on success, negative error code otherwise
 */
int gatt_log_server_send_formatted_log(syslog_level_t level, const char *format, ...) __attribute__((format(printf, 2, 3)));



/**
 * @brief Send debug log message
 */
int gatt_log_server_send_debug(uint16_t conn_handle, const char *log_data);

/**
 * @brief Send info log message
 */
int gatt_log_server_send_info(uint16_t conn_handle, const char *log_data);

/**
 * @brief Send warning log message
 */
int gatt_log_server_send_warn(uint16_t conn_handle, const char *log_data);

/**
 * @brief Send error log message
 */
int gatt_log_server_send_error(uint16_t conn_handle, const char *log_data);

/**
 * @brief Enable Log service
 */
void gatt_log_server_enable(void);

/**
 * @brief Disable Log service
 */
void gatt_log_server_disable(void);

#endif /* GATT_LOG_SERVER_H */