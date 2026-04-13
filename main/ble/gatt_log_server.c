/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble/gatt_log_server.h"
#include "system/syslog.h"

static const char *TAG = "GATT_LOG_SERVER";

// Log service initialization flag
static bool g_log_server_initialized = false;

// Service enabled flag
static bool g_log_service_enabled = false;

// Service registration flag
static bool g_log_service_registered = false;

// Log service characteristic value handle
uint16_t log_service_val_handle;

// Log service callback handler
static int log_service_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Check if service is enabled
    if (!g_log_service_enabled) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log service is disabled");
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        SYS_LOGD_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log Service - Read request");
        // Return empty value (not an error) - allows CCC descriptor to work properly
        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log Service - Write not supported (use System Server for configuration)");
        // Write not supported - use System Server for log configuration
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

    default:
        break;
    }
    return 0;
}

// Send log data
int gatt_log_server_send_log(uint16_t conn_handle, syslog_level_t level, const char *log_data, size_t log_len)
{
    if (!log_data || log_len == 0 || log_len > (MAX_LOG_DATA_SIZE - 1)) {
        ESP_LOGE(TAG, "Invalid log parameters: data=%p, len=%d", log_data, log_len);
        return BLE_HS_EINVAL;
    }

    // Check if connection is valid
    if (conn_handle == 0) {
        ESP_LOGW(TAG, "No valid connection, log dropped");
        return BLE_HS_ENOTCONN;
    }

    // Build log data packet: [log level][log content]
    uint8_t log_packet[MAX_LOG_DATA_SIZE];
    log_packet[0] = (uint8_t)level;
    memcpy(&log_packet[1], log_data, log_len);
    log_packet[log_len + 1] = '\0'; // Ensure null terminator

    size_t packet_len = log_len + 2; // Level(1) + log content + null terminator(1)

    struct os_mbuf *txom = ble_hs_mbuf_from_flat(log_packet, packet_len);
    if (!txom) {
        ESP_LOGE(TAG, "Failed to allocate mbuf for log");
        return BLE_HS_ENOMEM;
    }

    int rc = ble_gatts_notify_custom(conn_handle, log_service_val_handle, txom);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send log notification: rc=%d", rc);
        os_mbuf_free(txom);
        return rc;
    }

    return 0;
}

// Convenience interfaces: send logs with different levels
int gatt_log_server_send_debug(uint16_t conn_handle, const char *log_data)
{
    return gatt_log_server_send_log(conn_handle, SYS_LOG_LEVEL_DEBUG, log_data, strlen(log_data));
}

int gatt_log_server_send_info(uint16_t conn_handle, const char *log_data)
{
    return gatt_log_server_send_log(conn_handle, SYS_LOG_LEVEL_INFO, log_data, strlen(log_data));
}

int gatt_log_server_send_warn(uint16_t conn_handle, const char *log_data)
{
    return gatt_log_server_send_log(conn_handle, SYS_LOG_LEVEL_WARN, log_data, strlen(log_data));
}

int gatt_log_server_send_error(uint16_t conn_handle, const char *log_data)
{
    return gatt_log_server_send_log(conn_handle, SYS_LOG_LEVEL_ERROR, log_data, strlen(log_data));
}

// Log service definition
static const struct ble_gatt_svc_def log_service_defs[] = {
    {
        /*** Service: Log Reporting ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_LOG_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* Log Reporting Characteristic - Read + Notify */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_LOG_CHR_UUID16),
                .access_cb = log_service_handler,
                .val_handle = &log_service_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services. */
    },
};

// Register log service (only count config, do NOT register)
int gatt_log_server_init(void)
{
    int rc;

    // Prevent re-initialization
    if (g_log_server_initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "GATT log server already initialized");
        return 0;
    }

    // Calculate configuration space required for service
    rc = ble_gatts_count_cfg(log_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to count log service config: rc=%d", rc);
        return rc;
    }

    // Register service during initialization (must be done before connection)
    rc = ble_gatts_add_svcs(log_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to add log service: rc=%d", rc);
        return rc;
    }
    g_log_service_registered = true;

    g_log_server_initialized = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "GATT log server initialized (service registered)");
    return 0;
}

// Send formatted log message with level
int gatt_log_server_send_formatted_log(syslog_level_t level, const char *format, ...)
{
    if (!format) {
        return BLE_HS_EINVAL;
    }

    // Format the log message
    char log_buffer[MAX_LOG_DATA_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(log_buffer, MAX_LOG_DATA_SIZE - 4, format, args);
    va_end(args);

    if (len <= 0 || len >= MAX_LOG_DATA_SIZE - 4) {
        return BLE_HS_EINVAL;
    }

    // Note: This function would typically iterate through connected clients and send logs
    // For simplicity, we're just formatting the log here
    return 0;
}

/**
 * @brief Enable Log service
 */
void gatt_log_server_enable(void)
{
    g_log_service_enabled = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log service enabled");
}

/**
 * @brief Disable Log service
 */
void gatt_log_server_disable(void)
{
    g_log_service_enabled = false;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log service disabled");
}
