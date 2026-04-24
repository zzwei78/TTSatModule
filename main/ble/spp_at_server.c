/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "tt/tt_module.h"
#include "tt/gsm0710_manager.h"
#include "ble/spp_at_server.h"
#include "ble/ble_gatt_server.h"
#include "audio/voice_packet_handler.h"
#include "system/syslog.h"

static const char *TAG = "SPP_AT_SERVER";

/* AT command queue configuration */
/* Note: Constants are now defined in ble_gatt_server.h */

/* AT command queue item */
typedef struct {
    uint8_t data[SPP_AT_MAX_CMD_SIZE];
    size_t len;
    uint16_t conn_handle;
} __attribute__((packed)) spp_at_cmd_item_t;

/* AT response queue item (for async sending from uart_rx_task) */
typedef struct {
    uint8_t data[SPP_AT_RESP_BUFFER_SIZE];  // Larger buffer for responses
    size_t len;
    uint16_t conn_handle;
} __attribute__((packed)) spp_at_resp_item_t;

/* Queue and task handles */
static QueueHandle_t g_spp_at_cmd_queue = NULL;
static QueueHandle_t g_spp_at_resp_queue = NULL;
static TaskHandle_t g_spp_at_task_handle = NULL;

// SPP AT service characteristic value handle
uint16_t spp_at_service_val_handle;

/**
 * @brief Fixed AT command processing task
 *
 * This task runs continuously and processes:
 * 1. AT commands from the command queue (from BLE GATT)
 * 2. AT responses from the response queue (from uart_rx_task)
 *
 * This avoids blocking uart_rx_task with BLE notifications.
 */
static void spp_at_cmd_task(void *pvParameters)
{
    (void)pvParameters;
    spp_at_cmd_item_t cmd_item;
    spp_at_resp_item_t resp_item;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "SPP AT command task started");

    while (1) {
        // Priority: Process RESPONSES first, then COMMANDS
        // This ensures responses are sent immediately even when commands arrive continuously
        bool processed = false;

        // Try response queue FIRST (timeout QUEUE_TIMEOUT_SHORT_MS)
        if (xQueueReceive(g_spp_at_resp_queue, &resp_item, pdMS_TO_TICKS(QUEUE_TIMEOUT_SHORT_MS)) == pdTRUE) {
            // Send response via BLE (this is safe now, we're in low-priority task)
            int rc = spp_at_server_send_response(resp_item.conn_handle, resp_item.data, resp_item.len);
            if (rc != 0) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Failed to send queued response: rc=%d", rc);
            }
            processed = true;
        }

        // Then try command queue (timeout QUEUE_TIMEOUT_SHORT_MS)
        if (xQueueReceive(g_spp_at_cmd_queue, &cmd_item, pdMS_TO_TICKS(QUEUE_TIMEOUT_SHORT_MS)) == pdTRUE) {
            // Ensure null termination
            char *cmd_str = (char *)cmd_item.data;
            size_t cmd_len = cmd_item.len;

            // Remove trailing \n first
            if (cmd_len >= 1 && cmd_str[cmd_len - 1] == '\n') {
                cmd_str[cmd_len - 1] = '\0';
                cmd_len--;
            }
            // Then remove trailing \r (if present after removing \n)
            if (cmd_len >= 1 && cmd_str[cmd_len - 1] == '\r') {
                cmd_str[cmd_len - 1] = '\0';
                cmd_len--;
            }

            // Send AT command via new GATT API (asynchronous, response will be routed back)
            tt_at_result_t result = tt_module_send_at_cmd_gatt(cmd_str, cmd_item.conn_handle);

            // Check send result
            if (result != TT_AT_RESULT_OK) {
                const char *error_msg = "ERROR\r\n";
                spp_at_server_send_response(cmd_item.conn_handle, (const uint8_t *)error_msg, strlen(error_msg));
                SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "AT command send failed: %s", cmd_str);
            }
            // Success case: log is handled by tt_module for consistency
            processed = true;
        }

        // If nothing processed, yield to other tasks
        if (!processed) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    // Task should never reach here
    vTaskDelete(NULL);
}

// Handle AT command passthrough data
static int spp_at_service_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // DEBUG: Log ALL access attempts
    SYS_LOGD_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "AT Service access: op=%d, conn=%d, attr=0x%x, len=%d",
             ctxt->op, conn_handle, attr_handle, ctxt->om ? ctxt->om->om_len : 0);

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        SYS_LOGD_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "AT Service - Read request received");
        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        {
            uint16_t data_len = ctxt->om->om_len;

            SYS_LOGD_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "AT Service - Data received: len=%d", data_len);

            // Check size limit
            if (data_len > SPP_AT_MAX_CMD_SIZE) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Command too large: %d", data_len);
                return BLE_HS_ENOMEM;
            }

            // Prepare command queue item
            spp_at_cmd_item_t cmd_item;
            cmd_item.len = data_len;
            cmd_item.conn_handle = conn_handle;

            int rc = ble_hs_mbuf_to_flat(ctxt->om, cmd_item.data, data_len, NULL);
            if (rc != 0) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Failed to copy command data: %d", rc);
                return rc;
            }
            cmd_item.data[cmd_item.len] = '\0';

            // Send to queue (non-blocking, with 0 timeout)
            // If queue is full, drop the command and return error
            if (xQueueSend(g_spp_at_cmd_queue, &cmd_item, 0) != pdTRUE) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Command queue full, dropping command");
                return BLE_HS_ENOMEM;
            }

            SYS_LOGD_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Command queued for processing");
        }
        break;

    default:
        SYS_LOGD_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "AT Service - Default callback");
        break;
    }
    return 0;
}

// Send AT command response
int spp_at_server_send_response(uint16_t conn_handle, const uint8_t *data, uint16_t len)
{
    if (!data || len == 0) {
        return BLE_HS_EINVAL;
    }

    // Use safe wrapper that ensures mbuf is always freed
    int rc = ble_gatts_send_safe_notify(conn_handle, spp_at_service_val_handle, data, len);
    if (rc != 0) {
        // Change to DEBUG level - this is normal if client hasn't subscribed
        // Common error: BLE_HS_EDONE (client not subscribed)
        SYS_LOGD_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Failed to send notification (client may not be subscribed): rc=%d", rc);
        return rc;
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "AT response sent successfully: len=%d", len);
    return 0;
}

/**
 * @brief Send AT command response (non-blocking, queues for later sending)
 *
 * This function queues the response for later sending by spp_at_cmd_task.
 * It is safe to call from uart_rx_task or other high-priority contexts.
 *
 * @param conn_handle BLE connection handle
 * @param data Response data
 * @param len Data length
 * @return 0 on success (queued), non-zero on error (queue full or not initialized)
 */
int spp_at_server_send_response_async(uint16_t conn_handle, const uint8_t *data, uint16_t len)
{
    if (!data || len == 0) {
        return BLE_HS_EINVAL;
    }

    if (g_spp_at_resp_queue == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Response queue not initialized");
        return BLE_HS_EINVAL;
    }

    // Check size limit
    if (len > sizeof(((spp_at_resp_item_t*)0)->data)) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Response too large: %d (max %zu)",
                 len, sizeof(((spp_at_resp_item_t*)0)->data));
        return BLE_HS_ENOMEM;
    }

    // Prepare response queue item
    spp_at_resp_item_t resp_item;
    resp_item.len = len;
    resp_item.conn_handle = conn_handle;
    memcpy(resp_item.data, data, len);

    // Send to queue (non-blocking, with 0 timeout)
    // If queue is full, drop the response and return error
    if (xQueueSend(g_spp_at_resp_queue, &resp_item, 0) != pdTRUE) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Response queue full, dropping response");
        return BLE_HS_ENOMEM;
    }

    return 0;
}

// SPP AT service definition
static const struct ble_gatt_svc_def spp_at_service_defs[] = {
    {
        /*** Service: SPP AT Command ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_AT_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* AT Command Passthrough Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_AT_CHR_UUID16),
                .access_cb = spp_at_service_handler,
                .val_handle = &spp_at_service_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services. */
    },
};

// Register SPP AT service
int spp_at_server_init(void)
{
    int rc;

    // Create command queue
    g_spp_at_cmd_queue = xQueueCreate(SPP_AT_CMD_QUEUE_SIZE, sizeof(spp_at_cmd_item_t));
    if (g_spp_at_cmd_queue == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Failed to create AT command queue");
        return ESP_ERR_NO_MEM;
    }

    // Create response queue (for async sending from uart_rx_task)
    g_spp_at_resp_queue = xQueueCreate(SPP_AT_CMD_QUEUE_SIZE, sizeof(spp_at_resp_item_t));
    if (g_spp_at_resp_queue == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Failed to create AT response queue");
        vQueueDelete(g_spp_at_cmd_queue);
        g_spp_at_cmd_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Create fixed AT command processing task
    BaseType_t task_ret = xTaskCreate(
        spp_at_cmd_task,
        "spp_at_task",
        SPP_AT_TASK_STACK_SIZE,
        NULL,
        SPP_AT_TASK_PRIORITY,
        &g_spp_at_task_handle
    );

    if (task_ret != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Failed to create AT command task");
        vQueueDelete(g_spp_at_resp_queue);
        g_spp_at_resp_queue = NULL;
        vQueueDelete(g_spp_at_cmd_queue);
        g_spp_at_cmd_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Calculate configuration space required for service
    rc = ble_gatts_count_cfg(spp_at_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Failed to count SPP AT service config: rc=%d", rc);
        vTaskDelete(g_spp_at_task_handle);
        g_spp_at_task_handle = NULL;
        vQueueDelete(g_spp_at_resp_queue);
        g_spp_at_resp_queue = NULL;
        vQueueDelete(g_spp_at_cmd_queue);
        g_spp_at_cmd_queue = NULL;
        return rc;
    }

    // Add service
    rc = ble_gatts_add_svcs(spp_at_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Failed to add SPP AT service: rc=%d", rc);
        vTaskDelete(g_spp_at_task_handle);
        g_spp_at_task_handle = NULL;
        vQueueDelete(g_spp_at_resp_queue);
        g_spp_at_resp_queue = NULL;
        vQueueDelete(g_spp_at_cmd_queue);
        g_spp_at_cmd_queue = NULL;
        return rc;
    }

    // Note: With the new unified AT command context, AT responses are routed automatically
    // via tt_module_route_at_response() which is registered in tt_module.c
    // We don't need to register any callbacks here anymore

    SYS_LOGI_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "SPP AT server initialized successfully (queue size: %d)", SPP_AT_CMD_QUEUE_SIZE);
    return 0;
}

/**
 * @brief Clean up AT server state on BLE disconnect
 *
 * With the new unified AT command context design, GATT connection handles are
 * managed directly in the AT context. This function updates the last active connection.
 */
void spp_at_server_cleanup_on_disconnect(uint16_t conn_handle)
{
    // Notify tt_module about the disconnection
    extern void tt_module_notify_gatt_disconnect(uint16_t conn_handle);
    tt_module_notify_gatt_disconnect(conn_handle);

    SYS_LOGD_MODULE(SYS_LOG_MODULE_SPP_AT, TAG, "Cleanup called for conn=%d", conn_handle);
}
