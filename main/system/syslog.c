/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "host/ble_hs.h"
#include "syslog.h"
#include "ble/gatt_log_server.h"
#include "ble/ble_conn_manager.h"

static const char *TAG = "SYSLOG";

// ============================================================
// Global State
// ============================================================

// Global log level
static syslog_level_t g_global_level = SYS_LOG_LEVEL_INFO;

// Module configuration table
static syslog_module_config_t g_module_configs[] = {
    {SYS_LOG_MODULE_MAIN,         SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_BLE_GATT,     SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_AUDIO_PROC,   SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_VOICE_PACKET, SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_AT_CMD,       SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_SPP_AT,       SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_TT_MODULE,    SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_OTA,          SYS_LOG_LEVEL_INFO, true,  false},
};
#define NUM_MODULE_CONFIGS (sizeof(g_module_configs) / sizeof(g_module_configs[0]))

// GATT global enable switch
static bool g_gatt_global_enabled = false;

// GATT connection handle
static uint16_t g_gatt_conn_handle = 0;
static SemaphoreHandle_t g_conn_handle_mutex = NULL;

// GATT blocking state
static syslog_gatt_state_t g_gatt_state = SYS_LOG_GATT_STATE_IDLE;
static uint8_t g_gatt_block_reasons = 0;
static SemaphoreHandle_t g_gatt_state_mutex = NULL;

// ============================================================
// GATT Ring Buffer
// ============================================================

typedef struct {
    uint8_t buffer[SYS_LOG_GATT_BUF_SIZE];
    size_t head;
    size_t tail;
    size_t count;
    size_t dropped_bytes;
    SemaphoreHandle_t mutex;
    TaskHandle_t send_task_handle;
} syslog_gatt_buffer_t;

static syslog_gatt_buffer_t g_gatt_buf = {
    .head = 0,
    .tail = 0,
    .count = 0,
    .dropped_bytes = 0,
    .mutex = NULL,
    .send_task_handle = NULL
};

// ============================================================
// Forward Declarations
// ============================================================

static syslog_module_config_t *syslog_find_module_config(syslog_module_t module);
static size_t syslog_gatt_buffer_write(const uint8_t *data, size_t len);
static size_t syslog_gatt_buffer_read(uint8_t *data, size_t len);
static void syslog_gatt_buffer_clear(void);
static void syslog_gatt_send_task(void *arg);
static int syslog_format_message(char *buffer, size_t buf_size, const char *tag, const char *format, va_list args);

// ============================================================
// Initialization and Cleanup
// ============================================================

esp_err_t syslog_init(void)
{
    ESP_LOGI(TAG, "Initializing syslog module...");

    // Create mutex for connection handle
    g_conn_handle_mutex = xSemaphoreCreateMutex();
    if (!g_conn_handle_mutex) {
        ESP_LOGE(TAG, "Failed to create connection handle mutex");
        return ESP_ERR_NO_MEM;
    }

    // Create mutex for GATT state
    g_gatt_state_mutex = xSemaphoreCreateMutex();
    if (!g_gatt_state_mutex) {
        ESP_LOGE(TAG, "Failed to create GATT state mutex");
        vSemaphoreDelete(g_conn_handle_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Create mutex for GATT buffer
    g_gatt_buf.mutex = xSemaphoreCreateMutex();
    if (!g_gatt_buf.mutex) {
        ESP_LOGE(TAG, "Failed to create GATT buffer mutex");
        vSemaphoreDelete(g_conn_handle_mutex);
        vSemaphoreDelete(g_gatt_state_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Create GATT send task
    BaseType_t ret = xTaskCreate(
        syslog_gatt_send_task,
        "syslog_gatt",
        SYS_LOG_GATT_SEND_STACK,
        NULL,
        SYS_LOG_GATT_SEND_PRIO,
        &g_gatt_buf.send_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GATT send task");
        vSemaphoreDelete(g_conn_handle_mutex);
        vSemaphoreDelete(g_gatt_state_mutex);
        vSemaphoreDelete(g_gatt_buf.mutex);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Syslog module initialized successfully");
    ESP_LOGI(TAG, "  - Module configs: %d", NUM_MODULE_CONFIGS);
    ESP_LOGI(TAG, "  - GATT buffer: %d bytes", SYS_LOG_GATT_BUF_SIZE);

    return ESP_OK;
}

esp_err_t syslog_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing syslog module...");

    // Stop GATT send task
    if (g_gatt_buf.send_task_handle) {
        vTaskDelete(g_gatt_buf.send_task_handle);
        g_gatt_buf.send_task_handle = NULL;
    }

    // Delete mutexes
    if (g_conn_handle_mutex) {
        vSemaphoreDelete(g_conn_handle_mutex);
        g_conn_handle_mutex = NULL;
    }

    if (g_gatt_state_mutex) {
        vSemaphoreDelete(g_gatt_state_mutex);
        g_gatt_state_mutex = NULL;
    }

    if (g_gatt_buf.mutex) {
        vSemaphoreDelete(g_gatt_buf.mutex);
        g_gatt_buf.mutex = NULL;
    }

    ESP_LOGI(TAG, "Syslog module deinitialized");

    return ESP_OK;
}

// ============================================================
// Core Check Functions
// ============================================================

static syslog_module_config_t *syslog_find_module_config(syslog_module_t module)
{
    if (module == SYS_LOG_MODULE_ALL) {
        return NULL;  // ALL is not a valid module for lookup
    }

    for (size_t i = 0; i < NUM_MODULE_CONFIGS; i++) {
        if (g_module_configs[i].module == module) {
            return &g_module_configs[i];
        }
    }

    return NULL;
}

bool syslog_is_output_allowed(syslog_module_t module, syslog_level_t level)
{
    // Find module config
    syslog_module_config_t *config = syslog_find_module_config(module);
    if (!config) {
        return false;  // Unknown module
    }

    // Check if module is enabled
    if (!config->enabled) {
        return false;
    }

    // Check if level meets both global and module requirements
    if (level < g_global_level || level < config->level) {
        return false;
    }

    return true;
}

static bool syslog_should_output_to_gatt(syslog_module_t module)
{
    // 1. Check global GATT switch
    if (!g_gatt_global_enabled) {
        return false;
    }

    // 2. Check GATT blocking state
    if (g_gatt_state != SYS_LOG_GATT_STATE_IDLE) {
        return false;
    }

    // 3. Check module GATT configuration
    syslog_module_config_t *config = syslog_find_module_config(module);
    if (!config) {
        return false;
    }

    if (!config->enabled) {
        return false;
    }

    if (!config->gatt_output) {
        return false;
    }

    // 4. Check connection handle
    if (g_gatt_conn_handle == 0) {
        return false;
    }

    return true;
}

// ============================================================
// Control Interface Implementation
// ============================================================

esp_err_t syslog_set_global_level(syslog_level_t level)
{
    if (level > SYS_LOG_LEVEL_NONE) {
        ESP_LOGE(TAG, "Invalid log level: %d", level);
        return ESP_ERR_INVALID_ARG;
    }

    g_global_level = level;
    ESP_LOGI(TAG, "Global log level set to %d", level);

    return ESP_OK;
}

syslog_level_t syslog_get_global_level(void)
{
    return g_global_level;
}

esp_err_t syslog_set_module_config(syslog_module_t module,
                                   syslog_level_t level,
                                   bool enabled,
                                   bool gatt_output)
{
    if (level > SYS_LOG_LEVEL_NONE) {
        ESP_LOGE(TAG, "Invalid log level: %d", level);
        return ESP_ERR_INVALID_ARG;
    }

    if (module == SYS_LOG_MODULE_ALL) {
        // Set all modules
        for (size_t i = 0; i < NUM_MODULE_CONFIGS; i++) {
            g_module_configs[i].level = level;
            g_module_configs[i].enabled = enabled;
            g_module_configs[i].gatt_output = gatt_output;
        }
        ESP_LOGI(TAG, "All modules config set: level=%d, enabled=%d, gatt=%d",
                 level, enabled, gatt_output);
    } else {
        // Set specific module
        syslog_module_config_t *config = syslog_find_module_config(module);
        if (!config) {
            ESP_LOGE(TAG, "Invalid module ID: %d", module);
            return ESP_ERR_INVALID_ARG;
        }

        config->level = level;
        config->enabled = enabled;
        config->gatt_output = gatt_output;
        ESP_LOGI(TAG, "Module %d config set: level=%d, enabled=%d, gatt=%d",
                 module, level, enabled, gatt_output);
    }

    return ESP_OK;
}

esp_err_t syslog_get_module_config(syslog_module_t module,
                                   syslog_module_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    syslog_module_config_t *cfg = syslog_find_module_config(module);
    if (!cfg) {
        ESP_LOGE(TAG, "Module %d not found", module);
        return ESP_ERR_INVALID_ARG;
    }

    *config = *cfg;
    return ESP_OK;
}

esp_err_t syslog_get_all_configs(syslog_module_config_t *configs,
                                 int max_count,
                                 int *actual_count)
{
    if (!configs || max_count <= 0 || !actual_count) {
        return ESP_ERR_INVALID_ARG;
    }

    *actual_count = (max_count < NUM_MODULE_CONFIGS) ? max_count : NUM_MODULE_CONFIGS;

    for (int i = 0; i < *actual_count; i++) {
        configs[i] = g_module_configs[i];
    }

    return ESP_OK;
}

// ============================================================
// GATT Connection Management
// ============================================================

void syslog_set_gatt_conn_handle(uint16_t conn_handle)
{
    if (g_conn_handle_mutex) {
        xSemaphoreTake(g_conn_handle_mutex, portMAX_DELAY);
        g_gatt_conn_handle = conn_handle;
        xSemaphoreGive(g_conn_handle_mutex);

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
        ble_conn_role_t role = ble_conn_manager_get_role(conn_handle);
        ESP_LOGI(TAG, "GATT conn handle set: %d (role: %s)",
                 conn_handle,
                 role == BLE_CONN_ROLE_PRIMARY ? "PRIMARY" :
                 role == BLE_CONN_ROLE_DEBUG ? "DEBUG" : "UNKNOWN");
#else
        ESP_LOGI(TAG, "GATT conn handle set: %d", conn_handle);
#endif
    }
}

uint16_t syslog_get_gatt_conn_handle(void)
{
    uint16_t conn_handle = 0;
    if (g_conn_handle_mutex) {
        xSemaphoreTake(g_conn_handle_mutex, portMAX_DELAY);

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
        // Log routing logic:
        // 1. If DEBUG connection exists → route to DEBUG
        // 2. Else if PRIMARY connection exists → route to PRIMARY
        // 3. Else → no connection
        uint16_t debug_handle = ble_conn_manager_get_debug_handle();
        if (debug_handle != 0) {
            conn_handle = debug_handle;
        } else {
            uint16_t primary_handle = ble_conn_manager_get_primary_handle();
            conn_handle = primary_handle;
        }
#else
        conn_handle = g_gatt_conn_handle;
#endif

        xSemaphoreGive(g_conn_handle_mutex);
    }
    return conn_handle;
}

void syslog_clear_gatt_conn_handle(void)
{
    if (g_conn_handle_mutex) {
        xSemaphoreTake(g_conn_handle_mutex, portMAX_DELAY);
        g_gatt_conn_handle = 0;
        xSemaphoreGive(g_conn_handle_mutex);

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
        // Log routing will be handled by syslog_get_gatt_conn_handle()
        uint16_t debug_handle = ble_conn_manager_get_debug_handle();
        uint16_t primary_handle = ble_conn_manager_get_primary_handle();

        if (debug_handle != 0) {
            ESP_LOGI(TAG, "GATT conn handle cleared, logs will route to DEBUG (handle=%d)", debug_handle);
        } else if (primary_handle != 0) {
            ESP_LOGI(TAG, "GATT conn handle cleared, logs will route to PRIMARY (handle=%d)", primary_handle);
        } else {
            ESP_LOGI(TAG, "GATT conn handle cleared, no active connections");
        }
#else
        ESP_LOGI(TAG, "GATT conn handle cleared");
#endif
    }
}

void syslog_set_gatt_global_enabled(bool enabled)
{
    g_gatt_global_enabled = enabled;
    ESP_LOGI(TAG, "GATT global %s", enabled ? "enabled" : "disabled");
}

bool syslog_is_gatt_global_enabled(void)
{
    return g_gatt_global_enabled;
}

// ============================================================
// GATT Blocking Management
// ============================================================

void syslog_block_gatt_output(syslog_gatt_block_reason_t reason)
{
    if (g_gatt_state_mutex) {
        xSemaphoreTake(g_gatt_state_mutex, portMAX_DELAY);

        g_gatt_block_reasons |= (1 << reason);

        if (g_gatt_state == SYS_LOG_GATT_STATE_IDLE) {
            g_gatt_state = SYS_LOG_GATT_STATE_BLOCKED;

            // Clear GATT log buffer (Voice period logs will not be resent)
            syslog_gatt_buffer_clear();

            ESP_LOGI(TAG, "GATT output blocked and buffer cleared: reason=%d", reason);
        }

        xSemaphoreGive(g_gatt_state_mutex);
    }
}

void syslog_unblock_gatt_output(syslog_gatt_block_reason_t reason)
{
    if (g_gatt_state_mutex) {
        xSemaphoreTake(g_gatt_state_mutex, portMAX_DELAY);

        g_gatt_block_reasons &= ~(1 << reason);

        if (g_gatt_block_reasons == 0 && g_gatt_state == SYS_LOG_GATT_STATE_BLOCKED) {
            g_gatt_state = SYS_LOG_GATT_STATE_IDLE;
            ESP_LOGI(TAG, "GATT output unblocked: all reasons cleared");
        }

        xSemaphoreGive(g_gatt_state_mutex);
    }
}

bool syslog_is_gatt_blocked(void)
{
    return (g_gatt_state != SYS_LOG_GATT_STATE_IDLE);
}

syslog_gatt_state_t syslog_get_gatt_state(void)
{
    return g_gatt_state;
}

void syslog_get_gatt_buffer_stats(size_t *used, size_t *free, size_t *dropped)
{
    if (g_gatt_buf.mutex) {
        xSemaphoreTake(g_gatt_buf.mutex, pdMS_TO_TICKS(100));
        if (used) *used = g_gatt_buf.count;
        if (free) *free = SYS_LOG_GATT_BUF_SIZE - g_gatt_buf.count;
        if (dropped) *dropped = g_gatt_buf.dropped_bytes;
        xSemaphoreGive(g_gatt_buf.mutex);
    }
}

// ============================================================
// GATT Ring Buffer Operations
// ============================================================

static size_t syslog_gatt_buffer_write(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return 0;
    }

    size_t written = 0;

    if (xSemaphoreTake(g_gatt_buf.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Calculate available space
        size_t available = SYS_LOG_GATT_BUF_SIZE - g_gatt_buf.count;
        size_t write_len = (len < available) ? len : available;

        if (write_len == 0) {
            // Buffer full, drop new data
            g_gatt_buf.dropped_bytes += len;
            ESP_LOGW(TAG, "[GATT_BUFFER] Full! Dropped %d bytes (total dropped: %u)",
                     len, g_gatt_buf.dropped_bytes);
        } else {
            // Write data (handle ring boundary)
            size_t first_chunk = SYS_LOG_GATT_BUF_SIZE - g_gatt_buf.head;
            if (write_len <= first_chunk) {
                memcpy(&g_gatt_buf.buffer[g_gatt_buf.head], data, write_len);
                g_gatt_buf.head = (g_gatt_buf.head + write_len) % SYS_LOG_GATT_BUF_SIZE;
            } else {
                memcpy(&g_gatt_buf.buffer[g_gatt_buf.head], data, first_chunk);
                memcpy(g_gatt_buf.buffer, data + first_chunk, write_len - first_chunk);
                g_gatt_buf.head = write_len - first_chunk;
            }

            g_gatt_buf.count += write_len;
            written = write_len;

            // Notify send task
            if (g_gatt_buf.send_task_handle) {
                xTaskNotifyGive(g_gatt_buf.send_task_handle);
            }
        }

        xSemaphoreGive(g_gatt_buf.mutex);
    } else {
        ESP_LOGW(TAG, "[GATT_BUFFER] Failed to take mutex");
    }

    return written;
}

static size_t syslog_gatt_buffer_read(uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return 0;
    }

    size_t read_len = 0;

    if (xSemaphoreTake(g_gatt_buf.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        size_t available = g_gatt_buf.count;
        read_len = (len < available) ? len : available;

        // Read data (handle ring boundary)
        size_t first_chunk = SYS_LOG_GATT_BUF_SIZE - g_gatt_buf.tail;
        if (read_len <= first_chunk) {
            memcpy(data, &g_gatt_buf.buffer[g_gatt_buf.tail], read_len);
            g_gatt_buf.tail = (g_gatt_buf.tail + read_len) % SYS_LOG_GATT_BUF_SIZE;
        } else {
            memcpy(data, &g_gatt_buf.buffer[g_gatt_buf.tail], first_chunk);
            memcpy(data + first_chunk, g_gatt_buf.buffer, read_len - first_chunk);
            g_gatt_buf.tail = read_len - first_chunk;
        }

        g_gatt_buf.count -= read_len;

        xSemaphoreGive(g_gatt_buf.mutex);
    }

    return read_len;
}

static void syslog_gatt_buffer_clear(void)
{
    if (xSemaphoreTake(g_gatt_buf.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_gatt_buf.head = 0;
        g_gatt_buf.tail = 0;
        g_gatt_buf.count = 0;
        xSemaphoreGive(g_gatt_buf.mutex);

        ESP_LOGD(TAG, "GATT buffer cleared");
    }
}

// ============================================================
// GATT Send Task
// ============================================================

static void syslog_gatt_send_task(void *arg)
{
    ESP_LOGI(TAG, "GATT send task started");

    uint8_t chunk[SYS_LOG_GATT_CHUNK_SIZE];
    uint32_t total_sent = 0;
    uint32_t total_errors = 0;

    // Get the GATT log handle from gatt_log_server
    extern uint16_t log_service_val_handle;

    while (1) {
        // Wait for data notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {
            // Read from buffer
            size_t len = syslog_gatt_buffer_read(chunk, sizeof(chunk));

            if (len == 0) {
                break;  // Buffer empty
            }

            // Get connection handle
            uint16_t conn_handle = syslog_get_gatt_conn_handle();

            if (conn_handle == 0) {
                // Connection lost, clear buffer
                ESP_LOGW(TAG, "[GATT_SEND] No connection, clearing buffer");
                syslog_gatt_buffer_clear();
                break;
            }

            // Send to GATT using NimBLE API
            struct os_mbuf *txom = ble_hs_mbuf_from_flat(chunk, len);
            if (!txom) {
                ESP_LOGE(TAG, "[GATT_SEND] Failed to allocate mbuf");
                total_errors++;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            int rc = ble_gatts_notify_custom(conn_handle, log_service_val_handle, txom);

            if (rc == 0) {
                // Success, continue sending
                total_sent++;
            } else if (rc == BLE_HS_ENOMEM || rc == BLE_HS_EDONE) {
                // Queue full or operation in progress, wait a bit and retry
                vTaskDelay(pdMS_TO_TICKS(20));
            } else {
                // Other error, clear buffer and continue
                total_errors++;
                ESP_LOGW(TAG, "[GATT_SEND] Failed: rc=%d, clearing buffer (errors: %u)", rc, total_errors);
                syslog_gatt_buffer_clear();
                break;
            }
        }
    }
}

// ============================================================
// Message Formatting
// ============================================================

/**
 * @brief Format log message with timestamp and tag
 *
 * Format: "[HH:MM:SS.mmm][TAG] message"
 *
 * @param buffer Output buffer
 * @param buf_size Buffer size
 * @param tag Log tag
 * @param format Format string
 * @param args Variable arguments
 * @return Number of characters written, or -1 on error
 */
static int syslog_format_message(char *buffer, size_t buf_size, const char *tag, const char *format, va_list args)
{
    if (!buffer || buf_size == 0) {
        return -1;
    }

    // Get system time in milliseconds (same format as ESP LOG)
    TickType_t ticks = xTaskGetTickCount();
    uint32_t ms = ticks * portTICK_PERIOD_MS;

    // Format timestamp and tag: "(ms)[TAG] message" (same as ESP LOG format)
    int len = snprintf(buffer, buf_size, "(%lu)[%s] ", ms, tag);

    if (len > 0 && len < buf_size) {
        // Append the actual log message
        int msg_len = vsnprintf(buffer + len, buf_size - len, format, args);
        if (msg_len > 0) {
            len += msg_len;
        }
    }

    return len;
}

void syslog_output_to_gatt_if_allowed(syslog_module_t module,
                                       syslog_level_t level,
                                       const char *tag,
                                       const char *format, ...)
{
    // Check if should output to GATT
    if (!syslog_should_output_to_gatt(module)) {
        return;
    }

    // Use static buffer to avoid dynamic allocation
    static char msg_buffer[SYS_LOG_MAX_MSG_LEN];

    // Format message with timestamp
    va_list args;
    va_start(args, format);
    int len = syslog_format_message(msg_buffer, sizeof(msg_buffer), tag, format, args);
    va_end(args);

    if (len > 0) {
        // Write to GATT buffer
        syslog_gatt_buffer_write((const uint8_t *)msg_buffer, len);
    } else {
        ESP_LOGW(TAG, "[GATT_OUTPUT] Failed to format message");
    }
}

// ============================================================
// Remote Configuration Command Processing
// ============================================================

esp_err_t syslog_process_remote_config(const uint8_t *cmd_data,
                                       size_t cmd_len,
                                       uint8_t *response,
                                       size_t *resp_len)
{
    if (!cmd_data || cmd_len < 2) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!response || !resp_len) {
        return ESP_ERR_INVALID_ARG;
    }

    syslog_cmd_type_t cmd_type = (syslog_cmd_type_t)cmd_data[0];
    uint8_t cmd_data_len = cmd_data[1];

    if (cmd_data_len + 2 > cmd_len) {
        ESP_LOGE(TAG, "Invalid command format");
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *data = &cmd_data[2];
    *resp_len = 0;

    esp_err_t ret = ESP_OK;

    switch (cmd_type) {
        case SYS_LOG_CMD_SET_GLOBAL_LEVEL: {
            if (cmd_data_len == 1) {
                syslog_level_t level = (syslog_level_t)data[0];
                ret = syslog_set_global_level(level);

                response[(*resp_len)++] = SYS_LOG_CMD_SET_GLOBAL_LEVEL;
                response[(*resp_len)++] = 2;  // response length
                response[(*resp_len)++] = (ret == ESP_OK) ? 0 : 1;
                response[(*resp_len)++] = data[0];
            } else {
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case SYS_LOG_CMD_SET_MODULE_CONFIG: {
            if (cmd_data_len == 4) {
                syslog_module_t module = (syslog_module_t)data[0];
                syslog_level_t level = (syslog_level_t)data[1];
                bool enabled = (data[2] != 0);
                bool gatt_output = (data[3] != 0);

                ret = syslog_set_module_config(module, level, enabled, gatt_output);

                response[(*resp_len)++] = SYS_LOG_CMD_SET_MODULE_CONFIG;
                response[(*resp_len)++] = 5;  // response length
                response[(*resp_len)++] = (ret == ESP_OK) ? 0 : 1;
                response[(*resp_len)++] = data[0];  // module
                response[(*resp_len)++] = data[1];  // level
                response[(*resp_len)++] = data[2];  // enabled
                response[(*resp_len)++] = data[3];  // gatt_output
            } else {
                ESP_LOGE(TAG, "Invalid command format: expected 4 bytes, got %d", cmd_data_len);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case SYS_LOG_CMD_GET_MODULE_CONFIG: {
            if (cmd_data_len == 1) {
                syslog_module_t module = (syslog_module_t)data[0];

                if (module == SYS_LOG_MODULE_ALL) {
                    // Get all configs
                    response[(*resp_len)++] = SYS_LOG_CMD_GET_ALL_CONFIGS;
                    response[(*resp_len)++] = 1 + NUM_MODULE_CONFIGS * 4;  // length

                    for (size_t i = 0; i < NUM_MODULE_CONFIGS; i++) {
                        response[(*resp_len)++] = g_module_configs[i].module;
                        response[(*resp_len)++] = g_module_configs[i].level;
                        response[(*resp_len)++] = g_module_configs[i].enabled ? 1 : 0;
                        response[(*resp_len)++] = g_module_configs[i].gatt_output ? 1 : 0;
                    }
                } else {
                    // Get single config
                    syslog_module_config_t config;
                    ret = syslog_get_module_config(module, &config);

                    if (ret == ESP_OK) {
                        response[(*resp_len)++] = SYS_LOG_CMD_GET_MODULE_CONFIG;
                        response[(*resp_len)++] = 4;  // response length
                        response[(*resp_len)++] = config.module;
                        response[(*resp_len)++] = config.level;
                        response[(*resp_len)++] = config.enabled ? 1 : 0;
                        response[(*resp_len)++] = config.gatt_output ? 1 : 0;
                    }
                }
            } else {
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case SYS_LOG_CMD_SET_GATT_GLOBAL: {
            if (cmd_data_len == 1) {
                bool enabled = (data[0] != 0);
                syslog_set_gatt_global_enabled(enabled);

                response[(*resp_len)++] = SYS_LOG_CMD_SET_GATT_GLOBAL;
                response[(*resp_len)++] = 2;  // response length
                response[(*resp_len)++] = 0;  // success
                response[(*resp_len)++] = enabled ? 1 : 0;
            } else {
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case SYS_LOG_CMD_GET_GATT_STATE: {
            size_t used, free, dropped;
            syslog_get_gatt_buffer_stats(&used, &free, &dropped);

            response[(*resp_len)++] = SYS_LOG_CMD_GET_GATT_STATE;
            response[(*resp_len)++] = 4;  // response length
            response[(*resp_len)++] = syslog_is_gatt_global_enabled() ? 1 : 0;
            response[(*resp_len)++] = syslog_is_gatt_blocked() ? 1 : 0;
            response[(*resp_len)++] = g_gatt_block_reasons;
            break;
        }

        case SYS_LOG_CMD_GET_GATT_STATS: {
            size_t used = 0, free = 0, dropped = 0;
            syslog_get_gatt_buffer_stats(&used, &free, &dropped);

            response[(*resp_len)++] = SYS_LOG_CMD_GET_GATT_STATS;
            response[(*resp_len)++] = 3;  // response length
            response[(*resp_len)++] = used & 0xFF;
            response[(*resp_len)++] = (used >> 8) & 0xFF;
            response[(*resp_len)++] = dropped & 0xFF;
            break;
        }

        case SYS_LOG_CMD_DISABLE_ALL_GATT: {
            ret = syslog_set_module_config(SYS_LOG_MODULE_ALL,
                                            SYS_LOG_LEVEL_INFO,
                                            true,
                                            false);  // Disable all GATT

            response[(*resp_len)++] = SYS_LOG_CMD_DISABLE_ALL_GATT;
            response[(*resp_len)++] = 1;  // response length
            response[(*resp_len)++] = (ret == ESP_OK) ? 0 : 1;
            break;
        }

        default:
            ESP_LOGE(TAG, "Unknown command: 0x%02X", cmd_type);
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }

    if (ret != ESP_OK && *resp_len == 0) {
        // Send error response
        response[(*resp_len)++] = cmd_type;
        response[(*resp_len)++] = 1;  // length
        response[(*resp_len)++] = 1;  // error code
    }

    return ret;
}
