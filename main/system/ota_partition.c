/*
 * ota_partition.c - OTA Partition Operations Implementation
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "system/ota_partition.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "syslog.h"

/* Tag for logging */
static const char *TAG = "OTA_PARTITION";

/* Global OTA context */
static ota_context_t g_ota_ctx = {
    .type = OTA_PARTITION_MCU,
    .partition = NULL,
    .total_size = 0,
    .written_size = 0,
    .crc32 = 0,
    .status = OTA_PARTITION_STATUS_IDLE,
    .initialized = false
};

/* OTA handle for ESP32 update */
static esp_ota_handle_t g_ota_handle = 0;

/* Write buffer for reducing flash write operations */
#define OTA_WRITE_BUFFER_SIZE (4 * 1024)  // 4KB buffer
static uint8_t g_write_buffer[OTA_WRITE_BUFFER_SIZE];
static size_t g_buffer_offset = 0;  // Current buffer fill level

/**
 * @brief Initialize OTA partition for writing
 */
esp_err_t ota_partition_init(ota_partition_type_t type, size_t total_size, uint32_t expected_crc32)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "ota_partition_init called: type=%d, size=%d, crc32=0x%08x",
             type, total_size, expected_crc32);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Current state: initialized=%d, handle=%d, status=%d",
             g_ota_ctx.initialized, g_ota_handle, g_ota_ctx.status);

    if (g_ota_ctx.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA already initialized, abort current operation first");
        return ESP_ERR_INVALID_STATE;
    }

    if (total_size == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Invalid total size: %d", total_size);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    if (type == OTA_PARTITION_MCU) {
        // For MCU OTA, initialize ESP-IDF OTA update directly
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Initializing MCU OTA update...");

        // Get next OTA partition (ota_0 or ota_1)
        const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
        if (update_partition == NULL) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "No OTA update partition found");
            return ESP_ERR_NOT_FOUND;
        }

        // Check if partition is large enough
        if (total_size > update_partition->size) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Firmware size (%d) exceeds partition size (%d)",
                     total_size, update_partition->size);
            return ESP_ERR_INVALID_SIZE;
        }

        // Begin OTA update
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Calling esp_ota_begin: partition=%s, size=%d",
                 update_partition->label, total_size);
        ret = esp_ota_begin(update_partition, total_size, &g_ota_handle);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "ESP OTA begin failed: %s (0x%x)", esp_err_to_name(ret), ret);
            g_ota_handle = 0;  // Ensure handle is cleared on failure
            return ret;
        }
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "ESP OTA begin success: handle=%d", g_ota_handle);

        // Initialize context for MCU OTA
        g_ota_ctx.type = type;
        g_ota_ctx.partition = update_partition;
        g_ota_ctx.total_size = total_size;
        g_ota_ctx.written_size = 0;
        g_ota_ctx.crc32 = expected_crc32;
        g_ota_ctx.status = OTA_PARTITION_STATUS_WRITING;
        g_ota_ctx.initialized = true;

        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "MCU OTA initialized: partition=%s, total_size=%d, expected_crc32=0x%08x",
                 update_partition->label, total_size, expected_crc32);

    } else if (type == OTA_PARTITION_TT) {
        // For TT OTA, just initialize context (data will be forwarded directly)
        g_ota_ctx.type = type;
        g_ota_ctx.partition = NULL;  // No partition used for TT streaming OTA
        g_ota_ctx.total_size = total_size;
        g_ota_ctx.written_size = 0;
        g_ota_ctx.crc32 = expected_crc32;
        g_ota_ctx.status = OTA_PARTITION_STATUS_WRITING;
        g_ota_ctx.initialized = true;

        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT OTA initialized: total_size=%d, expected_crc32=0x%08x (streaming mode)",
                 total_size, expected_crc32);
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Invalid OTA type: %d", type);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Flush write buffer to flash
 *
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t ota_flush_buffer(void)
{
    if (g_buffer_offset == 0) {
        return ESP_OK;  // Nothing to flush
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "Flushing %d bytes from buffer to flash", g_buffer_offset);

    esp_err_t ret = esp_ota_write(g_ota_handle, g_write_buffer, g_buffer_offset);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to flush buffer to flash: %s", esp_err_to_name(ret));
        return ret;
    }

    g_buffer_offset = 0;
    return ESP_OK;
}

/**
 * @brief Write data to OTA partition
 */
esp_err_t ota_partition_write(const uint8_t *data, size_t len)
{
    if (!g_ota_ctx.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    // Check if we have enough space
    if (g_ota_ctx.written_size + len > g_ota_ctx.total_size) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Write would exceed total size: written=%d, len=%d, total=%d",
                 g_ota_ctx.written_size, len, g_ota_ctx.total_size);
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret;

    if (g_ota_ctx.type == OTA_PARTITION_MCU) {
        size_t remaining = len;
        size_t offset = 0;

        while (remaining > 0) {
            size_t buffer_space = OTA_WRITE_BUFFER_SIZE - g_buffer_offset;
            size_t to_copy = (remaining < buffer_space) ? remaining : buffer_space;

            // Copy data to buffer
            memcpy(g_write_buffer + g_buffer_offset, data + offset, to_copy);
            g_buffer_offset += to_copy;
            offset += to_copy;
            remaining -= to_copy;

            // Flush buffer if it's full
            if (g_buffer_offset >= OTA_WRITE_BUFFER_SIZE) {
                // Safety check: should never exceed, but log if it does
                if (g_buffer_offset > OTA_WRITE_BUFFER_SIZE) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "WARNING: Buffer offset overflow (%d > %d), forcing flush",
                             g_buffer_offset, OTA_WRITE_BUFFER_SIZE);
                }
                ret = ota_flush_buffer();
                if (ret != ESP_OK) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to flush buffer: %s", esp_err_to_name(ret));
                    g_ota_ctx.status = OTA_PARTITION_STATUS_FAILED;
                    return ret;
                }
            }
        }
    } else {
        // TT OTA - handled separately via streaming
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT OTA write not supported (use streaming mode)");
        return ESP_ERR_NOT_SUPPORTED;
    }

    g_ota_ctx.written_size += len;

    SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "Buffered %d bytes, total: %d/%d, buffer: %d/%d",
             len, g_ota_ctx.written_size, g_ota_ctx.total_size, g_buffer_offset, OTA_WRITE_BUFFER_SIZE);

    return ESP_OK;
}

/**
 * @brief Finalize OTA partition and verify integrity
 */
esp_err_t ota_partition_finalize(void)
{
    if (!g_ota_ctx.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Flush any remaining data in buffer
    if (g_buffer_offset > 0) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Finalizing: flushing remaining %d bytes", g_buffer_offset);
        esp_err_t ret = ota_flush_buffer();
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to flush remaining data: %s", esp_err_to_name(ret));
            g_ota_ctx.status = OTA_PARTITION_STATUS_FAILED;
            return ret;
        }
    }

    // Check if all data has been written
    if (g_ota_ctx.written_size != g_ota_ctx.total_size) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Incomplete write: written=%d, expected=%d",
                 g_ota_ctx.written_size, g_ota_ctx.total_size);
        g_ota_ctx.status = OTA_PARTITION_STATUS_FAILED;
        g_ota_ctx.initialized = false;
        return ESP_ERR_INVALID_SIZE;
    }

    g_ota_ctx.status = OTA_PARTITION_STATUS_VERIFYING;

    if (g_ota_ctx.type == OTA_PARTITION_MCU) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Finalizing MCU OTA...");

        // End OTA update and verify
        esp_err_t ret = esp_ota_end(g_ota_handle);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "ESP OTA end failed: %s", esp_err_to_name(ret));
            g_ota_ctx.status = OTA_PARTITION_STATUS_FAILED;
            g_ota_ctx.initialized = false;

            // Erase partition on failure
            esp_partition_erase_range(g_ota_ctx.partition, 0, g_ota_ctx.partition->size);
            return ret;
        }

        // Set OTA partition as boot partition
        ret = esp_ota_set_boot_partition(g_ota_ctx.partition);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to set boot partition: %s", esp_err_to_name(ret));
            g_ota_ctx.status = OTA_PARTITION_STATUS_FAILED;
            g_ota_ctx.initialized = false;
            return ret;
        }

        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "MCU OTA finalized successfully");
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "New firmware set as boot partition: %s", g_ota_ctx.partition->label);

    } else if (g_ota_ctx.type == OTA_PARTITION_TT) {
        // TT OTA finalization is handled by TT module OTA task
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT OTA finalize handled by streaming task");
    }

    g_ota_ctx.status = OTA_PARTITION_STATUS_SUCCESS;
    g_ota_ctx.initialized = false;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA completed successfully");
    return ESP_OK;
}

/**
 * @brief Get current OTA status
 */
ota_status_t ota_partition_get_status(void)
{
    return g_ota_ctx.status;
}

/**
 * @brief Get written size
 */
size_t ota_partition_get_written_size(void)
{
    return g_ota_ctx.written_size;
}

/**
 * @brief Get total size
 */
size_t ota_partition_get_total_size(void)
{
    return g_ota_ctx.total_size;
}

/**
 * @brief Abort OTA operation
 */
esp_err_t ota_partition_abort(void)
{
    if (!g_ota_ctx.initialized) {
        return ESP_OK;
    }

    SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Aborting OTA operation");

    // Clear write buffer
    g_buffer_offset = 0;

    // For MCU OTA, abort the ESP OTA handle
    if (g_ota_ctx.type == OTA_PARTITION_MCU && g_ota_handle != 0) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Aborting ESP OTA handle");
        esp_err_t ret = esp_ota_abort(g_ota_handle);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "esp_ota_abort failed: %s", esp_err_to_name(ret));
        }
        g_ota_handle = 0;
    }

    // Clear all context state
    g_ota_ctx.status = OTA_PARTITION_STATUS_IDLE;  // Changed from FAILED to IDLE
    g_ota_ctx.initialized = false;
    g_ota_ctx.type = OTA_PARTITION_MCU;  // Reset to default
    g_ota_ctx.partition = NULL;
    g_ota_ctx.written_size = 0;
    g_ota_ctx.total_size = 0;
    g_ota_ctx.crc32 = 0;  // Clear CRC32

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA context cleared");
    return ESP_OK;
}

/**
 * @brief Calculate CRC32 of data
 */
uint32_t ota_partition_calculate_crc32(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }
    return esp_crc32_le(0, data, len);
}

/**
 * @brief Get current OTA partition type
 */
ota_partition_type_t ota_partition_get_type(void)
{
    if (!g_ota_ctx.initialized) {
        return OTA_PARTITION_MCU;  // Default
    }
    return g_ota_ctx.type;
}

/**
 * @brief Get OTA partition name
 */
const char* ota_partition_get_name(void)
{
    if (!g_ota_ctx.initialized || g_ota_ctx.partition == NULL) {
        return "";
    }
    return g_ota_ctx.partition->label;
}
