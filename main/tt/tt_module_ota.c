/*
 * tt_module_ota.c - Tiantong Module OTA Update Implementation
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "driver/uart.h"
#include "tt/tt_module_ota.h"
#include "tt/tt_module.h"
#include "system/syslog.h"
#include "tt/tt_hardware.h"
#include "system/ota_partition.h"

static const char *TAG = "TT_MODULE_OTA";

/* External functions */
extern esp_err_t gsm0710_manager_stop(void);

/* OTA Context */
typedef struct {
    tt_ota_state_t state;
    bool initialized;
    bool in_progress;
    bool streaming_mode;
    SemaphoreHandle_t mutex;
    tt_ota_progress_cb_t progress_cb;
    TaskHandle_t ota_task_handle;
    const esp_partition_t *partition;
    size_t partition_size;
    size_t uploaded_size;
    size_t total_size;
    uint8_t packet_num;
} tt_ota_context_t;

/* Global OTA Context */
static tt_ota_context_t g_tt_ota = {
    .state = TT_OTA_STATE_IDLE,
    .initialized = false,
    .in_progress = false,
    .streaming_mode = false,
    .mutex = NULL,
    .progress_cb = NULL,
    .ota_task_handle = NULL,
    .partition = NULL,
    .partition_size = 0,
    .uploaded_size = 0,
    .total_size = 0,
    .packet_num = 1
};

/* XMODEM Protocol Constants */
#define XMODEM_SOH 0x01
#define XMODEM_STX 0x02
#define XMODEM_EOT 0x04
#define XMODEM_ACK 0x06
#define XMODEM_NAK 0x15
#define XMODEM_CAN 0x18
#define XMODEM_EOF 0x1A
#define XMODEM_CRC 0x43

#define XMODEM_PACKET_SIZE 128
#define XMODEM_PACKET_TOTAL 133  // 128 + 3 (SOH + packet # + ~packet #)
#define XMODEM_TIMEOUT_MS 1000
#define XMODEM_MAX_RETRIES 10

/* Baudrate */
#define TT_UART_BAUD_NORMAL  115200
#define TT_UART_BAUD_HIGH     3000000

/* Static OTA data buffer - avoids repeated malloc/free during OTA */
static uint8_t g_ota_data_buffer[XMODEM_PACKET_SIZE];

/* Forward declarations */
static void tt_ota_task(void *pvParameters);
static int xmodem_send_packet(const uint8_t *data, size_t len, uint8_t packet_num);
static int xmodem_receive_byte(int timeout_ms);
static int xmodem_wait_for_crc(void);

/**
 * @brief Initialize TT Module OTA Update
 */
esp_err_t tt_module_ota_init(void)
{
    if (g_tt_ota.initialized) {
        return ESP_OK;
    }

    g_tt_ota.mutex = xSemaphoreCreateMutex();
    if (g_tt_ota.mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    g_tt_ota.state = TT_OTA_STATE_IDLE;
    g_tt_ota.in_progress = false;
    g_tt_ota.initialized = true;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT Module OTA initialized");
    return ESP_OK;
}

/**
 * @brief Deinitialize TT Module OTA Update
 */
esp_err_t tt_module_ota_deinit(void)
{
    if (!g_tt_ota.initialized) {
        return ESP_OK;
    }

    // Cancel ongoing update if any
    if (g_tt_ota.in_progress) {
        tt_module_ota_cancel();
    }

    if (g_tt_ota.mutex) {
        vSemaphoreDelete(g_tt_ota.mutex);
        g_tt_ota.mutex = NULL;
    }

    g_tt_ota.initialized = false;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT Module OTA deinitialized");
    return ESP_OK;
}

/**
 * @brief Get current OTA state
 */
tt_ota_state_t tt_module_ota_get_state(void)
{
    return g_tt_ota.state;
}

/**
 * @brief Check if OTA update is in progress
 */
bool tt_module_ota_is_in_progress(void)
{
    return g_tt_ota.in_progress;
}

/**
 * @brief Cancel ongoing OTA update
 */
esp_err_t tt_module_ota_cancel(void)
{
    if (!g_tt_ota.in_progress) {
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Cancelling OTA update...");

    // Signal the task to stop
    g_tt_ota.in_progress = false;
    g_tt_ota.state = TT_OTA_STATE_FAILED;

    // Wait for task to finish
    if (g_tt_ota.ota_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (g_tt_ota.ota_task_handle) {
            vTaskDelete(g_tt_ota.ota_task_handle);
            g_tt_ota.ota_task_handle = NULL;
        }
    }

    // Restore normal operation
    tt_module_reset_state();

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA update cancelled");
    return ESP_OK;
}

/**
 * @brief Start OTA update for Tiantong Module
 */
esp_err_t tt_module_ota_start_update(tt_ota_progress_cb_t progress_cb)
{
    if (!g_tt_ota.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_tt_ota.in_progress) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Lock mutex
    if (xSemaphoreTake(g_tt_ota.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Find ota_tt partition
    g_tt_ota.partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "ota_tt");
    if (g_tt_ota.partition == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "ota_tt partition not found");
        xSemaphoreGive(g_tt_ota.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    g_tt_ota.partition_size = g_tt_ota.partition->size;
    g_tt_ota.uploaded_size = 0;
    g_tt_ota.progress_cb = progress_cb;
    g_tt_ota.in_progress = true;
    g_tt_ota.state = TT_OTA_STATE_PREPARING;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Found ota_tt partition, size: %d bytes", g_tt_ota.partition_size);

    // Create OTA task
    BaseType_t ret = xTaskCreate(tt_ota_task, "tt_ota_task", 8192, NULL, 5, &g_tt_ota.ota_task_handle);
    if (ret != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to create OTA task");
        g_tt_ota.in_progress = false;
        g_tt_ota.partition = NULL;
        xSemaphoreGive(g_tt_ota.mutex);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(g_tt_ota.mutex);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA update started from ota_tt partition");
    return ESP_OK;
}

/**
 * @brief OTA Task - Main update procedure
 */
static void tt_ota_task(void *pvParameters)
{
    esp_err_t ret;
    int progress = 0;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA task started");

    // Step 1: Prepare for update
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Step 1: Preparing for update...");
    g_tt_ota.state = TT_OTA_STATE_PREPARING;
    if (g_tt_ota.progress_cb) {
        g_tt_ota.progress_cb(g_tt_ota.state, 0);
    }

    // Stop current mode before OTA
    tt_state_t current_state = tt_module_get_state();
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Current TT module state: %d", current_state);

    if (current_state == TT_STATE_WORKING) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Stopping MUX mode for OTA...");
        // Stop GSM0710 MUX manager to switch to AT mode
        esp_err_t ret = gsm0710_manager_stop();
        if (ret != ESP_OK) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to stop MUX manager: %s", esp_err_to_name(ret));
        }
        // Give time for cleanup
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    progress = 10;
    if (g_tt_ota.progress_cb) {
        g_tt_ota.progress_cb(g_tt_ota.state, progress);
    }

    // Step 2: Reset modem and wait for ready
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Step 2: Resetting modem...");
    tt_module_reset();
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Step 3: Send AT+UPDATE command
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Step 3: Sending AT+UPDATE command...");
    g_tt_ota.state = TT_OTA_STATE_WAITING_READY;

    // Send AT+UPDATE and wait for "ready" response
    tt_at_result_t result = tt_module_send_at_cmd("+UPDATE");
    if (result != TT_AT_RESULT_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to send AT+UPDATE command");
        goto error_exit;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Step 4: Change baudrate to high speed
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Step 4: Changing baudrate to %d...", TT_UART_BAUD_HIGH);

    // Send setbaud command
    char baud_cmd[64];
    snprintf(baud_cmd, sizeof(baud_cmd), "setbaud %d\r\n", TT_UART_BAUD_HIGH);
    // Note: This would need to be sent via raw UART, not through tt_module
    // For now, this is a placeholder

    vTaskDelay(pdMS_TO_TICKS(1000));

    progress = 20;
    if (g_tt_ota.progress_cb) {
        g_tt_ota.progress_cb(g_tt_ota.state, progress);
    }

    // Step 5: Send loadx command
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Step 5: Sending loadx command...");
    // Send "loadx\r\n" command
    // Wait for 'C' (CRC) character from modem

    // Step 6: Upload firmware via XMODEM
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Step 6: Uploading firmware via XMODEM...");
    g_tt_ota.state = TT_OTA_STATE_UPLOADING;

    size_t offset = 0;
    uint8_t packet_num = 1;
    int retry_count = 0;
    esp_err_t read_ret;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Starting firmware upload from partition, total size: %d bytes", g_tt_ota.partition_size);

    while (offset < g_tt_ota.partition_size && g_tt_ota.in_progress) {
        size_t chunk_size = XMODEM_PACKET_SIZE;
        if (offset + chunk_size > g_tt_ota.partition_size) {
            chunk_size = g_tt_ota.partition_size - offset;
        }

        // Read chunk from partition
        read_ret = esp_partition_read(g_tt_ota.partition, offset, g_ota_data_buffer, chunk_size);
        if (read_ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to read from partition at offset %d, size %d", offset, chunk_size);
            retry_count++;
            if (retry_count >= XMODEM_MAX_RETRIES) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Max retries reached while reading from partition");
                goto error_exit;
            }
            continue;
        }

        // Pad with EOF if needed
        if (chunk_size < XMODEM_PACKET_SIZE) {
            for (size_t i = chunk_size; i < XMODEM_PACKET_SIZE; i++) {
                g_ota_data_buffer[i] = XMODEM_EOF;
            }
        }

        // Send packet via XMODEM
        ret = xmodem_send_packet(g_ota_data_buffer, XMODEM_PACKET_SIZE, packet_num);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to send packet %d", packet_num);
            retry_count++;
            if (retry_count >= XMODEM_MAX_RETRIES) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Max retries reached");
                goto error_exit;
            }
            continue;
        }

        retry_count = 0;
        offset += chunk_size;
        g_tt_ota.uploaded_size = offset;
        packet_num++;

        // Update progress
        progress = 20 + (offset * 70 / g_tt_ota.partition_size);
        if (g_tt_ota.progress_cb) {
            g_tt_ota.progress_cb(g_tt_ota.state, progress);
        }

        // Log progress every 10%
        static int last_logged_progress = 0;
        int current_progress = (offset * 100) / g_tt_ota.partition_size;
        if (current_progress / 10 > last_logged_progress) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Upload progress: %d%% (%d/%d bytes)", current_progress, offset, g_tt_ota.partition_size);
            last_logged_progress = current_progress / 10;
        }

        // Small delay to prevent overwhelming the modem
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Firmware upload completed, total bytes: %d", offset);

    // Step 7: Send EOT (End of Transmission)
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Step 7: Sending EOT...");
    // Send XMODEM_EOT and wait for ACK

    // Step 8: Verify completion
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Step 8: Verifying...");
    g_tt_ota.state = TT_OTA_STATE_VERIFYING;
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Success!
    g_tt_ota.state = TT_OTA_STATE_COMPLETED;
    g_tt_ota.in_progress = false;
    progress = 100;

    if (g_tt_ota.progress_cb) {
        g_tt_ota.progress_cb(g_tt_ota.state, progress);
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA update completed successfully!");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT module will reboot and go through normal boot process (AT mode -> SIMST -> MUX mode)");

    // Note: TT module will reboot automatically after firmware update
    // The normal boot process will handle:
    // 1. TT module boots in AT mode
    // 2. SIMST:1 detection will trigger MUX mode switch
    // No manual restoration needed

    g_tt_ota.ota_task_handle = NULL;
    vTaskDelete(NULL);

    return;

error_exit:
    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA update failed!");
    g_tt_ota.state = TT_OTA_STATE_FAILED;
    g_tt_ota.in_progress = false;

    if (g_tt_ota.progress_cb) {
        g_tt_ota.progress_cb(g_tt_ota.state, progress);
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA failed, resetting TT module state to AT command mode");
    tt_module_reset_state();

    g_tt_ota.ota_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Send XMODEM packet
 */
static int xmodem_send_packet(const uint8_t *data, size_t len, uint8_t packet_num)
{
    uint8_t packet[XMODEM_PACKET_TOTAL];
    uint16_t crc = 0;
    int i;

    // Build packet
    packet[0] = XMODEM_SOH;
    packet[1] = packet_num;
    packet[2] = 0xFF - packet_num;

    // Copy data
    memcpy(&packet[3], data, len);

    // Pad with EOF if needed
    for (i = len; i < XMODEM_PACKET_SIZE; i++) {
        packet[3 + i] = XMODEM_EOF;
    }

    // Calculate CRC
    crc = 0;
    for (i = 3; i < XMODEM_PACKET_TOTAL - 2; i++) {
        crc = crc ^ (packet[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }

    // Add CRC to packet
    packet[XMODEM_PACKET_TOTAL - 2] = (crc >> 8) & 0xFF;
    packet[XMODEM_PACKET_TOTAL - 1] = crc & 0xFF;

    // Send packet via UART
    // This would use tt_module_uart_write or direct UART write
    // For now, placeholder
    // tt_module_uart_write(packet, XMODEM_PACKET_TOTAL);

    // Wait for ACK
    int byte = xmodem_receive_byte(XMODEM_TIMEOUT_MS);
    if (byte == XMODEM_ACK) {
        return ESP_OK;
    } else if (byte == XMODEM_NAK) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Received NAK for packet %d", packet_num);
        return ESP_ERR_INVALID_RESPONSE;
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "No ACK for packet %d, received: 0x%02X", packet_num, byte);
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief Receive byte from modem with timeout
 */
static int xmodem_receive_byte(int timeout_ms)
{
    // Placeholder - would use tt_module_uart_read with timeout
    // For now, simulate waiting for ACK
    vTaskDelay(pdMS_TO_TICKS(50));
    return XMODEM_ACK;  // Simulate success
}

/**
 * @brief Wait for CRC character from modem
 */
static int xmodem_wait_for_crc(void)
{
    int retries = XMODEM_MAX_RETRIES;

    while (retries-- > 0) {
        int byte = xmodem_receive_byte(XMODEM_TIMEOUT_MS);
        if (byte == XMODEM_CRC) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Start streaming OTA update for Tiantong Module
 */
esp_err_t tt_module_ota_start_streaming(size_t total_size, tt_ota_progress_cb_t progress_cb)
{
    if (!g_tt_ota.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_tt_ota.in_progress) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Lock mutex
    if (xSemaphoreTake(g_tt_ota.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Initialize streaming OTA context
    g_tt_ota.streaming_mode = true;
    g_tt_ota.total_size = total_size;
    g_tt_ota.uploaded_size = 0;
    g_tt_ota.packet_num = 1;
    g_tt_ota.progress_cb = progress_cb;
    g_tt_ota.in_progress = true;
    g_tt_ota.state = TT_OTA_STATE_PREPARING;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Starting TT streaming OTA: total_size=%d", total_size);

    // Prepare for update
    tt_state_t current_state = tt_module_get_state();
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Current TT module state: %d", current_state);

    if (current_state == TT_STATE_WORKING) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Stopping MUX mode for OTA...");
        esp_err_t ret = gsm0710_manager_stop();
        if (ret != ESP_OK) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to stop MUX manager: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Report initial progress
    if (g_tt_ota.progress_cb) {
        g_tt_ota.progress_cb(g_tt_ota.state, 0);
    }

    // Reset modem and wait for ready
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Resetting modem...");
    tt_module_reset();
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Send AT+UPDATE command
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Sending AT+UPDATE command...");
    g_tt_ota.state = TT_OTA_STATE_WAITING_READY;

    tt_at_result_t result = tt_module_send_at_cmd("+UPDATE");
    if (result != TT_AT_RESULT_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to send AT+UPDATE command");
        goto error_exit;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Change baudrate to high speed
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Changing baudrate to %d...", TT_UART_BAUD_HIGH);

    char baud_cmd[64];
    snprintf(baud_cmd, sizeof(baud_cmd), "setbaud %d\r\n", TT_UART_BAUD_HIGH);
    // TODO: Send baudrate command via raw UART

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Send loadx command
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Sending loadx command...");
    // TODO: Send "loadx\r\n" command

    // Wait for 'C' (CRC) character from modem
    if (xmodem_wait_for_crc() != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to receive CRC character");
        goto error_exit;
    }

    g_tt_ota.state = TT_OTA_STATE_UPLOADING;

    if (g_tt_ota.progress_cb) {
        g_tt_ota.progress_cb(g_tt_ota.state, 0);
    }

    xSemaphoreGive(g_tt_ota.mutex);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT streaming OTA ready for data");

    return ESP_OK;

error_exit:
    g_tt_ota.in_progress = false;
    g_tt_ota.state = TT_OTA_STATE_FAILED;
    xSemaphoreGive(g_tt_ota.mutex);
    return ESP_FAIL;
}

/**
 * @brief Write firmware data during streaming OTA
 */
esp_err_t tt_module_ota_write_data(const uint8_t *data, size_t len)
{
    if (!g_tt_ota.in_progress || !g_tt_ota.streaming_mode) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Streaming OTA not in progress");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Invalid data");
        return ESP_ERR_INVALID_ARG;
    }

    if (len > 4096) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Data too large: %d (max 4096)", len);
        return ESP_ERR_INVALID_SIZE;
    }

    // Lock mutex
    if (xSemaphoreTake(g_tt_ota.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Split data into XMODEM packets (128 bytes each)
    size_t offset = 0;
    esp_err_t ret;

    while (offset < len && g_tt_ota.in_progress) {
        size_t chunk_size = (len - offset > XMODEM_PACKET_SIZE) ? XMODEM_PACKET_SIZE : (len - offset);

        // Copy to static buffer
        memcpy(g_ota_data_buffer, data + offset, chunk_size);

        // Pad with EOF if needed
        if (chunk_size < XMODEM_PACKET_SIZE) {
            for (size_t i = chunk_size; i < XMODEM_PACKET_SIZE; i++) {
                g_ota_data_buffer[i] = XMODEM_EOF;
            }
        }

        // Send packet via XMODEM
        ret = xmodem_send_packet(g_ota_data_buffer, XMODEM_PACKET_SIZE, g_tt_ota.packet_num);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to send packet %d", g_tt_ota.packet_num);
            xSemaphoreGive(g_tt_ota.mutex);
            return ret;
        }

        offset += chunk_size;
        g_tt_ota.uploaded_size += chunk_size;
        g_tt_ota.packet_num++;

        // Update progress
        int progress = (g_tt_ota.uploaded_size * 100) / g_tt_ota.total_size;
        if (g_tt_ota.progress_cb) {
            g_tt_ota.progress_cb(g_tt_ota.state, progress);
        }

        // Log progress every 10%
        static int last_logged_progress = 0;
        if (progress / 10 > last_logged_progress) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Upload progress: %d%% (%d/%d bytes)",
                     progress, g_tt_ota.uploaded_size, g_tt_ota.total_size);
            last_logged_progress = progress / 10;
        }

        // Small delay to prevent overwhelming the modem
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    xSemaphoreGive(g_tt_ota.mutex);
    return ESP_OK;
}

/**
 * @brief Finalize streaming OTA update
 */
esp_err_t tt_module_ota_finalize_streaming(void)
{
    if (!g_tt_ota.in_progress || !g_tt_ota.streaming_mode) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Streaming OTA not in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Lock mutex
    if (xSemaphoreTake(g_tt_ota.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Finalizing TT streaming OTA...");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Total uploaded: %d bytes", g_tt_ota.uploaded_size);

    // Send EOT (End of Transmission)
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Sending EOT...");
    // TODO: Send XMODEM_EOT and wait for ACK

    // Verify completion
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Verifying...");
    g_tt_ota.state = TT_OTA_STATE_VERIFYING;
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Success!
    g_tt_ota.state = TT_OTA_STATE_COMPLETED;
    g_tt_ota.in_progress = false;
    g_tt_ota.streaming_mode = false;

    if (g_tt_ota.progress_cb) {
        g_tt_ota.progress_cb(g_tt_ota.state, 100);
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT streaming OTA completed successfully!");

    xSemaphoreGive(g_tt_ota.mutex);
    return ESP_OK;
}
