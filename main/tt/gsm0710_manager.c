/*
 * gsm0710_manager.c - GSM0710 MUX Manager for ESP-IDF
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "tt/gsm0710_manager.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "tt/tt_module.h"
#include "tt/tt_hardware.h"
#include "syslog.h"

/* External variable for BLE connection subscriptions */
/* Note: conn_handle_subs is not used in LED_TEST (no BLE) */

/* Tag for logging */
static const char *TAG = "GSM0710_MANAGER";

/* Global GSM0710 Manager Context */
static gsm0710_manager_t g_gsm_mgr = {
    .gsm_ctx = NULL,
    .rx_task_handle = NULL,
    .tx_task_handle = NULL,
    .reconnect_task_handle = NULL,
    .heartbeat_task_handle = NULL,
    .mutex = NULL,
    .uart_num = UART_TT_PORT_NUM,
    .initialized = false,
    .started = false,
    .uart_ringbuf = {
        .buffer = {0},
        .head = 0,
        .tail = 0,
        .size = GSM0710_RINGBUF_SIZE,
        .used = 0
    },
    .ringbuf_mutex = NULL,
    .data_ready_sem = NULL,
    .reconnect_state = GSM0710_RECONNECT_STATE_IDLE,
    .reconnect_attempts = 0,
    .reconnect_sem = NULL,
    .heartbeat_waiting = false,
    .heartbeat_failures = 0,
    .heartbeat_sem = NULL,
    // Debug counters
    .read_cb_count = 0,
    .write_cb_count = 0,
    .deliver_data_cb_count = 0,
    .deliver_data_ch1_count = 0
};

/*
 * Static UART RX buffer - avoids repeated malloc/free
 *
 * Buffer size reasoning:
 * - MUX frame max size: 1600 bytes (configured in tt_mux_init_task)
 * - Largest observed frames: ~800 bytes during voice data transfer
 * - Buffer needs to handle at least 1 full frame + margin
 * - 1024 provides 20% headroom for worst-case fragmentation
 * History: Increased from 512 to 1024 due to cross-boundary frames issue
 */
#define GSM0710_UART_RX_BUF_SIZE 1024
static uint8_t g_gsm0710_uart_rx_buf[GSM0710_UART_RX_BUF_SIZE];

/* AT channel data receive callback */
/* Note: These are set once during init and read by callbacks, volatile for thread safety */
static volatile gsm0710_data_received_cb_t g_at_channel_callback = NULL;
static void * volatile g_at_channel_callback_user_data = NULL;

/* Local AT command response buffer */
static volatile bool g_local_command_waiting = false;
static uint8_t g_local_resp_buf[512];
static volatile size_t g_local_resp_len = 0;
static SemaphoreHandle_t g_local_resp_sem = NULL;

/* Voice data channel (channel 9) callback */
/* Note: These are set once during init and read by callbacks, volatile for thread safety */
static volatile gsm0710_data_received_cb_t g_voice_channel_callback = NULL;
static void * volatile g_voice_channel_callback_user_data = NULL;

/* Forward declarations for ring buffer operations */
static void gsm0710_ringbuf_init(gsm0710_ringbuf_t *ringbuf);
static int gsm0710_ringbuf_write(gsm0710_ringbuf_t *ringbuf, const uint8_t *data, int len);
static int gsm0710_ringbuf_read(gsm0710_ringbuf_t *ringbuf, uint8_t *data, int len);
static void gsm0710_ringbuf_flush(gsm0710_ringbuf_t *ringbuf);
static void gsm0710_ringbuf_discard(gsm0710_ringbuf_t *ringbuf, int bytes_to_discard);

/* Ring Buffer Implementation */
static void gsm0710_ringbuf_init(gsm0710_ringbuf_t *ringbuf)
{
    ringbuf->head = 0;
    ringbuf->tail = 0;
    ringbuf->used = 0;
    ringbuf->size = GSM0710_RINGBUF_SIZE;
    memset(ringbuf->buffer, 0, GSM0710_RINGBUF_SIZE);
}

static int gsm0710_ringbuf_write(gsm0710_ringbuf_t *ringbuf, const uint8_t *data, int len)
{
    if (len <= 0 || ringbuf == NULL || data == NULL) {
        return 0;
    }

    // Calculate available space
    int free_space = ringbuf->size - ringbuf->used;
    if (free_space <= 0) {
        return 0; // No space available
    }

    // Limit to available space
    int write_len = (len <= free_space) ? len : free_space;
    int written = 0;

    // Write data to ring buffer
    while (written < write_len) {
        ringbuf->buffer[ringbuf->head] = data[written];
        ringbuf->head = (ringbuf->head + 1) % ringbuf->size;
        written++;
    }

    // Update used space
    ringbuf->used += written;

    return written;
}

static int gsm0710_ringbuf_read(gsm0710_ringbuf_t *ringbuf, uint8_t *data, int len)
{
    if (len <= 0 || ringbuf == NULL || data == NULL) {
        return 0;
    }

    // Check if there is data available
    if (ringbuf->used == 0) {
        return 0; // No data available
    }

    // Limit to available data
    int read_len = (len <= ringbuf->used) ? len : ringbuf->used;
    int read = 0;

    // Read data from ring buffer
    while (read < read_len) {
        data[read] = ringbuf->buffer[ringbuf->tail];
        ringbuf->tail = (ringbuf->tail + 1) % ringbuf->size;
        read++;
    }

    // Update used space
    ringbuf->used -= read;

    return read;
}

static void gsm0710_ringbuf_flush(gsm0710_ringbuf_t *ringbuf)
{
    if (ringbuf != NULL) {
        ringbuf->head = 0;
        ringbuf->tail = 0;
        ringbuf->used = 0;
    }
}

/**
 * @brief Discard oldest data from ring buffer to make space
 *
 * This function discards a portion of the oldest data from the ring buffer
 * to make room for new data. This is better than full flush as it maintains
 * some data continuity.
 *
 * @param ringbuf Ring buffer pointer
 * @param bytes_to_discard Number of bytes to discard from the oldest data
 */
static void gsm0710_ringbuf_discard(gsm0710_ringbuf_t *ringbuf, int bytes_to_discard)
{
    if (ringbuf == NULL || bytes_to_discard <= 0) {
        return;
    }

    // Limit to available data
    if (bytes_to_discard > ringbuf->used) {
        bytes_to_discard = ringbuf->used;
    }

    // Advance tail pointer (discard oldest data)
    ringbuf->tail = (ringbuf->tail + bytes_to_discard) % ringbuf->size;
    ringbuf->used -= bytes_to_discard;
}



/* Forward declarations for callbacks */
static int gsm0710_read_cb(struct gsm0710_context *ctx, void *data, int len);
static int gsm0710_write_cb(struct gsm0710_context *ctx, const void *data, int len);
static void gsm0710_deliver_data_cb(struct gsm0710_context *ctx, int channel, const void *data, int len);
static void gsm0710_deliver_status_cb(struct gsm0710_context *ctx, int channel, int status);
static void gsm0710_open_channel_cb(struct gsm0710_context *ctx, int channel);
static void gsm0710_close_channel_cb(struct gsm0710_context *ctx, int channel);
static void gsm0710_terminate_cb(struct gsm0710_context *ctx);
static void gsm0710_debug_message_cb(struct gsm0710_context *ctx, const char *msg);
static void gsm0710_lock_cb(struct gsm0710_context *ctx);
static void gsm0710_unlock_cb(struct gsm0710_context *ctx);
static void gsm0710_reconnect_task(void *pvParameters);
static bool gsm0710_attempt_reconnect(void);
static void gsm0710_trigger_reconnect(void);
static void gsm0710_heartbeat_task(void *pvParameters);
static void gsm0710_trigger_heartbeat_recovery(void);



/**
 * @brief UART RX Task for reading data from UART and feeding to GSM0710
 */
static void uart_rx_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 UART RX Task started");

    while (1) {
        // Read data from UART using static buffer
        int len = tt_module_uart_read(g_gsm0710_uart_rx_buf, GSM0710_UART_RX_BUF_SIZE, 100);

        if (len > 0) {
            // Debug log - show UART1 RX data
            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART1 RX: %d bytes", len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, g_gsm0710_uart_rx_buf, len > 64 ? 64 : len, ESP_LOG_DEBUG);

            // Write data to the ring buffer
            if (xSemaphoreTake(g_gsm_mgr.ringbuf_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Check if buffer has enough space for new data
                int free_space = g_gsm_mgr.uart_ringbuf.size - g_gsm_mgr.uart_ringbuf.used;

                // If not enough space, discard oldest data to make room
                // Discard enough to fit new data plus 25% extra headroom
                if (free_space < len) {
                    int bytes_to_discard = len - free_space + (g_gsm_mgr.uart_ringbuf.size / 4);
                    SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                        "Ring buffer full (%d/%d), discarding %d oldest bytes",
                        g_gsm_mgr.uart_ringbuf.used, g_gsm_mgr.uart_ringbuf.size, bytes_to_discard);
                    gsm0710_ringbuf_discard(&g_gsm_mgr.uart_ringbuf, bytes_to_discard);
                }

                int bytes_written = gsm0710_ringbuf_write(&g_gsm_mgr.uart_ringbuf, g_gsm0710_uart_rx_buf, len);
                xSemaphoreGive(g_gsm_mgr.ringbuf_mutex);

                if (bytes_written > 0) {
                    // Immediately consume data by calling gsm0710_ready_read
                    // This is synchronous - gsm0710_read_cb will be called to read from ring buffer
                    if (g_gsm_mgr.gsm_ctx != NULL && g_gsm_mgr.started) {
                        gsm0710_ready_read(g_gsm_mgr.gsm_ctx);
                    }
                } else {
                    // Should not happen after discard, but log if it does
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                        "Ring buffer write failed after discard, %d bytes dropped", len);
                }
            }
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief GSM0710 Read Callback
 * 
 * This function is called by the GSM0710 library to request data. It reads
 * data from the ring buffer that was populated by the UART RX task.
 *
 * IMPORTANT: This callback is called synchronously from gsm0710_ready_read(),
 * so it must NOT block waiting for data. Just read what's available and return.
 */
static int gsm0710_read_cb(struct gsm0710_context *ctx, void *data, int len)
{
    g_gsm_mgr.read_cb_count++;

    // len=0 is valid - GSM0710 library may call with len=0 to probe state
    if (ctx == NULL || data == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "gsm0710_read_cb: invalid params ctx=%p data=%p", ctx, data);
        return 0;
    }

    // len=0 is a valid probe call, just return 0 without logging
    if (len <= 0) {
        return 0;
    }

    int bytes_read = 0;

    // Try to read data from the ring buffer (non-blocking)
    // Use short timeout to avoid blocking the caller
    if (xSemaphoreTake(g_gsm_mgr.ringbuf_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bytes_read = gsm0710_ringbuf_read(&g_gsm_mgr.uart_ringbuf, data, len);
        xSemaphoreGive(g_gsm_mgr.ringbuf_mutex);
    } else {
        // Failed to acquire mutex - log and return 0
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "gsm0710_read_cb: mutex timeout");
        return 0;
    }

    if (bytes_read > 0) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "gsm0710_read_cb [#%lu]: req %d, read %d, remaining %d",
                 g_gsm_mgr.read_cb_count, len, bytes_read, g_gsm_mgr.uart_ringbuf.used);
    }

    return bytes_read;
}

/**
 * @brief GSM0710 Write Callback
 */
static int gsm0710_write_cb(struct gsm0710_context *ctx, const void *data, int len)
{
    g_gsm_mgr.write_cb_count++;

    SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "gsm0710_write_cb [#%lu]: writing %d bytes",
             g_gsm_mgr.write_cb_count, len);

    // Write data to UART using tt_module interface
    int bytes_written = tt_module_uart_write(data, len);

    if (bytes_written != len) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to write all data to UART: %d/%d bytes written", bytes_written, len);
        return -1;
    }

    return bytes_written;
}

/**
 * @brief GSM0710 Deliver Data Callback
 */
static void gsm0710_deliver_data_cb(struct gsm0710_context *ctx, int channel, const void *data, int len)
{
    g_gsm_mgr.deliver_data_cb_count++;

    // Log AT channel data
    if (channel == GSM0710_CHANNEL_AT) {
        g_gsm_mgr.deliver_data_ch1_count++;
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 MUX READ (ch=%d): len=%d, data=%.*s",
                       channel, len, len, (char *)data);
    } else {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 MUX READ (ch=%d): %d bytes", channel, len);
    }

    // For AT channel (channel 1), forward to callback
    // The callback (tt_module_route_at_response) will handle routing based on context:
    // - LOCAL commands: Save to buffer and signal semaphore
    // - GATT commands: Forward to BLE connection
    // - NONE context: Forward as unsolicited notification
    if (channel == GSM0710_CHANNEL_AT) {
        if (g_at_channel_callback != NULL) {
            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> MUX ROUTE: Forwarding %d bytes from AT channel to tt_module", len);
            g_at_channel_callback((const uint8_t *)data, len, g_at_channel_callback_user_data);
        } else {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "<<< MUX ROUTE: No AT channel callback registered, data dropped!");
        }
    }

    // For Voice AT channel (channel 2), check for heartbeat response
    if (channel == GSM0710_CHANNEL_VOICE_AT) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 MUX READ (ch=%d): %.*s", channel, len, (char *)data);

        // Check if this is a heartbeat response
        if (g_gsm_mgr.heartbeat_waiting) {
            // Check for "OK" response
            if (strstr((char *)data, "OK") != NULL) {
                SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT: OK received");
                g_gsm_mgr.heartbeat_waiting = false;
                xSemaphoreGive(g_gsm_mgr.heartbeat_sem);
            } else if (strstr((char *)data, "ERROR") != NULL) {
                // Error response also counts as a response
                SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT: ERROR received");
                g_gsm_mgr.heartbeat_waiting = false;
                xSemaphoreGive(g_gsm_mgr.heartbeat_sem);
            } else {
                // Other response - log it
                SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT: unexpected response: %.*s", len, (char *)data);
            }
        }
        // Note: Channel 2 data is not forwarded to any callback as it's only used for heartbeat
    }

    // For Voice Data channel (channel 9), invoke registered callback
    if (channel == GSM0710_CHANNEL_VOICE_DATA) {
        if (g_voice_channel_callback != NULL) {
            g_voice_channel_callback((const uint8_t *)data, len, g_voice_channel_callback_user_data);
            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Forwarded %d bytes from Voice channel (9) to callback", len);
        }
    }

    // Send data to the corresponding channel queue
    // Skip channels that use callback mechanism (1=AT, 2=VOICE_AT for heartbeat, 9=Voice Data)
    if (channel > 0 && channel <= GSM0710_MAX_CHANNEL &&
        channel != GSM0710_CHANNEL_AT &&
        channel != GSM0710_CHANNEL_VOICE_AT &&
        channel != GSM0710_CHANNEL_VOICE_DATA) {
        if (g_gsm_mgr.data_queues[channel] != NULL) {
            gsm0710_packet_t packet;
            packet.channel = channel;
            packet.len = (len < GSM0710_PACKET_SIZE) ? len : GSM0710_PACKET_SIZE;
            memcpy(packet.data, data, packet.len);

            if (xQueueSend(g_gsm_mgr.data_queues[channel], &packet, pdMS_TO_TICKS(10)) != pdPASS) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to send data to channel %d queue", channel);
            } else {
                SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Delivered %d bytes to channel %d queue", len, channel);
            }
        }
    } else if (channel == GSM0710_CHANNEL_AT || channel == GSM0710_CHANNEL_VOICE_AT || channel == GSM0710_CHANNEL_VOICE_DATA) {
        // Channels using callback mechanism (or heartbeat for ch2) don't need queue
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Channel %d uses callback/heartbeat, skipping queue", channel);
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid channel %d for data delivery", channel);
    }
}

/**
 * @brief GSM0710 Deliver Status Callback
 */
static void gsm0710_deliver_status_cb(struct gsm0710_context *ctx, int channel, int status)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Channel %d status: 0x%02x", channel, status);
    // Handle status changes here if needed
}

/**
 * @brief GSM0710 Open Channel Callback
 */
static void gsm0710_open_channel_cb(struct gsm0710_context *ctx, int channel)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Channel %d opened", channel);
}

/**
 * @brief GSM0710 Close Channel Callback
 */
static void gsm0710_close_channel_cb(struct gsm0710_context *ctx, int channel)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Channel %d closed", channel);
}

/**
 * @brief GSM0710 Terminate Callback
 */
static void gsm0710_terminate_cb(struct gsm0710_context *ctx)
{
    SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 session terminated");
    // Trigger reconnection if MUX was started
    if (g_gsm_mgr.started) {
        g_gsm_mgr.started = false;
        gsm0710_trigger_reconnect();
    }
}

/**
 * @brief GSM0710 Debug Message Callback
 */
static void gsm0710_debug_message_cb(struct gsm0710_context *ctx, const char *msg)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710: %s", msg);
}

/**
 * @brief GSM0710 Lock Callback (protects frame_buffer)
 */
static void gsm0710_lock_cb(struct gsm0710_context *ctx)
{
    (void)ctx;
    if (g_gsm_mgr.mutex != NULL) {
        xSemaphoreTakeRecursive(g_gsm_mgr.mutex, portMAX_DELAY);
    }
}

/**
 * @brief GSM0710 Unlock Callback
 */
static void gsm0710_unlock_cb(struct gsm0710_context *ctx)
{
    (void)ctx;
    if (g_gsm_mgr.mutex != NULL) {
        xSemaphoreGiveRecursive(g_gsm_mgr.mutex);
    }
}

/**
 * @brief GSM0710 Reconnection Task
 * 
 * This task handles automatic reconnection when MUX connection is lost
 */
static void gsm0710_reconnect_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Reconnection Task started");

    while (1) {
        // Wait for reconnection trigger
        if (xSemaphoreTake(g_gsm_mgr.reconnect_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX reconnection triggered");

        // Set reconnection state
        g_gsm_mgr.reconnect_state = GSM0710_RECONNECT_STATE_RECONNECTING;
        g_gsm_mgr.reconnect_attempts = 0;

        // Attempt reconnection
        bool reconnected = false;
        while (g_gsm_mgr.reconnect_attempts < GSM0710_RECONNECT_MAX_ATTEMPTS) {
            g_gsm_mgr.reconnect_attempts++;
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Reconnection attempt %d/%d", 
                     g_gsm_mgr.reconnect_attempts, GSM0710_RECONNECT_MAX_ATTEMPTS);

            // Try to reconnect
            if (gsm0710_attempt_reconnect()) {
                reconnected = true;
                break;
            }

            // Wait before next attempt
            vTaskDelay(pdMS_TO_TICKS(GSM0710_RECONNECT_INTERVAL_MS));
        }

        if (reconnected) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX reconnected successfully");
            g_gsm_mgr.reconnect_state = GSM0710_RECONNECT_STATE_IDLE;
        } else {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX reconnection failed after %d attempts", 
                     GSM0710_RECONNECT_MAX_ATTEMPTS);
            g_gsm_mgr.reconnect_state = GSM0710_RECONNECT_STATE_FAILED;
            
            // Reset TT module state and restart AT task
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Resetting to AT command mode due to MUX failure");
            
            // Update TT module state to AT command mode
            tt_module_reset_state();
            
            // Restart AT command task
            tt_module_restart_at_task();
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief Attempt a single MUX reconnection
 * 
 * @return bool True if reconnection succeeded, false otherwise
 */
static bool gsm0710_attempt_reconnect(void)
{
    esp_err_t ret = ESP_OK;

    // Check if TT module is initialized
    if (!tt_module_uart_is_initialized()) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT module not initialized, cannot reconnect");
        return false;
    }

    // Stop and deinitialize current MUX session
    gsm0710_manager_stop();
    gsm0710_context_free(g_gsm_mgr.gsm_ctx);
    g_gsm_mgr.gsm_ctx = NULL;

    // Recreate GSM0710 context
    g_gsm_mgr.gsm_ctx = gsm0710_context_new();
    if (g_gsm_mgr.gsm_ctx == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create GSM0710 context");
        return false;
    }

    // Set callbacks
    g_gsm_mgr.gsm_ctx->read = gsm0710_read_cb;
    g_gsm_mgr.gsm_ctx->write = gsm0710_write_cb;
    g_gsm_mgr.gsm_ctx->deliver_data = gsm0710_deliver_data_cb;
    g_gsm_mgr.gsm_ctx->deliver_status = gsm0710_deliver_status_cb;
    g_gsm_mgr.gsm_ctx->open_channel = gsm0710_open_channel_cb;
    g_gsm_mgr.gsm_ctx->close_channel = gsm0710_close_channel_cb;
    g_gsm_mgr.gsm_ctx->terminate = gsm0710_terminate_cb;
    g_gsm_mgr.gsm_ctx->debug_message = gsm0710_debug_message_cb;
    g_gsm_mgr.gsm_ctx->lock = gsm0710_lock_cb;
    g_gsm_mgr.gsm_ctx->unlock = gsm0710_unlock_cb;
    g_gsm_mgr.gsm_ctx->user_data = &g_gsm_mgr;

    // Restart MUX session
    ret = gsm0710_manager_start();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to restart MUX session: %s", esp_err_to_name(ret));
        gsm0710_context_free(g_gsm_mgr.gsm_ctx);
        g_gsm_mgr.gsm_ctx = NULL;
        return false;
    }

    // Reopen channels
    int channels[] = {
        GSM0710_CHANNEL_MUX_CONTROL,
        GSM0710_CHANNEL_AT,
        GSM0710_CHANNEL_VOICE_AT,
        GSM0710_CHANNEL_SMS,
        GSM0710_CHANNEL_DATA_AT,
        GSM0710_CHANNEL_VOICE_DATA,
        GSM0710_CHANNEL_IOT
    };
    
    for (int i = 0; i < sizeof(channels)/sizeof(channels[0]); i++) {
        ret = gsm0710_manager_open_channel(channels[i]);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to reopen channel %d: %s", channels[i], esp_err_to_name(ret));
            // Continue opening other channels
        }
    }

    return true;
}

/**
 * @brief Trigger MUX reconnection
 */
static void gsm0710_trigger_reconnect(void)
{
    // Only trigger if not already reconnecting
    if (g_gsm_mgr.reconnect_state != GSM0710_RECONNECT_STATE_RECONNECTING) {
        xSemaphoreGive(g_gsm_mgr.reconnect_sem);
    }
}

/**
 * @brief Heartbeat Task for monitoring MUX link health
 *
 * This task sends periodic AT commands via channel 2 (VOICE_AT) to check if
 * the TT module is still responsive. If consecutive heartbeats fail, it triggers
 * a recovery procedure.
 */
static void gsm0710_heartbeat_task(void *pvParameters)
{
    const char *heartbeat_cmd = "AT\r\n";
    int heartbeat_count = 0;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Heartbeat Task started (interval=%d ms, timeout=%d ms, max_failures=%d)",
             GSM0710_HEARTBEAT_INTERVAL_MS, GSM0710_HEARTBEAT_TIMEOUT_MS, GSM0710_HEARTBEAT_MAX_FAILURES);

    while (1) {
        // Wait for heartbeat interval
        vTaskDelay(pdMS_TO_TICKS(GSM0710_HEARTBEAT_INTERVAL_MS));

        heartbeat_count++;
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT: Tick #%d, started=%d",
                 heartbeat_count, g_gsm_mgr.started);

        // Only send heartbeat if MUX is started
        if (!g_gsm_mgr.started) {
            continue;
        }

        // Check TT module state - only send heartbeat when module is WORKING
        tt_state_t tt_state = tt_module_get_state();
        if (tt_state != TT_STATE_WORKING) {
            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                           "HEARTBEAT: TT module not in WORKING state (state=%d), skipping",
                           tt_state);
            continue;
        }

        // Send heartbeat via channel 2 (VOICE_AT)
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX AT heartbeat #%d sending...", heartbeat_count);

        // Check and wakeup BP before sending heartbeat
        if (tt_hw_is_bp_sleeping()) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT: BP is sleeping, waking up...");
            esp_err_t wakeup_ret = tt_hw_wakeup_bp(500);  // Wait up to 500ms for BP wakeup
            if (wakeup_ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT: Failed to wakeup BP: %s", esp_err_to_name(wakeup_ret));
                g_gsm_mgr.heartbeat_failures++;
                goto check_failures;
            }
            // Add small delay after wakeup to ensure BP is ready
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        g_gsm_mgr.heartbeat_waiting = true;

        esp_err_t ret = gsm0710_manager_send(GSM0710_CHANNEL_VOICE_AT,
                                              (uint8_t *)heartbeat_cmd,
                                              strlen(heartbeat_cmd));
        if (ret != ESP_OK) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT: send failed: %s", esp_err_to_name(ret));
            g_gsm_mgr.heartbeat_waiting = false;
            g_gsm_mgr.heartbeat_failures++;
            goto check_failures;
        }

        // Wait for response with timeout
        if (xSemaphoreTake(g_gsm_mgr.heartbeat_sem, pdMS_TO_TICKS(GSM0710_HEARTBEAT_TIMEOUT_MS)) == pdTRUE) {
            // Response received
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX AT heartbeat #%d OK", heartbeat_count);
            g_gsm_mgr.heartbeat_waiting = false;
            g_gsm_mgr.heartbeat_failures = 0;  // Reset failure counter
        } else {
            // Timeout - no response
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT #%d: timeout (failures=%d/%d)",
                     heartbeat_count, g_gsm_mgr.heartbeat_failures + 1, GSM0710_HEARTBEAT_MAX_FAILURES);
            g_gsm_mgr.heartbeat_waiting = false;
            g_gsm_mgr.heartbeat_failures++;
        }

check_failures:
        // Check if we've exceeded max failures
        if (g_gsm_mgr.heartbeat_failures >= GSM0710_HEARTBEAT_MAX_FAILURES) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "HEARTBEAT: %d consecutive failures, triggering recovery",
                     g_gsm_mgr.heartbeat_failures);
            g_gsm_mgr.heartbeat_failures = 0;
            gsm0710_trigger_heartbeat_recovery();
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief Trigger heartbeat recovery procedure
 *
 * This function is called when consecutive heartbeat failures indicate that
 * the TT module is unresponsive. It stops MUX mode and resets the module.
 */
static void gsm0710_trigger_heartbeat_recovery(void)
{
    SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "=== HEARTBEAT RECOVERY TRIGGERED ===");

    // Stop MUX session
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Stopping MUX session...");
    gsm0710_manager_stop();

    // Clear MUX state
    if (g_gsm_mgr.gsm_ctx != NULL) {
        gsm0710_context_free(g_gsm_mgr.gsm_ctx);
        g_gsm_mgr.gsm_ctx = NULL;
    }

    // Reset TT module to AT command mode
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Resetting TT module to AT command mode...");
    tt_module_reset_state();

    // Restart AT command task
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Restarting AT command task...");
    tt_module_restart_at_task();

    // Hardware reset TT module for full recovery
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Performing hardware reset of TT module...");
    tt_module_reset();

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "=== HEARTBEAT RECOVERY COMPLETE ===");
}

/**
 * @brief Initialize GSM0710 Manager
 */
esp_err_t gsm0710_manager_init(void)
{
    esp_err_t ret = ESP_OK;

    if (g_gsm_mgr.initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager already initialized");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Initializing GSM0710 Manager");

    // Create recursive mutex for protecting shared resources
    // Use recursive mutex to prevent deadlock when same task acquires lock multiple times
    g_gsm_mgr.mutex = xSemaphoreCreateRecursiveMutex();
    if (g_gsm_mgr.mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create recursive mutex");
        return ESP_FAIL;
    }

    // Create semaphore for reconnection trigger
    g_gsm_mgr.reconnect_sem = xSemaphoreCreateBinary();
    if (g_gsm_mgr.reconnect_sem == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create reconnection semaphore");
        return ESP_FAIL;
    }

    // Create ring buffer mutex
    g_gsm_mgr.ringbuf_mutex = xSemaphoreCreateMutex();
    if (g_gsm_mgr.ringbuf_mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create ring buffer mutex");
        return ESP_FAIL;
    }

    // Create data ready semaphore
    g_gsm_mgr.data_ready_sem = xSemaphoreCreateBinary();
    if (g_gsm_mgr.data_ready_sem == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create data ready semaphore");
        return ESP_FAIL;
    }

    // Create heartbeat response semaphore
    g_gsm_mgr.heartbeat_sem = xSemaphoreCreateBinary();
    if (g_gsm_mgr.heartbeat_sem == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create heartbeat semaphore");
        return ESP_FAIL;
    }

    // Create local command response semaphore
    g_local_resp_sem = xSemaphoreCreateBinary();
    if (g_local_resp_sem == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create local response semaphore");
        return ESP_FAIL;
    }

    // Initialize ring buffer
    gsm0710_ringbuf_init(&g_gsm_mgr.uart_ringbuf);

    // Create data queues for each channel
    for (int i = 0; i <= GSM0710_MAX_CHANNEL; i++) {
        g_gsm_mgr.data_queues[i] = xQueueCreate(GSM0710_QUEUE_SIZE, sizeof(gsm0710_packet_t));
        if (g_gsm_mgr.data_queues[i] == NULL) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create queue for channel %d", i);
            ret = ESP_FAIL;
            goto cleanup;
        }
    }

    // Create GSM0710 context
    g_gsm_mgr.gsm_ctx = gsm0710_context_new();
    if (g_gsm_mgr.gsm_ctx == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create GSM0710 context");
        ret = ESP_FAIL;
        goto cleanup;
    }

    // Set callbacks
    g_gsm_mgr.gsm_ctx->read = gsm0710_read_cb;
    g_gsm_mgr.gsm_ctx->write = gsm0710_write_cb;
    g_gsm_mgr.gsm_ctx->deliver_data = gsm0710_deliver_data_cb;
    g_gsm_mgr.gsm_ctx->deliver_status = gsm0710_deliver_status_cb;
    g_gsm_mgr.gsm_ctx->open_channel = gsm0710_open_channel_cb;
    g_gsm_mgr.gsm_ctx->close_channel = gsm0710_close_channel_cb;
    g_gsm_mgr.gsm_ctx->terminate = gsm0710_terminate_cb;
    g_gsm_mgr.gsm_ctx->debug_message = gsm0710_debug_message_cb;
    g_gsm_mgr.gsm_ctx->lock = gsm0710_lock_cb;
    g_gsm_mgr.gsm_ctx->unlock = gsm0710_unlock_cb;
    g_gsm_mgr.gsm_ctx->user_data = &g_gsm_mgr;

    // Create UART RX task (4KB stack is sufficient now that frame_buffer is not on stack)
    if (xTaskCreate(uart_rx_task, "uart_rx_task", 4096*2, NULL, 10, &g_gsm_mgr.rx_task_handle) != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create UART RX task");
        ret = ESP_FAIL;
        goto cleanup;
    }

    // Create reconnection task
    if (xTaskCreate(gsm0710_reconnect_task, "gsm0710_reconnect_task", 4096, NULL, 7, &g_gsm_mgr.reconnect_task_handle) != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create reconnection task");
        ret = ESP_FAIL;
        goto cleanup;
    }

    g_gsm_mgr.initialized = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager initialized successfully");

    return ret;

cleanup:
    // Cleanup resources
    if (g_gsm_mgr.mutex != NULL) {
        vSemaphoreDelete(g_gsm_mgr.mutex);
        g_gsm_mgr.mutex = NULL;
    }

    if (g_gsm_mgr.reconnect_sem != NULL) {
        vSemaphoreDelete(g_gsm_mgr.reconnect_sem);
        g_gsm_mgr.reconnect_sem = NULL;
    }

    if (g_gsm_mgr.ringbuf_mutex != NULL) {
        vSemaphoreDelete(g_gsm_mgr.ringbuf_mutex);
        g_gsm_mgr.ringbuf_mutex = NULL;
    }

    if (g_gsm_mgr.data_ready_sem != NULL) {
        vSemaphoreDelete(g_gsm_mgr.data_ready_sem);
        g_gsm_mgr.data_ready_sem = NULL;
    }

    if (g_gsm_mgr.heartbeat_sem != NULL) {
        vSemaphoreDelete(g_gsm_mgr.heartbeat_sem);
        g_gsm_mgr.heartbeat_sem = NULL;
    }

    if (g_local_resp_sem != NULL) {
        vSemaphoreDelete(g_local_resp_sem);
        g_local_resp_sem = NULL;
    }

    for (int i = 0; i <= GSM0710_MAX_CHANNEL; i++) {
        if (g_gsm_mgr.data_queues[i] != NULL) {
            vQueueDelete(g_gsm_mgr.data_queues[i]);
            g_gsm_mgr.data_queues[i] = NULL;
        }
    }

    if (g_gsm_mgr.gsm_ctx != NULL) {
        gsm0710_context_free(g_gsm_mgr.gsm_ctx);
        g_gsm_mgr.gsm_ctx = NULL;
    }

    if (g_gsm_mgr.reconnect_task_handle != NULL) {
        vTaskDelete(g_gsm_mgr.reconnect_task_handle);
        g_gsm_mgr.reconnect_task_handle = NULL;
    }

    g_gsm_mgr.initialized = false;

    return ret;
}

/**
 * @brief Deinitialize GSM0710 Manager
 */
esp_err_t gsm0710_manager_deinit(void)
{
    if (!g_gsm_mgr.initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager not initialized");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Deinitializing GSM0710 Manager");

    // Stop GSM0710 session
    gsm0710_manager_stop();

    // Delete UART RX task
    if (g_gsm_mgr.rx_task_handle != NULL) {
        vTaskDelete(g_gsm_mgr.rx_task_handle);
        g_gsm_mgr.rx_task_handle = NULL;
    }

    // Delete reconnection task
    if (g_gsm_mgr.reconnect_task_handle != NULL) {
        vTaskDelete(g_gsm_mgr.reconnect_task_handle);
        g_gsm_mgr.reconnect_task_handle = NULL;
    }

    // Delete data queues
    for (int i = 0; i <= GSM0710_MAX_CHANNEL; i++) {
        if (g_gsm_mgr.data_queues[i] != NULL) {
            vQueueDelete(g_gsm_mgr.data_queues[i]);
            g_gsm_mgr.data_queues[i] = NULL;
        }
    }

    // Free GSM0710 context
    if (g_gsm_mgr.gsm_ctx != NULL) {
        gsm0710_context_free(g_gsm_mgr.gsm_ctx);
        g_gsm_mgr.gsm_ctx = NULL;
    }

    // Delete mutex
    if (g_gsm_mgr.mutex != NULL) {
        vSemaphoreDelete(g_gsm_mgr.mutex);
        g_gsm_mgr.mutex = NULL;
    }

    // Delete reconnection semaphore
    if (g_gsm_mgr.reconnect_sem != NULL) {
        vSemaphoreDelete(g_gsm_mgr.reconnect_sem);
        g_gsm_mgr.reconnect_sem = NULL;
    }

    // Delete heartbeat semaphore
    if (g_gsm_mgr.heartbeat_sem != NULL) {
        vSemaphoreDelete(g_gsm_mgr.heartbeat_sem);
        g_gsm_mgr.heartbeat_sem = NULL;
    }

    // Delete local command response semaphore
    if (g_local_resp_sem != NULL) {
        vSemaphoreDelete(g_local_resp_sem);
        g_local_resp_sem = NULL;
    }

    // Delete ring buffer resources
    if (g_gsm_mgr.ringbuf_mutex != NULL) {
        vSemaphoreDelete(g_gsm_mgr.ringbuf_mutex);
        g_gsm_mgr.ringbuf_mutex = NULL;
    }

    if (g_gsm_mgr.data_ready_sem != NULL) {
        vSemaphoreDelete(g_gsm_mgr.data_ready_sem);
        g_gsm_mgr.data_ready_sem = NULL;
    }

    // Flush the ring buffer
    gsm0710_ringbuf_flush(&g_gsm_mgr.uart_ringbuf);

    g_gsm_mgr.initialized = false;
    g_gsm_mgr.started = false;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager deinitialized successfully");

    return ESP_OK;
}

/**
 * @brief Start GSM0710 MUX session
 */
esp_err_t gsm0710_manager_start(void)
{
    if (!g_gsm_mgr.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_gsm_mgr.gsm_ctx == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 context not available");
        return ESP_ERR_INVALID_STATE;
    }

    // Check if TT module is initialized
    if (!tt_module_uart_is_initialized()) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT module not initialized, cannot start MUX session");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Starting GSM0710 MUX session");

    // Reset debug counters
    g_gsm_mgr.read_cb_count = 0;
    g_gsm_mgr.write_cb_count = 0;
    g_gsm_mgr.deliver_data_cb_count = 0;
    g_gsm_mgr.deliver_data_ch1_count = 0;

    // Set basic mode and frame size according to requirements
    g_gsm_mgr.gsm_ctx->mode = GSM0710_MODE_BASIC;  // Only basic mode is supported
    g_gsm_mgr.gsm_ctx->frame_size = 1600;  // Frame size as required
    g_gsm_mgr.gsm_ctx->port_speed = 115200;  // Only 115200 baud is supported

    // Initialize GSM0710 session (send AT+CMUX command)
    int ret = gsm0710_startup(g_gsm_mgr.gsm_ctx, 1);
    if (ret != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to start GSM0710 session: %d", ret);
        return ESP_FAIL;
    }

    // Set started flag
    g_gsm_mgr.started = true;

    // Create heartbeat task
    if (g_gsm_mgr.heartbeat_task_handle == NULL) {
        BaseType_t ret = xTaskCreate(gsm0710_heartbeat_task, "gsm0710_heartbeat", 4096, NULL, 6, &g_gsm_mgr.heartbeat_task_handle);
        if (ret != pdPASS) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create heartbeat task");
            // Continue anyway - heartbeat is not critical for basic operation
        } else {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Heartbeat task created (interval=%d ms)", GSM0710_HEARTBEAT_INTERVAL_MS);
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 MUX session started successfully");

    return ESP_OK;
}

/**
 * @brief Stop GSM0710 MUX session
 */
esp_err_t gsm0710_manager_stop(void)
{
    if (!g_gsm_mgr.initialized || g_gsm_mgr.gsm_ctx == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager not initialized or no context");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Stopping GSM0710 MUX session");

    // Stop heartbeat task
    if (g_gsm_mgr.heartbeat_task_handle != NULL) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Deleting heartbeat task...");
        vTaskDelete(g_gsm_mgr.heartbeat_task_handle);
        g_gsm_mgr.heartbeat_task_handle = NULL;
    }

    // Reset heartbeat state
    g_gsm_mgr.heartbeat_waiting = false;
    g_gsm_mgr.heartbeat_failures = 0;

    gsm0710_shutdown(g_gsm_mgr.gsm_ctx);

    // Clear started flag
    g_gsm_mgr.started = false;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 MUX session stopped successfully");

    return ESP_OK;
}

/**
 * @brief Open a GSM0710 channel
 */
esp_err_t gsm0710_manager_open_channel(int channel)
{
    if (!g_gsm_mgr.initialized || g_gsm_mgr.gsm_ctx == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel < 1 || channel > GSM0710_MAX_CHANNEL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid channel number: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Opening channel %d", channel);

    int ret = gsm0710_open_channel(g_gsm_mgr.gsm_ctx, channel);
    // gsm0710_open_channel returns: 0 = invalid channel (failure), 1 = success
    if (ret == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to open channel %d: invalid channel", channel);
        return ESP_FAIL;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Channel %d opened successfully", channel);

    return ESP_OK;
}

/**
 * @brief Close a GSM0710 channel
 */
esp_err_t gsm0710_manager_close_channel(int channel)
{
    if (!g_gsm_mgr.initialized || g_gsm_mgr.gsm_ctx == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel < 1 || channel > GSM0710_MAX_CHANNEL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid channel number: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Closing channel %d", channel);

    gsm0710_close_channel(g_gsm_mgr.gsm_ctx, channel);

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Channel %d closed successfully", channel);

    return ESP_OK;
}

/**
 * @brief Send data over a GSM0710 channel
 */
esp_err_t gsm0710_manager_send(int channel, const uint8_t *data, size_t len)
{
    if (!g_gsm_mgr.initialized || g_gsm_mgr.gsm_ctx == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel < 1 || channel > GSM0710_MAX_CHANNEL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid channel number: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL || len == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid data or length");
        return ESP_ERR_INVALID_ARG;
    }

    // Check if channel is open
    if (!gsm0710_is_channel_open(g_gsm_mgr.gsm_ctx, channel)) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Channel %d is not open", channel);
        return ESP_ERR_INVALID_STATE;
    }

    // Log AT channel data for debugging
    if (channel == GSM0710_CHANNEL_AT) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 MUX WRITE (ch=%d): %.*s",
                       channel, (int)len, (char *)data);
    } else {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 MUX WRITE (ch=%d): %d bytes", channel, len);
    }

    // Send data via GSM0710
    // Note: gsm0710_write_frame will internally call lock_cb/unlock_cb
    // to protect ctx->frame_buffer from concurrent access
    gsm0710_write_data(g_gsm_mgr.gsm_ctx, channel, data, len);

    return ESP_OK;
}

/**
 * @brief Receive data from a GSM0710 channel
 */
esp_err_t gsm0710_manager_receive(int channel, uint8_t *data, size_t *len, TickType_t timeout)
{
    if (!g_gsm_mgr.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GSM0710 Manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel < 1 || channel > GSM0710_MAX_CHANNEL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid channel number: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL || len == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid data or length pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // For AT channel, check if there's local command response
    if (channel == GSM0710_CHANNEL_AT) {
        // Wait for response signal
        if (xSemaphoreTake(g_local_resp_sem, timeout) == pdPASS) {
            // Copy response from buffer
            size_t copy_len = (g_local_resp_len < *len) ? g_local_resp_len : (*len - 1);
            memcpy(data, g_local_resp_buf, copy_len);
            data[copy_len] = '\0';
            *len = copy_len;

            // Clear buffer
            g_local_resp_len = 0;

            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Received %d bytes from local command buffer", copy_len);
            return ESP_OK;
        } else {
            // Timeout
            *len = 0;
            return ESP_ERR_TIMEOUT;
        }
    }

    // For other channels, use queue
    if (g_gsm_mgr.data_queues[channel] == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Queue for channel %d not available", channel);
        return ESP_ERR_INVALID_STATE;
    }

    gsm0710_packet_t packet;
    if (xQueueReceive(g_gsm_mgr.data_queues[channel], &packet, timeout) == pdPASS) {
        // Copy data to output buffer
        size_t copy_len = (packet.len < *len) ? packet.len : *len;
        memcpy(data, packet.data, copy_len);
        *len = copy_len;

        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Received %d bytes from channel %d", copy_len, channel);
        return ESP_OK;
    } else {
        // Timeout or queue error
        *len = 0;
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief Register AT Channel Data Receive Callback
 */
esp_err_t gsm0710_manager_register_at_callback(gsm0710_data_received_cb_t callback, void *user_data)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Registering AT channel callback: %p, user_data: %p", callback, user_data);

    g_at_channel_callback = callback;
    g_at_channel_callback_user_data = user_data;

    return ESP_OK;
}

/**
 * @brief Register Voice Data Channel Data Receive Callback
 */
esp_err_t gsm0710_manager_register_voice_callback(gsm0710_data_received_cb_t callback, void *user_data)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Registering Voice channel (9) callback: %p, user_data: %p", callback, user_data);

    g_voice_channel_callback = callback;
    g_voice_channel_callback_user_data = user_data;

    return ESP_OK;
}

/**
 * @brief Set local command waiting flag for AT channel
 */
esp_err_t gsm0710_manager_set_local_command_waiting(bool waiting)
{
    SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Setting local command waiting: %s", waiting ? "true" : "false");

    if (waiting) {
        // Clear buffer before waiting for new command
        g_local_resp_len = 0;
    }

    g_local_command_waiting = waiting;
    return ESP_OK;
}
