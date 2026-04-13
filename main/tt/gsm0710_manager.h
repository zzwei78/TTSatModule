/*
 * gsm0710_manager.h - GSM0710 MUX Manager for ESP-IDF
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef GSM0710_MANAGER_H
#define GSM0710_MANAGER_H

#include "gsm0710.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* UART Configuration for TT Module */
#define UART_TT_PORT_NUM      UART_NUM_1
#define UART_TT_TX_PIN        17
#define UART_TT_RX_PIN        18
#define UART_TT_RTS_PIN       19
#define UART_TT_CTS_PIN       20
#define UART_TT_BAUD_RATE     115200

/* GSM0710 Channel Configuration */
#define GSM0710_CHANNEL_MUX_CONTROL 0   // MUX Control Channel
#define GSM0710_CHANNEL_AT          1   // General AT Commands
#define GSM0710_CHANNEL_VOICE_AT    2   // Voice AT Commands
#define GSM0710_CHANNEL_SMS         3   // SMS AT Commands
#define GSM0710_CHANNEL_DATA_AT     4   // Data AT Commands
#define GSM0710_CHANNEL_VOICE_DATA  9   // Voice Data
#define GSM0710_CHANNEL_IOT         10  // IoT Data

/* Maximum channel number */
#define GSM0710_MAX_CHANNEL         10

/* Data Queue Configuration */
#define GSM0710_QUEUE_SIZE    5   // Number of packets per channel (reduced from 10)
#define GSM0710_PACKET_SIZE   256 // Maximum packet size (reduced from 512)

/* Packet structure for channel data */
typedef struct {
    int channel;                  // Channel number
    uint8_t data[GSM0710_PACKET_SIZE]; // Data buffer
    size_t len;                   // Data length
} gsm0710_packet_t;

/* MUX Reconnection Configuration */
#define GSM0710_RECONNECT_MAX_ATTEMPTS 5    // Maximum number of reconnection attempts
#define GSM0710_RECONNECT_INTERVAL_MS  3000  // Interval between reconnection attempts

/* MUX Reconnection States */
typedef enum {
    GSM0710_RECONNECT_STATE_IDLE = 0,
    GSM0710_RECONNECT_STATE_RECONNECTING,
    GSM0710_RECONNECT_STATE_FAILED
} gsm0710_reconnect_state_t;

/* Ring Buffer Configuration for UART Data */
#define GSM0710_RINGBUF_SIZE 4096  // Size of the ring buffer for UART data (increased from 2048)

/* Heartbeat Configuration */
#define GSM0710_HEARTBEAT_INTERVAL_MS    60000  // Heartbeat interval: 30 seconds
#define GSM0710_HEARTBEAT_TIMEOUT_MS     5000   // Heartbeat response timeout: 5 seconds
#define GSM0710_HEARTBEAT_MAX_FAILURES   3      // Max heartbeat failures before recovery

/* Ring Buffer Structure */
typedef struct {
    uint8_t buffer[GSM0710_RINGBUF_SIZE];
    int head;  // Index of the next byte to write
    int tail;  // Index of the next byte to read
    int size;  // Total size of the ring buffer
    int used;  // Number of bytes used in the ring buffer
} gsm0710_ringbuf_t;

/* GSM0710 Manager Context */
typedef struct {
    struct gsm0710_context *gsm_ctx;  // GSM0710 context
    TaskHandle_t rx_task_handle;      // UART RX Task handle
    TaskHandle_t tx_task_handle;      // UART TX Task handle
    TaskHandle_t reconnect_task_handle;  // Reconnection Task handle
    QueueHandle_t data_queues[GSM0710_MAX_CHANNEL + 1];  // Data queues for each channel
    SemaphoreHandle_t mutex;          // Mutex for protecting shared resources
    uart_port_t uart_num;             // UART port number
    bool initialized;                 // Initialization flag
    bool started;                     // MUX session started flag
    
    // Ring buffer for UART data
    gsm0710_ringbuf_t uart_ringbuf;   // Ring buffer for incoming UART data
    SemaphoreHandle_t ringbuf_mutex;  // Mutex for protecting ring buffer access
    SemaphoreHandle_t data_ready_sem;  // Semaphore to signal data is ready for GSM0710
    
    // Reconnection variables
    gsm0710_reconnect_state_t reconnect_state;  // Current reconnection state
    int reconnect_attempts;           // Number of reconnection attempts
    SemaphoreHandle_t reconnect_sem;  // Semaphore to trigger reconnection

    // Heartbeat variables
    TaskHandle_t heartbeat_task_handle;  // Heartbeat task handle
    volatile bool heartbeat_waiting;     // Waiting for heartbeat response
    volatile int heartbeat_failures;     // Number of consecutive heartbeat failures
    SemaphoreHandle_t heartbeat_sem;     // Semaphore to signal heartbeat response received

    // Debug counters for tracking GSM0710 callback invocations
    volatile uint32_t read_cb_count;     // Number of read_cb invocations
    volatile uint32_t write_cb_count;    // Number of write_cb invocations
    volatile uint32_t deliver_data_cb_count;  // Number of deliver_data_cb invocations
    volatile uint32_t deliver_data_ch1_count; // Number of deliver_data_cb for channel 1
} gsm0710_manager_t;

/**
 * @brief Initialize GSM0710 Manager
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_init(void);

/**
 * @brief Deinitialize GSM0710 Manager
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_deinit(void);

/**
 * @brief Start GSM0710 MUX session
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_start(void);

/**
 * @brief Stop GSM0710 MUX session
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_stop(void);

/**
 * @brief Open a GSM0710 channel
 *
 * @param channel Channel number (1-63)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_open_channel(int channel);

/**
 * @brief Close a GSM0710 channel
 *
 * @param channel Channel number (1-63)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_close_channel(int channel);

/**
 * @brief Send data over a GSM0710 channel
 *
 * @param channel Channel number (1-63)
 * @param data Pointer to data buffer
 * @param len Data length
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_send(int channel, const uint8_t *data, size_t len);

/**
 * @brief Receive data from a GSM0710 channel
 *
 * @param channel Channel number (1-63)
 * @param data Pointer to data buffer
 * @param len Pointer to data length
 * @param timeout Timeout in ticks
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_receive(int channel, uint8_t *data, size_t *len, TickType_t timeout);

/**
 * @brief Data Receive Callback Function Type
 *
 * This callback is invoked when data is received from the GSM0710 MUX layer
 * and needs to be forwarded to the upper layer (e.g., JTAG, BLE, etc.)
 *
 * @param data Received data buffer
 * @param len Length of received data
 * @param user_data User data passed during registration
 */
typedef void (*gsm0710_data_received_cb_t)(const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Register Data Receive Callback for AT Channel
 *
 * This function registers a callback to handle data received from the AT channel (channel 1).
 * The callback will be invoked from the GSM0710 deliver_data context, so it should not block.
 *
 * @param callback Callback function pointer (NULL to unregister)
 * @param user_data User data to pass to the callback
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_register_at_callback(gsm0710_data_received_cb_t callback, void *user_data);

/**
 * @brief Register Data Receive Callback for Voice Data Channel
 *
 * This function registers a callback to handle data received from the Voice Data channel (channel 9).
 * The callback will be invoked from the GSM0710 deliver_data context, so it should not block.
 *
 * @param callback Callback function pointer (NULL to unregister)
 * @param user_data User data to pass to the callback
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t gsm0710_manager_register_voice_callback(gsm0710_data_received_cb_t callback, void *user_data);

/**
 * @brief Set local command waiting flag for AT channel
 *
 * This function controls whether AT channel responses should be routed to the local
 * command buffer (for TT_MODULE's own AT commands) or forwarded to the registered
 * callback (for BLE remote commands).
 *
 * Call this function with 'true' BEFORE sending local AT commands that you expect
 * to receive responses from via gsm0710_manager_receive(). The flag will be
 * automatically cleared after the first response is received.
 *
 * @param waiting true to route responses to local buffer, false for callback routing
 * @return esp_err_t ESP_OK on success
 *
 * @note This function is thread-safe
 * @note The local command buffer is automatically cleared when waiting=true is set
 */
esp_err_t gsm0710_manager_set_local_command_waiting(bool waiting);

#endif /* GSM0710_MANAGER_H */
