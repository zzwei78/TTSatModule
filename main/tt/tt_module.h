/*
 * tt_module.h - Tiantong Module Control Interface
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef TT_MODULE_H
#define TT_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/queue.h"
#include "tt/gsm0710_manager.h"

/* Tiantong Module Channel Aliases */
#define TT_CHANNEL_MUX_CONTROL  GSM0710_CHANNEL_MUX_CONTROL  // MUX Control Channel
#define TT_CHANNEL_AT          GSM0710_CHANNEL_AT          // General AT Commands
#define TT_CHANNEL_VOICE_AT    GSM0710_CHANNEL_VOICE_AT    // Voice AT Commands
#define TT_CHANNEL_SMS         GSM0710_CHANNEL_SMS         // SMS AT Commands
#define TT_CHANNEL_DATA_AT     GSM0710_CHANNEL_DATA_AT     // Data AT Commands
#define TT_CHANNEL_VOICE_DATA  GSM0710_CHANNEL_VOICE_DATA  // Voice Data
#define TT_CHANNEL_IOT         GSM0710_CHANNEL_IOT         // IoT Data

/* UART Configuration */
#define TT_UART_AT_PORT_NUM     UART_NUM_1   // AT Command Port
#define TT_UART_AT_TX_PIN       17
#define TT_UART_AT_RX_PIN       18
#define TT_UART_AT_RTS_PIN      19
#define TT_UART_AT_CTS_PIN      20
#define TT_UART_AT_BAUD_RATE    115200
#define TT_UART_AT_BAUD_HIGH    921600       // High speed baud rate

#define TT_UART_LOG_PORT_NUM    UART_NUM_2   // Log Port
#define TT_UART_LOG_TX_PIN      4
#define TT_UART_LOG_RX_PIN      5
#define TT_UART_LOG_BAUD_RATE   3000000  // 3Mbps for Tiantong module log output

/* Baud Rate Switch Configuration */
#define TT_BAUD_SWITCH_TIMEOUT_MS    5000   // 5 seconds timeout for baud rate switch
#define TT_BAUD_SWITCH_MAX_RETRIES    3      // Maximum retry attempts

/* AT Command Response Buffer Size */
#define TT_AT_RESP_BUF_SIZE     1024

/* AT Command Timeout */
#define TT_AT_CMD_TIMEOUT_MS    5000

/* ========== Tiantong Module State Definitions ========== */

/**
 * @brief 天通模块主状态
 */
typedef enum {
    TT_STATE_HARDWARE_FAULT   = 0,  /* 硬件异常 */
    TT_STATE_INITIALIZING     = 1,  /* 初始化阶段 */
    TT_STATE_WAITING_MUX_RESP = 2,  /* 等待MUX响应（内部瞬态） */
    TT_STATE_LOW_BATTERY_OFF  = 3,  /* 低电关机（自动保护） */
    TT_STATE_USER_OFF         = 4,  /* 用户关机（NVS持久化） */
    TT_STATE_WORKING          = 5,  /* 正常工作 */
    TT_STATE_UPDATING         = 6,  /* 升级状态（OTA） */
} tt_state_t;

/* ========== Tiantong Module Error Codes ========== */

typedef enum {
    TT_ERROR_NONE = 0,                   /* No error */
    TT_ERROR_MUX_FAILED = 1,            /* MUX switch failed */
    TT_ERROR_COMM_TIMEOUT = 2,          /* Communication timeout */
    TT_ERROR_SIM_NOT_READY = 3,        /* SIM card not ready */
    TT_ERROR_NETWORK_REG_FAILED = 4,    /* Network registration failed */
    TT_ERROR_LOW_BATTERY = 5,           /* Low battery voltage */
    TT_ERROR_HARDWARE_FAULT = 6,        /* Hardware fault */
    TT_ERROR_UNKNOWN = 0xFF             /* Unknown error */
} tt_error_code_t;

/* ========== Tiantong Module Status Information ========== */

/**
 * @brief 天通模块状态信息
 *
 * 注意：工作子状态（SIM状态、网络注册状态）由客户端通过AT命令查询，
 * 设备端只返回主状态和电池电压。
 */
typedef struct {
    tt_state_t state;              /* 主状态 */
    uint16_t voltage_mv;           /* 当前电池电压 */
    uint8_t error_code;            /* 错误码 (state=HARDWARE_FAULT时有效) */
    uint8_t reserved1;             /* 预留 */
    uint8_t reserved2;             /* 预留 */
} tt_status_info_t;

/* ========== Legacy Status Type (for backward compatibility) ========== */

/**
 * @brief Legacy module status type (deprecated, use tt_state_t instead)
 */
typedef enum {
    TT_MODULE_STATUS_POWER_OFF = 0,      /* Power off state */
    TT_MODULE_STATUS_INITIALIZING = 1,   /* Initializing */
    TT_MODULE_STATUS_WORKING = 2,        /* Working */
    TT_MODULE_STATUS_OTA_UPDATING = 3,   /* OTA updating */
    TT_MODULE_STATUS_ERROR = 4           /* Hardware fault */
} tt_module_status_t;

/* ========== Legacy Types (for compatibility) ========== */

/* Tiantong Module Event Types */
typedef enum {
    TT_EVENT_NONE = 0,
    TT_EVENT_SIMST,            // SIM Status Changed
    TT_EVENT_REG,              // Network Registration Status
    TT_EVENT_CALL_STATUS,      // Call Status Changed
    TT_EVENT_SMS_RECEIVED,     // SMS Received
    TT_EVENT_DATA_READY,       // Data Ready on Channel
    TT_EVENT_ERROR             // Error Occurred
} tt_event_type_t;

/* Tiantong Module Event Data */
typedef struct {
    tt_event_type_t type;      // Event type
    union {
        int sim_status;        // SIM status code (for TT_EVENT_SIMST)
        int reg_status;        // Registration status code (for TT_EVENT_REG)
        int call_status;       // Call status code (for TT_EVENT_CALL_STATUS)
        struct {
            int channel;       // Channel number (for TT_EVENT_DATA_READY)
            size_t len;        // Data length
        } data;
    } data;                    // Event specific data
} tt_event_t;

/* AT Command Result */
typedef enum {
    TT_AT_RESULT_OK = 0,
    TT_AT_RESULT_ERROR,
    TT_AT_RESULT_TIMEOUT,
    TT_AT_RESULT_BUSY
} tt_at_result_t;

/* Tiantong Module UART Modes */
typedef enum {
    TT_UART_MODE_AT = 0,           // AT command mode (single task)
    TT_UART_MODE_MUX = 1           // MUX mode (multiple channels)
} tt_uart_mode_t;

/* Tiantong Module Context */
typedef struct {
    QueueHandle_t event_queue;    // Event queue
    bool initialized;             // Initialization flag
    tt_state_t state;             // Current module state
    tt_uart_mode_t uart_mode;     // Current UART mode
    TaskHandle_t uart_at_task_handle;  // UART AT task handle
    SemaphoreHandle_t mode_mutex; // Mutex for protecting mode changes
} tt_module_t;

/* ========== Initialization APIs ========== */

/**
 * @brief Initialize Tiantong Module Control
 *
 * @param event_queue_size Size of the event queue (0 to disable events)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_init(int event_queue_size);

/**
 * @brief Deinitialize Tiantong Module Control
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_deinit(void);

/**
 * @brief Start Tiantong Module Communication
 *
 * This function initializes UART interfaces and GSM0710 MUX
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_start(void);

/**
 * @brief Stop Tiantong Module Communication
 *
 * This function deinitializes UART interfaces and GSM0710 MUX
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_stop(void);

/* ========== AT Command APIs ========== */

/**
 * @brief Send AT Command to Tiantong Module (Universal Interface - Passthrough Mode)
 *
 * This is a universal interface that automatically detects the current mode
 * (AT command mode or MUX mode) and sends the command accordingly.
 * This function does NOT wait for response, suitable for passthrough usage like BLE GATT.
 *
 * The function is thread-safe and can be called from any context including
 * BLE GATT handlers, tasks, and interrupt handlers.
 *
 * @param cmd AT command string (without "AT" prefix and CRLF, e.g., "+CPIN?")
 *            The function will automatically add "AT" prefix and CRLF.
 *            If cmd already contains "AT" prefix, it will be used as-is.
 * @return tt_at_result_t Result code:
 *         - TT_AT_RESULT_OK: Command sent successfully
 *         - TT_AT_RESULT_ERROR: Command failed to send
 *         - TT_AT_RESULT_BUSY: Mutex acquisition failed
 *
 * @note In AT command mode, the function sends directly via UART.
 * @note In MUX mode, the function sends via GSM0710 channel 1.
 * @note This function is thread-safe and uses mutex for synchronization.
 * @note This function does NOT wait for response - suitable for passthrough usage.
 * @note If module is in POWER_OFF or ERROR state, returns TT_AT_RESULT_ERROR.
 */
tt_at_result_t tt_module_send_at_cmd(const char *cmd);

/**
 * @brief Send AT Command and Wait for Response
 *
 * This function sends an AT command and waits for the response.
 *
 * @param cmd AT command string (without "AT" prefix and CRLF)
 * @param response Buffer to store response
 * @param resp_len Response buffer size
 * @param timeout_ms Timeout in milliseconds
 * @return tt_at_result_t Result code:
 *         - TT_AT_RESULT_OK: Command successful, response contains data
 *         - TT_AT_RESULT_ERROR: Command failed or error response
 *         - TT_AT_RESULT_TIMEOUT: No response within timeout
 *         - TT_AT_RESULT_BUSY: Mutex acquisition failed
 *
 * @note This function is thread-safe and uses mutex for synchronization.
 * @note If module is in POWER_OFF or ERROR state, returns TT_AT_RESULT_ERROR.
 */
tt_at_result_t tt_module_send_at_cmd_wait(const char *cmd, char *response, size_t resp_len, int timeout_ms);

/**
 * @brief Send AT Command via GATT (Asynchronous with connection handle)
 *
 * This function sends an AT command asynchronously for BLE GATT service.
 * Response will be routed to the specified GATT connection.
 *
 * @param cmd AT command string (without "AT" prefix and CRLF)
 * @param conn_handle GATT connection handle for response routing
 * @return tt_at_result_t Result code:
 *         - TT_AT_RESULT_OK: Command sent successfully
 *         - TT_AT_RESULT_ERROR: Command failed to send
 *         - TT_AT_RESULT_BUSY: Local command in progress
 *
 * @note This function is specifically for BLE GATT AT service.
 * @note Response will be routed back to the specified GATT connection.
 * @note This function does NOT wait for response.
 * @note If module is in POWER_OFF or ERROR state, returns TT_AT_RESULT_ERROR.
 */
tt_at_result_t tt_module_send_at_cmd_gatt(const char *cmd, uint16_t conn_handle);

/**
 * @brief Notify tt_module about GATT disconnection
 *
 * This function should be called when a GATT connection is closed
 * to update the last active GATT connection handle used for unsolicited notifications.
 *
 * @param conn_handle The disconnected connection handle
 */
void tt_module_notify_gatt_disconnect(uint16_t conn_handle);

/**
 * @brief Set active GATT connection for AT response routing
 *
 * This function sets the active GATT connection handle for routing
 * unsolicited AT notifications (like incoming calls, network status, etc.).
 * It should be called when a BLE GATT client connects.
 *
 * @param conn_handle BLE GATT connection handle
 */
void tt_module_set_active_gatt_conn(uint16_t conn_handle);

/* ========== Data Transfer APIs ========== */

/**
 * @brief Send Data to Tiantong Module
 *
 * @param channel Channel number (1-3)
 * @param data Data buffer
 * @param len Data length
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_send_data(int channel, const uint8_t *data, size_t len);

/**
 * @brief Receive Data from Tiantong Module
 *
 * @param channel Channel number
 * @param data Buffer to store received data
 * @param len Pointer to store received data length
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_receive_data(int channel, uint8_t *data, size_t *len, int timeout_ms);

/* ========== Event APIs ========== */

/**
 * @brief Get Next Event from Tiantong Module
 *
 * @param event Event structure to fill
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_get_event(tt_event_t *event, int timeout_ms);

/* ========== State Query APIs ========== */

/**
 * @brief Get Current Tiantong Module State
 *
 * @return tt_state_t Current module state
 */
tt_state_t tt_module_get_state(void);

/**
 * @brief Get TT Module Status (Legacy, for backward compatibility)
 *
 * @deprecated Use tt_module_get_state() or tt_module_get_status_info() instead
 * @return tt_module_status_t Legacy status code
 */
tt_module_status_t tt_module_get_status(void);

/**
 * @brief Get TT Module Status Information
 *
 * @param info Pointer to store status information
 * @return esp_err_t
 *     - ESP_OK: Get status successfully
 *     - ESP_ERR_INVALID_ARG: info is NULL
 */
esp_err_t tt_module_get_status_info(tt_status_info_t *info);

/* ========== Power Control APIs ========== */

/**
 * @brief User Power On TT Module
 *
 * This function is called by user to power on the TT module.
 * It checks:
 * 1. Battery voltage (if read succeeds, must be above threshold)
 * 2. Current state (LOW_BATTERY_OFF, UPDATING, INITIALIZING not allowed)
 *
 * Operations:
 * 1. Clear NVS user manual off flag
 * 2. Hardware power on
 * 3. State -> INITIALIZING -> WORKING
 *
 * @return esp_err_t
 *     - ESP_OK: Power on successful
 *     - ESP_ERR_INVALID_STATE: Current state not allowed (low battery, updating, etc.)
 */
esp_err_t tt_module_user_power_on(void);

/**
 * @brief User Power Off TT Module
 *
 * This function is called by user to power off the TT module.
 * It checks:
 * 1. Battery voltage (if read succeeds, must be above threshold)
 * 2. Current state (LOW_BATTERY_OFF, UPDATING, INITIALIZING not allowed)
 *
 * Operations:
 * 1. Set NVS user manual off flag
 * 2. Hardware power off
 * 3. State -> USER_OFF
 *
 * @return esp_err_t
 *     - ESP_OK: Power off successful
 *     - ESP_ERR_INVALID_STATE: Current state not allowed (low battery, updating, etc.)
 */
esp_err_t tt_module_user_power_off(void);

/**
 * @brief Check if user control is allowed
 *
 * @return true if user control is allowed, false otherwise
 */
bool tt_module_is_user_control_allowed(void);

/**
 * @brief Power On Tiantong Module (Internal)
 *
 * This function is for internal use by power management module.
 * It does NOT check battery voltage or user permission.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_power_on(void);

/**
 * @brief Power Off Tiantong Module (Internal)
 *
 * This function is for internal use by power management module.
 * It does NOT check battery voltage or user permission.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_power_off(void);

/**
 * @brief Shutdown TT Module with reason code (Internal)
 *
 * This function powers off the module and records the shutdown reason.
 * For internal use by power management module.
 *
 * @param reason Shutdown reason (tt_error_code_t)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_shutdown_with_reason(uint8_t reason);

/**
 * @brief Complete startup of TT Module
 *
 * This is the complete startup flow including:
 * 1. Initialize (create queues, semaphores, etc.)
 * 2. Power on hardware
 * 3. Start communication
 * 4. Re-register callbacks
 *
 * @return esp_err_t
 *     - ESP_OK: Startup successful
 *     - ESP_ERR_INVALID_STATE: Already powered on
 *     - ESP_FAIL: Startup failed (check logs)
 */
esp_err_t tt_module_full_startup(void);

/**
 * @brief Complete shutdown of TT Module
 *
 * This is the complete shutdown flow including:
 * 1. Stop communication
 * 2. Cleanup state (clear SIMST, MUX mode, callbacks)
 * 3. Power off hardware
 * 4. Deinitialize
 *
 * @return esp_err_t
 *     - ESP_OK: Shutdown successful
 *     - ESP_ERR_INVALID_STATE: Already powered off
 *     - ESP_ERR_NOT_ALLOWED: OTA in progress, shutdown not allowed
 */
esp_err_t tt_module_full_shutdown(void);

/**
 * @brief Check if TT Module is powered on
 *
 * @return true if powered on, false otherwise
 */
bool tt_module_is_powered(void);

/* ========== Utility APIs ========== */

/**
 * @brief Reset Tiantong Module State to AT Command Mode
 *
 * This function resets the module state and UART mode back to AT command mode
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_reset_state(void);

/**
 * @brief Restart AT Command Task
 *
 * This function restarts the AT command task after MUX mode failure
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_restart_at_task(void);

/**
 * @brief Reset Tiantong Module
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_reset(void);

/**
 * @brief Cleanup Tiantong Module State
 *
 * This function clears all module state including SIMST detection, MUX mode,
 * and data callbacks. It should be called when the module is powered off
 * or needs to be fully reinitialized.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_cleanup(void);

/* ========== UART APIs ========== */

/**
 * @brief Write data to AT UART port
 *
 * @param data Pointer to data buffer
 * @param len Length of data to write
 * @return int Number of bytes written, or -1 on error
 */
int tt_module_uart_write(const void *data, size_t len);

/**
 * @brief Read data from AT UART port
 *
 * @param data Pointer to buffer to store read data
 * @param len Maximum length of data to read
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes read, or -1 on error
 */
int tt_module_uart_read(void *data, size_t len, int timeout_ms);

/**
 * @brief Check if AT UART port is initialized
 *
 * @return bool True if UART is initialized, false otherwise
 */
bool tt_module_uart_is_initialized(void);

/* ========== OTA APIs ========== */

/**
 * @brief Enter OTA Update Mode
 *
 * This function prepares the module for OTA firmware update.
 * It will stop normal communication and switch to OTA mode.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_enter_ota_mode(void);

/**
 * @brief Exit OTA Update Mode
 *
 * This function restores normal operation after OTA update.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_module_exit_ota_mode(void);

/**
 * @brief Check if module is in OTA mode
 *
 * @return bool True if in OTA mode, false otherwise
 */
bool tt_module_is_in_ota_mode(void);

/* ========== Data Callback APIs ========== */

/**
 * @brief Data Receive Callback Function Type
 *
 * This callback is invoked when data is received from the Tiantong module
 * and needs to be forwarded to the upper layer (e.g., JTAG, BLE, etc.)
 *
 * @param data Received data buffer
 * @param len Length of received data
 * @param user_data User data passed during registration
 */
typedef void (*tt_data_received_cb_t)(const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Register Data Receive Callback
 *
 * This function registers a callback to handle data received from the Tiantong module.
 * The callback will be invoked from the UART RX task context, so it should not block.
 *
 * @param callback Callback function pointer (NULL to unregister)
 * @param user_data User data to pass to the callback
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_register_data_callback(tt_data_received_cb_t callback, void *user_data);

/* ========== Internal APIs ========== */

/**
 * @brief Route AT response data (for MUX mode callback)
 *
 * This function is called by gsm0710_manager when AT data is received in MUX mode.
 * It routes the response based on the current AT command context (LOCAL/GATT/NONE).
 *
 * @param data AT response data
 * @param len Data length
 * @param user_data Not used
 */
void tt_module_route_at_response(const uint8_t *data, size_t len, void *user_data);

#endif /* TT_MODULE_H */
