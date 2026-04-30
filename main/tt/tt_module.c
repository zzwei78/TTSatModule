/*
 * tt_module.c - Tiantong Module Control Interface
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
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "tt/tt_module.h"
#include "tt/tt_hardware.h"
#include "syslog.h"
#include "system/power_manage.h"
#include "config/user_params.h"
#include "bq27220.h"

/* External variable for BLE connection subscriptions */
/* Note: conn_handle_subs is not used in LED_TEST (no BLE) */

/* Tag for logging */
static const char *TAG = "TT_MODULE";

/* Global Tiantong Module Context */
static tt_module_t g_tt_module = {
    .event_queue = NULL,
    .initialized = false,
    .state = TT_STATE_USER_OFF,
    .uart_mode = TT_UART_MODE_AT,
    .uart_at_task_handle = NULL,
    .mode_mutex = NULL
};

/* ========== TT Module Status Management ========== */

/**
 * @brief TT Module Global Status Variables
 */
static bool g_tt_module_powered = false;              /* Power state */
static uint8_t g_tt_error_code = TT_ERROR_NONE;       /* Current error code */

/* UART Configuration for Log Port */
static const uart_config_t uart_log_config __attribute__((unused)) = {
    .baud_rate = TT_UART_LOG_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};

/* UART Configuration for AT Port */
static const uart_config_t uart_at_config __attribute__((unused)) = {
    .baud_rate = TT_UART_AT_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Disable hardware flow control
    .rx_flow_ctrl_thresh = 122,
};

/* Forward declarations */
static esp_err_t uart_log_init(void);
static void uart_log_deinit(void);
static void uart_log_rx_task(void *pvParameters);
static esp_err_t uart_at_init(void);
static void uart_at_deinit(void);
static void uart_at_rx_task(void *pvParameters);
static void tt_event_task(void *pvParameters);
static void tt_mux_init_task(void *pvParameters);
static void tt_module_forward_response_to_gatt(const uint8_t *data, size_t len);
void tt_module_route_at_response(const uint8_t *data, size_t len, void *user_data);
static esp_err_t tt_switch_to_high_baud_rate(void);
static tt_at_result_t tt_module_send_at_cmd_internal(const char *cmd, bool check_state);
static tt_at_result_t tt_module_send_at_cmd_local(const char *cmd, char *response, size_t resp_len, int timeout_ms);
static tt_at_result_t tt_module_send_at_cmd_gatt_internal(const char *cmd, uint16_t conn_handle);
static void set_tt_state(tt_state_t new_state);

/* Global variables for UART AT Port */
static bool g_simst_detected = false;
static bool g_tt_module_ready = false;  // TT module ready (^SIMST: detected)
static SemaphoreHandle_t g_simst_sem = NULL;
static TaskHandle_t g_mux_init_task_handle = NULL;
static SemaphoreHandle_t g_at_cmd_mutex = NULL;  // Mutex for AT command synchronization

/* ========== Unified AT Command Context ========== */

/* AT command response target type */
typedef enum {
    TT_AT_TARGET_NONE = 0,      // No active command (idle)
    TT_AT_TARGET_LOCAL,         // Local command (synchronous, wait for response)
    TT_AT_TARGET_GATT           // GATT command (asynchronous, callback via GATT)
} tt_at_target_t;

/* Unified AT command context */
typedef struct {
    tt_at_target_t target;          // Current command target type

    union {
        /* Local command context (synchronous) */
        struct {
            char *buffer;            // Response buffer
            size_t size;             // Buffer size
            size_t received;         // Bytes received
            SemaphoreHandle_t done_sem;  // Completion semaphore
            tt_at_result_t result;   // Result code
        } local;

        /* GATT command context (asynchronous) */
        struct {
            uint16_t conn_handle;    // GATT connection handle
        } gatt;
    };

    TickType_t cmd_start_time;      // Command start time (for timeout detection)
    uint16_t last_active_gatt_conn;  // Last active GATT connection for unsolicited notifications
} tt_at_context_t;

/* Global AT command context (single context, natural mutual exclusion) */
static tt_at_context_t g_at_context = {0};
static SemaphoreHandle_t g_at_context_mutex = NULL;

/* Static UART RX buffer pool - shared between tasks to save memory */
#define TT_UART_RX_BUF_SIZE 1024
static uint8_t g_uart_log_rx_buf[TT_UART_RX_BUF_SIZE];
static uint8_t g_uart_at_rx_buf[TT_UART_RX_BUF_SIZE];
static uint8_t g_simst_detect_buf_static[100];

/* Runtime control flags */
static volatile bool g_tt_module_running = false;  // Controls if UART tasks should process data

/* Data receive callback */
static tt_data_received_cb_t g_data_callback = NULL;
static void *g_data_callback_user_data = NULL;

/**
 * @brief Initialize UART for TT Module Log Port
 */
static esp_err_t uart_log_init(void)
{
    esp_err_t ret = ESP_OK;

    // Initialize UART2 using hardware layer
    ret = tt_hw_init_uart2(TT_UART_LOG_BAUD_RATE);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to initialize UART%d via hardware layer: %s", TT_UART_LOG_PORT_NUM, esp_err_to_name(ret));
        return ret;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART%d (Log Port) initialized successfully", TT_UART_LOG_PORT_NUM);
    return ret;
}

/**
 * @brief Deinitialize UART for TT Module Log Port
 */
static void uart_log_deinit(void)
{
    uart_driver_delete(TT_UART_LOG_PORT_NUM);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART%d (Log Port) deinitialized", TT_UART_LOG_PORT_NUM);
}

/**
 * @brief UART Log RX Task for reading log data from TT Module
 */
static void uart_log_rx_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART%d Log RX Task started", TT_UART_LOG_PORT_NUM);

    while (1) {
        // Pre-check available data to prevent buffer overflow
        size_t buffered_len = 0;
        uart_get_buffered_data_len(TT_UART_LOG_PORT_NUM, &buffered_len);
        size_t read_len = (buffered_len > TT_UART_RX_BUF_SIZE) ? TT_UART_RX_BUF_SIZE : buffered_len;

        // Read data from UART using static buffer
        int len = uart_read_bytes(TT_UART_LOG_PORT_NUM, g_uart_log_rx_buf, read_len, pdMS_TO_TICKS(100));

        if (len > 0) {
            // Ensure null termination (safe since we reserve space)
            if (len < TT_UART_RX_BUF_SIZE) {
                g_uart_log_rx_buf[len] = '\0';
            } else {
                g_uart_log_rx_buf[TT_UART_RX_BUF_SIZE - 1] = '\0';
            }
            // Print log data
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT_LOG: %s", g_uart_log_rx_buf);

            // Log if data was truncated
            if (buffered_len > TT_UART_RX_BUF_SIZE) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                    "UART log data truncated: %zu bytes available, read %d bytes",
                    buffered_len, len);
            }
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief Detect ^SIMST: pattern in UART data
 *
 * @param data Pointer to UART receive buffer
 * @param len Length of data in buffer
 * @param simst_buf_ptr Pointer to SIMST detection buffer
 * @param simst_buf_len_ptr Pointer to SIMST buffer length
 * @return true if ^SIMST: pattern found, false otherwise
 */
static bool detect_simst_pattern(const uint8_t *data, size_t len,
                                  uint8_t *simst_buf_ptr, int *simst_buf_len_ptr)
{
    const char *simst_prefix = "^SIMST: ";
    const int simst_prefix_len = 8;  // strlen("^SIMST: ")
    const int SIMST_DETECT_BUF_SIZE = 100;

    // Add new data to SIMST detection buffer (keep last 100 bytes)
    if (len >= SIMST_DETECT_BUF_SIZE) {
        // Large data, just keep the last 100 bytes
        memcpy(simst_buf_ptr, data + len - SIMST_DETECT_BUF_SIZE, SIMST_DETECT_BUF_SIZE);
        *simst_buf_len_ptr = SIMST_DETECT_BUF_SIZE;
    } else {
        // Normal data, append to buffer
        if (*simst_buf_len_ptr + len > SIMST_DETECT_BUF_SIZE) {
            // Buffer would overflow, shift old data out
            int overflow = (*simst_buf_len_ptr + len) - SIMST_DETECT_BUF_SIZE;
            memmove(simst_buf_ptr,
                   simst_buf_ptr + overflow,
                   *simst_buf_len_ptr - overflow);
            *simst_buf_len_ptr -= overflow;
        }
        memcpy(simst_buf_ptr + *simst_buf_len_ptr, data, len);
        *simst_buf_len_ptr += len;
    }

    // Search for ^SIMST: (common prefix for both 0 and 1)
    if (*simst_buf_len_ptr >= simst_prefix_len) {
        char *found_simst = memmem(simst_buf_ptr, *simst_buf_len_ptr,
                                   simst_prefix, simst_prefix_len);
        return (found_simst != NULL);
    }

    return false;
}

/**
 * @brief Handle ^SIMST: detection and trigger appropriate actions
 *
 * @param found_simst Pointer to the found "^SIMST:" string in the buffer
 * @param simst_buf_ptr Pointer to SIMST detection buffer
 * @param simst_buf_len_ptr Pointer to SIMST buffer length
 */
static void handle_simst_detection(const char *found_simst,
                                   uint8_t *simst_buf_ptr,
                                   int *simst_buf_len_ptr)
{
    const int simst_prefix_len = 8;  // strlen("^SIMST: ")

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Detected '^SIMST:', entering AT mode, discarding startup log...");

    // Check if it's ^SIMST: 1 (SIM ready) or ^SIMST: 0 (SIM not ready)
    bool sim_ready = false;
    if (found_simst + simst_prefix_len <= (char *)simst_buf_ptr + *simst_buf_len_ptr) {
        char simst_value = *(found_simst + simst_prefix_len);
        sim_ready = (simst_value == '1');
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "SIM status: %c (%s)", simst_value,
                 sim_ready ? "ready" : "not ready");
    }

    // Calculate how much data to keep (from ^SIMST: onwards)
    int offset = found_simst - (char *)simst_buf_ptr + simst_prefix_len + 1;  // +1 for the digit
    int keep_len = *simst_buf_len_ptr - offset;

    // Move ^SIMST:x and following data to start of buffer
    if (keep_len > 0 && keep_len <= *simst_buf_len_ptr) {
        memmove(simst_buf_ptr,
               simst_buf_ptr + offset,
               keep_len);
        *simst_buf_len_ptr = keep_len;
    } else {
        *simst_buf_len_ptr = 0;  // Nothing to keep
    }

    // Mark that we've found ^SIMST: (AT mode active)
    g_simst_detected = true;
    g_tt_module_ready = true;  // Module is now ready for AT commands

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT module ready (^SIMST: detected), AT commands now allowed");

    // Notify MUX init task or create one if not running
    if (sim_ready) {
        if (g_mux_init_task_handle == NULL) {
            // Task not running (e.g., previous attempt exited after baud rate failure)
            // Create a new mux init task
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "SIM ready! Creating new MUX init task...");
            if (xTaskCreate(tt_mux_init_task, "tt_mux_init_task", 8192, NULL, 8, &g_mux_init_task_handle) != pdPASS) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create MUX initialization task");
            } else {
                xSemaphoreGive(g_simst_sem);
            }
        } else {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "SIM ready! Notifying existing MUX init task...");
            xSemaphoreGive(g_simst_sem);
        }
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "SIM not ready, will wait for next ^SIMST: notification...");
    }
}

/**
 * @brief UART AT RX Task for reading AT data from TT Module
 */
static void uart_at_rx_task(void *pvParameters)
{
    int simst_buf_len = 0;
    int loop_count = 0;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART%d AT RX Task started (waiting for start signal)", TT_UART_AT_PORT_NUM);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> AT RX Task: task_running=true, g_tt_module_running=%d", g_tt_module_running);

    while (1) {
        loop_count++;

        // Wait for start signal if module is not running
        if (!g_tt_module_running) {
            if (loop_count % 50 == 1) {  // Log every 5 seconds
                SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT RX task waiting for g_tt_module_running=true");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // First time after module starts, log the state
        if (loop_count == 51) {  // Just after the first check passes
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> AT RX Task: g_tt_module_running=true, task now active");
        }

        // Check if UART mode has changed to MUX
        if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(0)) == pdTRUE) {
            bool is_mux_mode = (g_tt_module.uart_mode == TT_UART_MODE_MUX);
            xSemaphoreGive(g_tt_module.mode_mutex);

            if (is_mux_mode) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART mode changed to MUX, AT RX task exiting...");
                break;
            }
        }

        // Pause reading UART if we're waiting for MUX response
        // This allows the MUX init task to read the AT+CMUX response
        if (g_tt_module.state == TT_STATE_WAITING_MUX_RESP) {
            if (loop_count % 10 == 1) {  // Log every 1 second
                SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT RX task paused - state=%d (WAITING_MUX_RESP=%d), simst_detected=%d",
                         g_tt_module.state, (g_tt_module.state == TT_STATE_WAITING_MUX_RESP), g_simst_detected);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Pre-check available data to prevent buffer overflow
        size_t buffered_len = 0;
        uart_get_buffered_data_len(TT_UART_AT_PORT_NUM, &buffered_len);
        size_t read_len = (buffered_len > TT_UART_RX_BUF_SIZE) ? TT_UART_RX_BUF_SIZE : buffered_len;

        // Read data from UART using static buffer
        int len = uart_read_bytes(TT_UART_AT_PORT_NUM, g_uart_at_rx_buf, read_len, pdMS_TO_TICKS(100));

        // Log heartbeat every 10 seconds (100 iterations)
        if (loop_count % 100 == 0) {
            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT RX task heartbeat - running=%d, simst_detected=%d, state=%d",
                           g_tt_module_running, g_simst_detected, g_tt_module.state);
        }

        if (len > 0) {
            // Log if data was truncated
            if (buffered_len > TT_UART_RX_BUF_SIZE) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                    "UART AT data truncated: %zu bytes available, read %d bytes",
                    buffered_len, len);
            }
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "<<< UART RX RAW: %d bytes received", len);

            // Check for ^SIMST: if not already detected
            if (!g_simst_detected) {
                const char *simst_prefix = "^SIMST: ";

                if (detect_simst_pattern(g_uart_at_rx_buf, len,
                                        g_simst_detect_buf_static, &simst_buf_len)) {
                    // Found the prefix, now find exact location
                    char *found_simst = memmem(g_simst_detect_buf_static, simst_buf_len,
                                              simst_prefix, 8);  // strlen("^SIMST: ")
                    if (found_simst != NULL) {
                        handle_simst_detection(found_simst,
                                               g_simst_detect_buf_static,
                                               &simst_buf_len);
                    }
                }
            }

            // Route AT response using unified routing function
            // This function will route responses based on the current command context:
            // - LOCAL context: Save to local buffer (for synchronous commands)
            // - GATT context: Forward to GATT callback (for asynchronous commands)
            // - NONE context: Forward as unsolicited notification
            tt_module_route_at_response(g_uart_at_rx_buf, len, NULL);

            // Log for debugging - show hex dump
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "<<< UART RX: %d bytes: %.*s", len, len, (char *)g_uart_at_rx_buf);
        }
    }

    // Clean up
    g_tt_module.uart_at_task_handle = NULL;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART%d AT RX Task exited", TT_UART_AT_PORT_NUM);
    vTaskDelete(NULL);
}

/**
 * @brief MUX Initialization Task
 *
 * This task waits for SIMST:1 detection, sends AT command to enter MUX mode,
 * and starts the GSM0710 MUX module.
 */
/**
 * @brief MUX Initialization Task
 *
 * This task waits for SIMST:1 detection with retry logic, sends AT command to enter MUX mode,
 * and starts the GSM0710 MUX module.
 *
 * SIMST wait retry:
 * - Waits up to SIMST_WAIT_TIMEOUT_SEC per attempt
 * - On timeout: power cycles the TT module and retries
 * - After SIMST_MAX_RETRIES failures: declares HARDWARE_FAULT
 */
#define SIMST_WAIT_TIMEOUT_SEC    15    // Per-attempt SIMST wait timeout
#define SIMST_MAX_RETRIES         3     // Maximum power-cycle retries
#define SIMST_RETRY_DELAY_MS      2000  // Delay between power off and power on

static void tt_mux_init_task(void *pvParameters)
{
    esp_err_t ret = ESP_OK;
    char at_response[TT_AT_RESP_BUF_SIZE];
    bool mutex_taken = false;  /* Track mutex ownership for cleanup */

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX Initialization Task started");

    /* Cleanup label for error handling */
    bool cleanup_needed = false;

    // === SIMST detection with power-cycle retry ===
    bool simst_detected = false;

    for (int attempt = 1; attempt <= SIMST_MAX_RETRIES; attempt++) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Waiting for SIMST:1... (attempt %d/%d, timeout=%ds)",
                        attempt, SIMST_MAX_RETRIES, SIMST_WAIT_TIMEOUT_SEC);

        int waited_sec = 0;
        while (waited_sec < SIMST_WAIT_TIMEOUT_SEC) {
            if (xSemaphoreTake(g_simst_sem, pdMS_TO_TICKS(5000)) == pdTRUE) {
                simst_detected = true;
                break;
            }
            waited_sec += 5;
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                            "Waiting for SIMST:1... (%ds/%ds, attempt %d/%d)",
                            waited_sec, SIMST_WAIT_TIMEOUT_SEC, attempt, SIMST_MAX_RETRIES);
        }

        if (simst_detected) {
            break;  // Got SIMST, proceed to MUX init
        }

        // SIMST timeout for this attempt
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                        "SIMST timeout after %ds (attempt %d/%d)",
                        SIMST_WAIT_TIMEOUT_SEC, attempt, SIMST_MAX_RETRIES);

        if (attempt < SIMST_MAX_RETRIES) {
            // Power cycle the TT module and retry
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Power cycling TT module for retry...");

            // Drain any stale semaphore before power cycle
            xSemaphoreTake(g_simst_sem, 0);

            tt_hw_power_off();
            g_tt_module_running = false;
            g_simst_detected = false;
            g_tt_module_ready = false;
            vTaskDelay(pdMS_TO_TICKS(SIMST_RETRY_DELAY_MS));

            ret = tt_hw_power_on();
            if (ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Power on failed during retry: %s", esp_err_to_name(ret));
                continue;
            }
            ret = tt_hw_enter_normal_mode();
            if (ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Enter normal mode failed during retry: %s", esp_err_to_name(ret));
                continue;
            }

            g_tt_module_running = true;
            vTaskDelay(pdMS_TO_TICKS(500));  // Wait for module to start booting
        }
    }

    if (!simst_detected) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                        "SIMST not detected after %d attempts, declaring hardware fault",
                        SIMST_MAX_RETRIES);
        set_tt_state(TT_STATE_HARDWARE_FAULT);
        g_tt_error_code = TT_ERROR_COMM_TIMEOUT;
        tt_module_user_power_off();
        g_mux_init_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "SIMST:1 detected! Proceeding with baud rate switch and MUX initialization...");

    // Try to switch to high baud rate (921600) before MUX initialization
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Attempting to switch UART baud rate to 921600...");
    ret = tt_switch_to_high_baud_rate();

    // Give uart_at_rx_task time to stabilize after baud rate switch
    vTaskDelay(pdMS_TO_TICKS(50));

    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "High baud rate mode enabled (921600)");
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to switch to high baud rate, resetting Tiantong module...");

        // Reset Tiantong module
        tt_module_reset();

        // Clear SIMST detected flag to let uart_at_rx_task restart initialization
        g_simst_detected = false;

        // Exit current task - uart_at_rx_task will handle re-initialization
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX init task exiting, waiting for re-initialization after module reset...");
        set_tt_state(TT_STATE_INITIALIZING);
        g_mux_init_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Send AT+CMUX command to enter MUX mode
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Sending AT+CMUX command to enter MUX mode...");

    // Update state BEFORE sending command
    g_tt_module.state = TT_STATE_WAITING_MUX_RESP;  // Transient state, no status update

    // Format the AT command (without "AT" prefix, as tt_module_send_at_cmd will add it)
    // Use parameters: <mode>=0 (basic), <subset>=0 (no UIH frames only), <port_speed>=5 (115200), <N1>=1600 (max frame size)
    // Tiantong module only supports basic mode and UIH frames
    char cmux_cmd[] = "+CMUX=0,0,5,1600";

    // Use tt_module_send_at_cmd_wait for unified response handling
    // This will automatically pause uart_at_rx_task and read response
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Sending AT+CMUX and waiting for response...");
    tt_at_result_t send_result = tt_module_send_at_cmd_wait(cmux_cmd, at_response, sizeof(at_response), 5000);

    if (send_result != TT_AT_RESULT_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to send AT+CMUX command or get response, result=%d", send_result);
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Response: %s", at_response);
        set_tt_state(TT_STATE_HARDWARE_FAULT);
        g_tt_error_code = TT_ERROR_MUX_FAILED;
        tt_module_user_power_off();
        g_mux_init_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT+CMUX response: %s", at_response);

    // Check if response contains OK
    if (strstr(at_response, "OK") == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT+CMUX command failed");
        set_tt_state(TT_STATE_HARDWARE_FAULT);
        g_tt_error_code = TT_ERROR_MUX_FAILED;
        tt_module_user_power_off();
        g_mux_init_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Successfully entered MUX mode");
    // State remains INITIALIZING until CFUN=1 succeeds

    // Update UART mode to MUX mode
    if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_tt_module.uart_mode = TT_UART_MODE_MUX;
        xSemaphoreGive(g_tt_module.mode_mutex);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART mode changed to MUX");
    }
    
    // Initialize GSM0710 MUX Manager
    ret = gsm0710_manager_init();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to initialize GSM0710 Manager: %s", esp_err_to_name(ret));
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Reverting to AT command mode...");

        // Cleanup GSM0710 manager
        gsm0710_manager_deinit();

        // Hardware reset TT module to ensure clean state
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Performing hardware reset of TT module...");
        tt_module_reset();

        // Revert UART mode to AT mode
        if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_tt_module.uart_mode = TT_UART_MODE_AT;
            xSemaphoreGive(g_tt_module.mode_mutex);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART mode reverted to AT");
        }

        // Restart AT RX task
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Restarting AT RX task...");
        tt_module_restart_at_task();

        set_tt_state(TT_STATE_INITIALIZING);
        g_mux_init_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Start GSM0710 MUX session
    ret = gsm0710_manager_start();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to start GSM0710 MUX session: %s", esp_err_to_name(ret));
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Reverting to AT command mode...");

        // Cleanup GSM0710 manager
        gsm0710_manager_deinit();

        // Hardware reset TT module to ensure clean state
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Performing hardware reset of TT module...");
        tt_module_reset();

        // Revert UART mode to AT mode
        if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_tt_module.uart_mode = TT_UART_MODE_AT;
            xSemaphoreGive(g_tt_module.mode_mutex);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART mode reverted to AT");
        }

        // Restart AT RX task
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Restarting AT RX task...");
        tt_module_restart_at_task();

        set_tt_state(TT_STATE_INITIALIZING);
        g_mux_init_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Open channels according to configuration
    // Note: Channel 0 (MUX_CONTROL) is automatically opened when MUX session starts
    int channels[] = {
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
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to open channel %d: %s", channels[i], esp_err_to_name(ret));
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Reverting to AT command mode...");

            // Close any open channels
            for (int j = 0; j < i; j++) {
                gsm0710_manager_close_channel(channels[j]);
            }

            // Cleanup GSM0710 manager
            gsm0710_manager_stop();
            gsm0710_manager_deinit();

            // Hardware reset TT module to ensure clean state
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Performing hardware reset of TT module...");
            tt_module_reset();

            // Revert UART mode to AT mode
            if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_tt_module.uart_mode = TT_UART_MODE_AT;
                xSemaphoreGive(g_tt_module.mode_mutex);
                SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART mode reverted to AT");
            }

            // Restart AT RX task
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Restarting AT RX task...");
            tt_module_restart_at_task();

            set_tt_state(TT_STATE_INITIALIZING);
            g_mux_init_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX initialization completed successfully");

    // Register AT response routing callback with gsm0710_manager
    // This ensures that AT responses in MUX mode are routed through the unified routing function
    ret = gsm0710_manager_register_at_callback(tt_module_route_at_response, NULL);
    if (ret != ESP_OK) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to register AT callback with gsm0710_manager: %d", ret);
        // Continue anyway - MUX mode should still work, but responses may not be routed correctly
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Registered AT response routing callback with gsm0710_manager");
    }

    // ===== Configure terminal type and voice rate before AT+CFUN=1 =====
    // These commands must be sent before CFUN=1 according to the AT command documentation.

    // Step 1: Query current terminal type
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Querying terminal type (AT^TETYPE?)...");
    char tetype_response[128];
    bool tetype_needs_set = true;
    tt_at_result_t tetype_result = tt_module_send_at_cmd_wait("^TETYPE?", tetype_response, sizeof(tetype_response), 3000);
    if (tetype_result == TT_AT_RESULT_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^TETYPE? response: %s", tetype_response);
        // Parse response: "^TETYPE: <type>"
        char *p = strstr(tetype_response, "^TETYPE:");
        if (p != NULL) {
            int type = atoi(p + strlen("^TETYPE:"));
            if (type == 6) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Terminal type already 6 (降速语音), skip setting");
                tetype_needs_set = false;
            } else {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Current terminal type is %d, need to set to 6", type);
            }
        }
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^TETYPE? query failed (result=%d), will try to set", tetype_result);
    }

    // Step 2: Set terminal type to 6 if needed
    if (tetype_needs_set) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Setting terminal type to 6 (AT^TETYPE=6)...");
        char tetype_set_response[128];
        tt_at_result_t tetype_set_result = tt_module_send_at_cmd_wait("^TETYPE=6", tetype_set_response, sizeof(tetype_set_response), 3000);
        if (tetype_set_result == TT_AT_RESULT_OK) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^TETYPE=6 successful, rebooting system to take effect...");
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            return;  // Never reached, but for clarity
        } else {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^TETYPE=6 failed (result=%d), continuing without reboot", tetype_set_result);
        }
    }

    // Step 3: Query current voice rate
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Querying voice rate (AT^VOICERATE?)...");
    char voicerate_response[128];
    bool voicerate_needs_set = true;
    tt_at_result_t voicerate_result = tt_module_send_at_cmd_wait("^VOICERATE?", voicerate_response, sizeof(voicerate_response), 3000);
    if (voicerate_result == TT_AT_RESULT_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^VOICERATE? response: %s", voicerate_response);
        // Parse response: "^VOICERATE: <rate1>[,<rate2>[,<rate3>]]"
        // Check if rate 5 (1.0Kbps) is already present
        char *p = strstr(voicerate_response, "^VOICERATE:");
        if (p != NULL) {
            p += strlen("^VOICERATE:");
            // Check each comma-separated value
            bool has_1k = false;
            char *token = strtok(p, ", \r\n");
            while (token != NULL) {
                if (atoi(token) == 5) {
                    has_1k = true;
                    break;
                }
                token = strtok(NULL, ", \r\n");
            }
            if (has_1k) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Voice rate already includes 1.0Kbps (rate=5), skip setting");
                voicerate_needs_set = false;
            }
        }
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^VOICERATE? query failed (result=%d), will try to set", voicerate_result);
    }

    // Step 4: Set voice rate to 1.0Kbps if needed
    if (voicerate_needs_set) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Setting voice rate to 1.0Kbps (AT^VOICERATE=5)...");
        char voicerate_set_response[128];
        tt_at_result_t voicerate_set_result = tt_module_send_at_cmd_wait("^VOICERATE=5", voicerate_set_response, sizeof(voicerate_set_response), 3000);
        if (voicerate_set_result == TT_AT_RESULT_OK) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^VOICERATE=5 successful: %s", voicerate_set_response);
        } else {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^VOICERATE=5 failed (result=%d), continuing...", voicerate_set_result);
        }
    }

    // ===== End of terminal type and voice rate configuration =====

    // Send AT+CFUN=1 to enable full functionality after MUX mode initialization
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Sending AT+CFUN=1 to enable full functionality...");
    char cfun_response[128];
    tt_at_result_t cfun_result = tt_module_send_at_cmd_wait("+CFUN=1", cfun_response, sizeof(cfun_response), 5000);
    if (cfun_result != TT_AT_RESULT_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT+CFUN=1 failed (result=%d), module not functional", cfun_result);
        set_tt_state(TT_STATE_HARDWARE_FAULT);
        g_tt_error_code = TT_ERROR_COMM_TIMEOUT;
        tt_module_user_power_off();
        g_mux_init_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT+CFUN=1 successful: %s", cfun_response);

    // Send AT^VOICEMODE=1 to enable voice mode
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Sending AT^VOICEMODE=1 to enable voice mode...");
    char voicemode_response[128];
    tt_at_result_t voicemode_result = tt_module_send_at_cmd_wait("^VOICEMODE=1", voicemode_response, sizeof(voicemode_response), 2000);
    if (voicemode_result != TT_AT_RESULT_OK) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^VOICEMODE=1 failed (result=%d), voice not available", voicemode_result);
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT^VOICEMODE=1 successful: %s", voicemode_response);
    }

    // All initialization complete - now set to WORKING
    set_tt_state(TT_STATE_WORKING);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "=== TT Module fully initialized and WORKING ===");

    // This task is done, it can be deleted
    g_mux_init_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Tiantong Module Event Task
 */
static void tt_event_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT Event Task started");

    while (1) {
        // This task can be used to process module events
        // For example, process data from channels and generate events
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}

/**
 * @brief Initialize Tiantong Module Control
 */
esp_err_t tt_module_init(int event_queue_size)
{
    esp_err_t ret = ESP_OK;

    if (g_tt_module.initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module already initialized");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Initializing Tiantong Module Control");

    // Create event queue if requested
    if (event_queue_size > 0) {
        g_tt_module.event_queue = xQueueCreate(event_queue_size, sizeof(tt_event_t));
        if (g_tt_module.event_queue == NULL) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create event queue");
            return ESP_FAIL;
        }
    }

    // Create mode mutex for protecting UART mode changes
    g_tt_module.mode_mutex = xSemaphoreCreateMutex();
    if (g_tt_module.mode_mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create mode mutex");
        return ESP_FAIL;
    }

    // Create AT command mutex for synchronization
    g_at_cmd_mutex = xSemaphoreCreateMutex();
    if (g_at_cmd_mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create AT command mutex");
        return ESP_FAIL;
    }

    // Create AT command context mutex for unified response routing
    g_at_context_mutex = xSemaphoreCreateMutex();
    if (g_at_context_mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create AT context mutex");
        return ESP_FAIL;
    }

    // Initialize AT command context to idle state
    memset(&g_at_context, 0, sizeof(g_at_context));
    g_at_context.target = TT_AT_TARGET_NONE;

    /* Initialize status variables */
    g_tt_module_powered = false;
    g_tt_error_code = TT_ERROR_NONE;

    // Initialize Hardware Layer
    ret = tt_hw_init();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to initialize hardware layer");
        goto cleanup;
    }

    // Initialize UART Log Port
    ret = uart_log_init();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to initialize UART Log Port");
        goto cleanup;
    }

    // Initialize UART AT Port
    ret = uart_at_init();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to initialize UART AT Port");
        goto cleanup;
    }

    /*
     * UART Log RX Task Status: DISABLED
     *
     * Purpose: Read and log TT module's debug output at 3Mbps baudrate
     *
     * Issue: Watchdog timeout occurs when using ESP_LOGI at high baudrate
     *
     * Root Cause:
     * - TT module UART runs at 3000000 baud (3Mbps)
     * - At this speed, data arrives very quickly (~3 bytes/microsecond)
     * - ESP_LOGI is too slow and causes task watchdog timeout
     * - Task cannot keep up with data rate
     *
     * Current Mitigation:
     * - UART Log RX task disabled
     * - TT module logs not captured
     * - Functional impact: NONE (logs are for debugging only)
     *
     * Possible Solutions (if needed in future):
     * 1. Use ring buffer + queue to defer logging
     * 2. Increase task priority and stack size
     * 3. Use raw UART ISR with double buffering
     * 4. Reduce TT module baudrate (not recommended)
     *
     * Risk: LOW
     * - TT module operates independently without log capture
     * - All critical info is available via AT commands
     */
    /*
    if (xTaskCreate(uart_log_rx_task, "uart_log_rx_task", 8192, NULL, 5, NULL) != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create UART Log RX task");
        ret = ESP_FAIL;
        goto cleanup;
    }
    */

    // Create UART AT RX Task (increased stack for safety)
    if (xTaskCreate(uart_at_rx_task, "uart_at_rx_task", 8192, NULL, 10, &g_tt_module.uart_at_task_handle) != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create UART AT RX task");
        ret = ESP_FAIL;
        goto cleanup;
    }

    // Note: MUX Initialization Task will be created automatically when SIMST:1 is detected

    // Create Event Task if event queue is enabled
    if (g_tt_module.event_queue != NULL) {
        if (xTaskCreate(tt_event_task, "tt_event_task", 4096, NULL, 5, NULL) != pdPASS) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create TT Event task");
            ret = ESP_FAIL;
            goto cleanup;
        }
    }

    g_tt_module.initialized = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module Control initialized successfully");

    return ret;

cleanup:
    // Cleanup resources
    if (g_tt_module.event_queue != NULL) {
        vQueueDelete(g_tt_module.event_queue);
        g_tt_module.event_queue = NULL;
    }

    // Cleanup UART ports
    uart_log_deinit();
    uart_at_deinit();
    
    // Cleanup tasks
    if (g_mux_init_task_handle != NULL) {
        vTaskDelete(g_mux_init_task_handle);
        g_mux_init_task_handle = NULL;
    }

    g_tt_module.initialized = false;
    set_tt_state(TT_STATE_USER_OFF);
    g_simst_detected = false;

    return ret;
}

/**
 * @brief Initialize UART for TT Module AT Port
 */
static esp_err_t uart_at_init(void)
{
    esp_err_t ret = ESP_OK;

    // Set log level to INFO to see UART data
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // Initialize UART1 using hardware layer
    ret = tt_hw_init_uart1(TT_UART_AT_BAUD_RATE);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to initialize UART%d via hardware layer: %s", TT_UART_AT_PORT_NUM, esp_err_to_name(ret));
        return ret;
    }

    // Create SIMST detection semaphore
    g_simst_sem = xSemaphoreCreateBinary();
    if (g_simst_sem == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create SIMST semaphore");
        return ESP_FAIL;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART%d (AT Port) initialized successfully", TT_UART_AT_PORT_NUM);
    return ret;
}

/**
 * @brief Deinitialize UART for TT Module AT Port
 */
static void uart_at_deinit(void)
{
    uart_driver_delete(TT_UART_AT_PORT_NUM);

    // Delete SIMST semaphore if it exists
    if (g_simst_sem != NULL) {
        vSemaphoreDelete(g_simst_sem);
        g_simst_sem = NULL;
    }

    // Delete AT command mutex if it exists
    if (g_at_cmd_mutex != NULL) {
        vSemaphoreDelete(g_at_cmd_mutex);
        g_at_cmd_mutex = NULL;
    }

    // Delete AT command context mutex if it exists
    if (g_at_context_mutex != NULL) {
        vSemaphoreDelete(g_at_context_mutex);
        g_at_context_mutex = NULL;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART%d (AT Port) deinitialized", TT_UART_AT_PORT_NUM);
}

/**
 * @brief Deinitialize Tiantong Module Control
 */
esp_err_t tt_module_deinit(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Deinitializing Tiantong Module Control");

    // Stop Tiantong Module Communication
    tt_module_stop();

    // Delete event queue
    if (g_tt_module.event_queue != NULL) {
        vQueueDelete(g_tt_module.event_queue);
        g_tt_module.event_queue = NULL;
    }

    // Deinitialize Hardware Layer
    tt_hw_deinit();
    
    // Deinitialize UART ports
    uart_log_deinit();
    uart_at_deinit();

    // Cleanup tasks
    if (g_tt_module.uart_at_task_handle != NULL) {
        vTaskDelete(g_tt_module.uart_at_task_handle);
        g_tt_module.uart_at_task_handle = NULL;
    }

    if (g_mux_init_task_handle != NULL) {
        vTaskDelete(g_mux_init_task_handle);
        g_mux_init_task_handle = NULL;
    }

    g_tt_module.initialized = false;
    set_tt_state(TT_STATE_USER_OFF);
    g_simst_detected = false;
    
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module Control deinitialized successfully");

    return ESP_OK;
}

/**
 * @brief Start Tiantong Module Communication
 *
 * This function performs the complete startup sequence:
 * 1. Power on the module
 * 2. Enter normal mode
 * 3. Reset the module
 * 4. Activate UART tasks to start processing data
 *
 * The module starts in AT command mode. MUX mode will be automatically
 * started when SIMST:1 is detected.
 */
esp_err_t tt_module_start(void)
{
    esp_err_t ret;

    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_tt_module_running) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Module already running");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Starting Tiantong Module Communication");

    // Reset ready flags - module needs to send ^SIMST: again after restart
    g_simst_detected = false;
    g_tt_module_ready = false;

    // 1. Power on the module
    ret = tt_hw_power_on();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to power on module: %s", esp_err_to_name(ret));
        set_tt_state(TT_STATE_HARDWARE_FAULT);
        g_tt_error_code = TT_ERROR_HARDWARE_FAULT;
        tt_module_user_power_off();
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Enter normal mode (exit download mode if needed)
    ret = tt_hw_enter_normal_mode();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to enter normal mode: %s", esp_err_to_name(ret));
        tt_hw_power_off();
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // 3. Set running flag to wake up UART tasks
    g_tt_module_running = true;
    set_tt_state(TT_STATE_INITIALIZING);  // Will be WORKING after MUX init completes
    g_tt_module.uart_mode = TT_UART_MODE_AT;

    // Set powered flag
    g_tt_module_powered = true;

    // Clear any stale AT context
    if (g_at_context_mutex != NULL) {
        xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100));
        g_at_context.target = TT_AT_TARGET_NONE;
        xSemaphoreGive(g_at_context_mutex);
    }

    // Create MUX initialization task immediately (it waits for SIMST internally)
    if (g_mux_init_task_handle == NULL) {
        if (xTaskCreate(tt_mux_init_task, "tt_mux_init_task", 8192, NULL, 8, &g_mux_init_task_handle) != pdPASS) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create MUX initialization task");
        } else {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX init task created, waiting for SIMST detection...");
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module started in AT command mode (state=INITIALIZING)");

    return ESP_OK;
}

/**
 * @brief Stop Tiantong Module Communication
 *
 * This function performs a complete shutdown sequence:
 * 1. Stop UART tasks from processing data
 * 2. Exit MUX mode if active
 * 3. Power off the module
 * 4. Restore to initial state
 */
esp_err_t tt_module_stop(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_OK;
    }

    if (!g_tt_module_running) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Module already stopped");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Stopping Tiantong Module Communication");

    // 1. Clear running flag to pause UART tasks
    g_tt_module_running = false;

    // Clear any AT command context
    if (g_at_context_mutex != NULL) {
        xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100));
        g_at_context.target = TT_AT_TARGET_NONE;
        xSemaphoreGive(g_at_context_mutex);
    }

    // Give tasks time to notice the flag change
    vTaskDelay(pdMS_TO_TICKS(200));

    // 2. Check current mode and stop accordingly
    if (g_tt_module.uart_mode == TT_UART_MODE_MUX) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Stopping MUX mode...");

        // Stop GSM0710 MUX session
        esp_err_t ret = gsm0710_manager_stop();
        if (ret != ESP_OK) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to stop MUX session gracefully: %s", esp_err_to_name(ret));
        }

        // Deinitialize GSM0710 MUX Manager
        gsm0710_manager_deinit();

        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX mode stopped");
    }

    // 3. Power off the module
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Powering off TT module...");
    esp_err_t ret = tt_hw_power_off();
    if (ret != ESP_OK) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to power off module: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 4. Restore to initial state
    set_tt_state(TT_STATE_USER_OFF);
    g_tt_module.uart_mode = TT_UART_MODE_AT;
    g_simst_detected = false;
    g_tt_module_ready = false;  // Reset ready flag - module needs to re-announce ready state
    g_tt_module_powered = false;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module stopped and powered off");

    return ESP_OK;
}

/**
 * @brief Send AT Command to Tiantong Module (Internal Implementation)
 *
 * This is the internal implementation that performs the actual AT command sending.
 * It optionally checks module state based on the check_state parameter.
 *
 * @param cmd AT command string
 * @param check_state If true, check module state before sending
 * @return tt_at_result_t Result code
 */
static tt_at_result_t tt_module_send_at_cmd_internal(const char *cmd, bool check_state)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return TT_AT_RESULT_ERROR;
    }

    // Validate command parameter
    if (cmd == NULL || strlen(cmd) == 0 || strlen(cmd) > 256) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid command: NULL or empty or too long");
        return TT_AT_RESULT_ERROR;
    }

    // Check module state if requested (only allow WORKING for GATT client commands)
    if (check_state) {
        tt_state_t current_state = g_tt_module.state;
        if (current_state != TT_STATE_WORKING) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                "AT command not allowed in state %d (only WORKING state allowed for GATT commands)",
                current_state);
            return TT_AT_RESULT_ERROR;
        }
    }

    // Check if TT module is ready (^SIMST: detected)
    if (!g_tt_module_ready) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT module not ready (no ^SIMST: detected) - AT commands not allowed");
        return TT_AT_RESULT_ERROR;
    }

    // Check and wakeup BP BEFORE taking mutex (to avoid holding mutex during long wakeup wait)
    if (tt_hw_is_bp_sleeping()) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "BP is sleeping, waking up before sending AT command...");
        esp_err_t wakeup_ret = tt_hw_wakeup_bp(500);  // Wait up to 500ms for BP wakeup
        if (wakeup_ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to wakeup BP: %s", esp_err_to_name(wakeup_ret));
            return TT_AT_RESULT_ERROR;
        }
        // Add small delay after wakeup to ensure BP is ready to receive commands
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Take mutex to ensure thread-safe AT command sending
    if (xSemaphoreTake(g_at_cmd_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to take AT command mutex");
        return TT_AT_RESULT_BUSY;
    }

    // Determine current mode
    bool is_mux_mode = false;
    if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        is_mux_mode = (g_tt_module.uart_mode == TT_UART_MODE_MUX);
        xSemaphoreGive(g_tt_module.mode_mutex);
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Sending AT command: %s (mode: %s)", cmd, is_mux_mode ? "MUX" : "AT");

    // Format AT command with CRLF
    // Check if cmd already starts with "AT"
    char at_cmd[TT_AT_RESP_BUF_SIZE];
    memset(at_cmd, 0, sizeof(at_cmd));  // Clear buffer to avoid stale data
    int cmd_len;

    if ((cmd[0] == 'A' || cmd[0] == 'a') && (cmd[1] == 'T' || cmd[1] == 't')) {
        // Command already has "AT" prefix
        cmd_len = snprintf(at_cmd, sizeof(at_cmd), "%s\r\n", cmd);
    } else {
        // Add "AT" prefix
        cmd_len = snprintf(at_cmd, sizeof(at_cmd), "AT%s\r\n", cmd);
    }
    
    if (cmd_len >= sizeof(at_cmd)) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Command too long");
        xSemaphoreGive(g_at_cmd_mutex);
        return TT_AT_RESULT_ERROR;
    }

    esp_err_t ret = ESP_OK;

    if (is_mux_mode) {
        // MUX Mode: Send via GSM0710 channel 1
        ret = gsm0710_manager_send(GSM0710_CHANNEL_AT, (uint8_t *)at_cmd, cmd_len);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "MUX send failed: %s", esp_err_to_name(ret));
            xSemaphoreGive(g_at_cmd_mutex);
            return TT_AT_RESULT_ERROR;
        }
    } else {
        // AT Command Mode: Send directly via UART
        int bytes_written = uart_write_bytes(TT_UART_AT_PORT_NUM, at_cmd, cmd_len);
        if (bytes_written != cmd_len) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART send failed: written=%d, expected=%d", bytes_written, cmd_len);
            xSemaphoreGive(g_at_cmd_mutex);
            return TT_AT_RESULT_ERROR;
        }
    }

    xSemaphoreGive(g_at_cmd_mutex);
    return TT_AT_RESULT_OK;
}

/**
 * @brief Send AT Command to Tiantong Module (Universal Interface - Passthrough Mode)
 *
 * This function automatically detects the current mode (AT command mode or MUX mode)
 * and sends the AT command accordingly. It does NOT wait for response, suitable for
 * passthrough usage like BLE GATT.
 *
 * **This version includes state checking - only allows AT commands in AT mode or MUX mode.**
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
 * @note **State checking enabled** - only allows commands in AT mode or MUX mode.
 */
tt_at_result_t tt_module_send_at_cmd(const char *cmd)
{
    // This function is deprecated for external use - use tt_module_send_at_cmd_gatt() instead
    // For backward compatibility, return error (no conn_handle)
    SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "tt_module_send_at_cmd() called without conn_handle, use tt_module_send_at_cmd_gatt()");
    return TT_AT_RESULT_ERROR;
}

/**
 * @brief Send AT Command via GATT (Public API)
 *
 * This is the public API for BLE GATT AT service to send AT commands.
 */
tt_at_result_t tt_module_send_at_cmd_gatt(const char *cmd, uint16_t conn_handle)
{
    return tt_module_send_at_cmd_gatt_internal(cmd, conn_handle);
}

/**
 * @brief Send AT Command and Wait for Response (Local/Synchronous)
 *
 * This function sends an AT command and synchronously waits for the response.
 * Used for local commands (initialization, configuration, etc.).
 *
 * @param cmd AT command string (without "AT" prefix and CRLF)
 * @param response Buffer to store response
 * @param resp_len Response buffer size
 * @param timeout_ms Timeout in milliseconds
 * @return tt_at_result_t TT_AT_RESULT_OK on success, error code otherwise
 */
tt_at_result_t tt_module_send_at_cmd_wait(const char *cmd, char *response, size_t resp_len, int timeout_ms)
{
    // Local/Synchronous mode - for initialization and configuration
    return tt_module_send_at_cmd_local(cmd, response, resp_len, timeout_ms);
}

/**
 * @brief Send Data to Tiantong Module
 */
esp_err_t tt_module_send_data(int channel, const uint8_t *data, size_t len)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Check parameters
    if (data == NULL || len == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid data");
        return ESP_ERR_INVALID_ARG;
    }

    if (channel < 0 || channel > GSM0710_MAX_CHANNEL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid channel number: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // Send data via GSM0710 channel
    return gsm0710_manager_send(channel, data, len);
}

/**
 * @brief Notify tt_module about GATT disconnection
 *
 * This function should be called when a GATT connection is closed
 * to update the last active GATT connection handle.
 *
 * @param conn_handle The disconnected connection handle
 */
void tt_module_notify_gatt_disconnect(uint16_t conn_handle)
{
    if (g_at_context_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_at_context.last_active_gatt_conn == conn_handle) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Clearing last active GATT conn=%d", conn_handle);
            g_at_context.last_active_gatt_conn = 0;
        }
        xSemaphoreGive(g_at_context_mutex);
    }
}

/**
 * @brief Set active GATT connection for AT response routing
 *
 * This function sets the active GATT connection handle for routing
 * unsolicited AT notifications (like incoming calls, network status, etc.).
 * It should be called when a BLE GATT client connects.
 *
 * @param conn_handle BLE GATT connection handle
 */
void tt_module_set_active_gatt_conn(uint16_t conn_handle)
{
    if (g_at_context_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_at_context.last_active_gatt_conn = conn_handle;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Set active GATT conn=%d for unsolicited notifications", conn_handle);
        xSemaphoreGive(g_at_context_mutex);
    }
}

/**
 * @brief Receive Data from Tiantong Module
 */
esp_err_t tt_module_receive_data(int channel, uint8_t *data, size_t *len, int timeout_ms)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Check parameters
    if (data == NULL || len == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (channel < 0 || channel > GSM0710_MAX_CHANNEL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid channel number: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // Receive data via GSM0710 channel
    return gsm0710_manager_receive(channel, data, len, pdMS_TO_TICKS(timeout_ms));
}

tt_state_t tt_module_get_state(void)
{
    return g_tt_module.state;
}

/**
 * @brief Send AT Command (Local/Synchronous)
 *
 * This function sends an AT command and synchronously waits for the response.
 * Used for local commands (initialization, configuration, etc.).
 *
 * @param cmd AT command string (without "AT" prefix)
 * @param response Buffer to store response
 * @param resp_len Response buffer size
 * @param timeout_ms Timeout in milliseconds
 * @return tt_at_result_t Result code
 */
static tt_at_result_t tt_module_send_at_cmd_local(const char *cmd, char *response, size_t resp_len, int timeout_ms)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return TT_AT_RESULT_ERROR;
    }

    if (cmd == NULL || response == NULL || resp_len == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid parameters");
        return TT_AT_RESULT_ERROR;
    }

    // Determine current mode BEFORE sending command
    bool is_mux_mode = false;
    if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        is_mux_mode = (g_tt_module.uart_mode == TT_UART_MODE_MUX);
        xSemaphoreGive(g_tt_module.mode_mutex);
    }

    // In MUX mode, set local command waiting flag to route response to buffer
    if (is_mux_mode) {
        gsm0710_manager_set_local_command_waiting(true);
    }

    // Create completion semaphore
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (done_sem == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create completion semaphore");
        if (is_mux_mode) {
            gsm0710_manager_set_local_command_waiting(false);
        }
        return TT_AT_RESULT_ERROR;
    }

    // Take context mutex
    if (xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        if (is_mux_mode) {
            gsm0710_manager_set_local_command_waiting(false);
        }
        return TT_AT_RESULT_BUSY;
    }

    // Wait for GATT command to complete (if any)
    int retry_count = 0;
    while (g_at_context.target == TT_AT_TARGET_GATT && retry_count < 50) {
        xSemaphoreGive(g_at_context_mutex);
        vTaskDelay(pdMS_TO_TICKS(10));
        retry_count++;
        if (xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            vSemaphoreDelete(done_sem);
            if (is_mux_mode) {
                gsm0710_manager_set_local_command_waiting(false);
            }
            return TT_AT_RESULT_BUSY;
        }
    }

    if (g_at_context.target == TT_AT_TARGET_GATT) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "GATT command still active after waiting, aborting local command");
        xSemaphoreGive(g_at_context_mutex);
        vSemaphoreDelete(done_sem);
        if (is_mux_mode) {
            gsm0710_manager_set_local_command_waiting(false);
        }
        return TT_AT_RESULT_BUSY;
    }

    // Set LOCAL context
    g_at_context.target = TT_AT_TARGET_LOCAL;
    g_at_context.local.buffer = response;
    g_at_context.local.size = resp_len;
    g_at_context.local.received = 0;
    g_at_context.local.done_sem = done_sem;
    g_at_context.local.result = TT_AT_RESULT_ERROR;
    g_at_context.cmd_start_time = xTaskGetTickCount();
    response[0] = '\0';  // Clear response buffer

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> LOCAL CMD: %s (timeout=%dms, mode=%s)",
                   cmd, timeout_ms, is_mux_mode ? "MUX" : "AT");

    xSemaphoreGive(g_at_context_mutex);

    // Send command (tt_module_send_at_cmd_internal will take g_at_cmd_mutex)
    tt_at_result_t send_ret = tt_module_send_at_cmd_internal(cmd, false);

    if (send_ret != TT_AT_RESULT_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to send AT command");
        xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100));
        g_at_context.target = TT_AT_TARGET_NONE;
        xSemaphoreGive(g_at_context_mutex);
        vSemaphoreDelete(done_sem);
        if (is_mux_mode) {
            gsm0710_manager_set_local_command_waiting(false);
        }
        return send_ret;
    }

    // Wait for response (blocking)
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(done_sem, timeout_ticks) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "LOCAL CMD timeout: %s", cmd);

        // Timeout cleanup
        xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100));
        g_at_context.target = TT_AT_TARGET_NONE;
        xSemaphoreGive(g_at_context_mutex);
        vSemaphoreDelete(done_sem);
        if (is_mux_mode) {
            gsm0710_manager_set_local_command_waiting(false);
        }
        return TT_AT_RESULT_TIMEOUT;
    }

    vSemaphoreDelete(done_sem);

    // Clear MUX local command waiting flag
    if (is_mux_mode) {
        gsm0710_manager_set_local_command_waiting(false);
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> LOCAL CMD complete: %s -> %s", cmd, response);
    return TT_AT_RESULT_OK;
}

/**
 * @brief Send AT Command (GATT/Asynchronous) - Internal Implementation
 *
 * This function sends an AT command asynchronously. Response will be
 * routed to GATT callback. Used for BLE GATT commands.
 *
 * @param cmd AT command string (without "AT" prefix)
 * @param conn_handle GATT connection handle for response routing
 * @return tt_at_result_t Result code
 */
static tt_at_result_t tt_module_send_at_cmd_gatt_internal(const char *cmd, uint16_t conn_handle)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return TT_AT_RESULT_ERROR;
    }

    if (cmd == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid parameters");
        return TT_AT_RESULT_ERROR;
    }

    // Take context mutex
    if (xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to take context mutex for GATT command");
        return TT_AT_RESULT_BUSY;
    }

    // Check if local command is in progress
    if (g_at_context.target == TT_AT_TARGET_LOCAL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Local command in progress, GATT command deferred");
        xSemaphoreGive(g_at_context_mutex);
        return TT_AT_RESULT_BUSY;
    }

    // Set GATT context
    g_at_context.target = TT_AT_TARGET_GATT;
    g_at_context.gatt.conn_handle = conn_handle;
    g_at_context.last_active_gatt_conn = conn_handle;  // Remember for unsolicited notifications

    xSemaphoreGive(g_at_context_mutex);

    // Log the command being sent (single, concise log entry)
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT > %s", cmd);

    // Send command
    tt_at_result_t ret = tt_module_send_at_cmd_internal(cmd, true);
    if (ret != TT_AT_RESULT_OK) {
        // Clear context on error
        xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100));
        g_at_context.target = TT_AT_TARGET_NONE;
        xSemaphoreGive(g_at_context_mutex);
    }

    return ret;
}

/**
 * @brief Forward AT response/notification to upper layer
 *
 * This function forwards AT responses and module notifications to the registered
 * callback (e.g., JTAG UART, BLE GATT, etc.). This provides unified handling
 * for both AT command mode and MUX mode.
 *
 * Data is ONLY forwarded after ^SIMST: is detected, which indicates the module
 * has completed initialization and is ready to process AT commands. Startup log
 * data received before ^SIMST: is discarded.
 *
 * @param data Response/notification data
 * @param len Data length
 */
static void tt_module_forward_response_to_gatt(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> FORWARD: NULL or empty data");
        return;
    }

    // Only forward data after ^SIMST: is detected (module initialized and ready)
    // This prevents startup log data from being forwarded to GATT AT server
    if (!g_simst_detected) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                       ">>> FORWARD: Discarding %d bytes (module not ready, waiting for ^SIMST:)", len);
        return;
    }

    // Debug log - show what we're forwarding
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> FORWARD: TT module received %d bytes: %.*s", len, len, (char *)data);

    // Call registered callback if available
    if (g_data_callback != NULL) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> FORWARD: Calling g_data_callback %p", g_data_callback);
        g_data_callback(data, len, g_data_callback_user_data);
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> FORWARD: No g_data_callback registered, data dropped!");
    }
}

/**
 * @brief Unified AT Command Response Routing
 *
 * This is the core function that routes AT command responses based on the current
 * command context. It replaces the old g_at_cmd_waiting_resp mechanism.
 *
 * Routing logic:
 * - If context.target == LOCAL: Save response to local buffer (for synchronous local commands)
 * - If context.target == GATT: Forward response to GATT callback (for asynchronous GATT commands)
 * - If context.target == NONE: Response is an unsolicited notification (forward to GATT)
 *
 * @param data Response data received from module
 * @param len Data length
 * @param user_data Not used (for callback compatibility)
 */
void tt_module_route_at_response(const uint8_t *data, size_t len, void *user_data)
{
    (void)user_data;  // Unused

    if (data == NULL || len == 0) {
        return;
    }

    // Take context mutex to safely check and modify context
    if (xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to take context mutex for response routing");
        return;
    }

    // For unsolicited notifications (NONE context) and GATT commands, only process after module is ready
    // LOCAL commands (initialization) are allowed before g_simst_detected
    if (g_at_context.target == TT_AT_TARGET_NONE || g_at_context.target == TT_AT_TARGET_GATT) {
        if (!g_simst_detected) {
            xSemaphoreGive(g_at_context_mutex);
            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                           ">>> ROUTE: Discarding %d bytes (module not ready, waiting for ^SIMST:)", len);
            return;
        }
    }

    switch (g_at_context.target) {
        case TT_AT_TARGET_LOCAL: {
            // Local command (synchronous) - save to buffer
            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> ROUTE: Local command response (%d bytes)", len);

            if (g_at_context.local.buffer != NULL &&
                g_at_context.local.received < g_at_context.local.size) {

                // Calculate copy length (prevent buffer overflow, leave space for null terminator)
                size_t copy_len = len;
                size_t available = g_at_context.local.size - g_at_context.local.received - 1;  // -1 for null terminator
                if (copy_len > available) {
                    copy_len = available;
                }

                // Copy to local buffer
                memcpy(g_at_context.local.buffer + g_at_context.local.received, data, copy_len);
                g_at_context.local.received += copy_len;
                g_at_context.local.buffer[g_at_context.local.received] = '\0';  // Null terminate (safe now)

                SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> ROUTE: Saved %d bytes to local buffer (total=%zu): %.*s",
                               copy_len, g_at_context.local.received, (int)copy_len, (char*)data);
            }

            // Check if response is complete (contains OK or ERROR)
            if (strstr((char*)data, "OK") || strstr((char*)data, "ERROR")) {
                SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> ROUTE: Local command complete, signaling semaphore");

                // Set result
                g_at_context.local.result = (strstr((char*)data, "OK") != NULL) ?
                                           TT_AT_RESULT_OK : TT_AT_RESULT_ERROR;

                // Clear context (set to idle)
                g_at_context.target = TT_AT_TARGET_NONE;

                // Signal completion
                if (g_at_context.local.done_sem != NULL) {
                    xSemaphoreGive(g_at_context.local.done_sem);
                }
            }
            break;
        }

        case TT_AT_TARGET_GATT: {
            // GATT command (asynchronous) - forward to GATT callback
            // Log response (trimmed for readability)
            size_t log_len = (len > 100) ? 100 : len;  // Truncate long responses
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT < %.*s", (int)log_len, (char*)data);

            // Forward to GATT via spp_at_server (use non-blocking async version)
            extern int spp_at_server_send_response_async(uint16_t conn_handle, const uint8_t *data, uint16_t len);
            spp_at_server_send_response_async(g_at_context.gatt.conn_handle, data, len);

            // Check if response is complete (contains OK or ERROR)
            if (strstr((char*)data, "OK") || strstr((char*)data, "ERROR")) {
                // Clear context (set to idle)
                g_at_context.target = TT_AT_TARGET_NONE;
                memset(&g_at_context.gatt, 0, sizeof(g_at_context.gatt));
            }
            break;
        }

        case TT_AT_TARGET_NONE:
        default: {
            // No active command context - this is an unsolicited notification
            // Forward to GATT directly for notifications like incoming calls, network status, etc.
            SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> ROUTE: Unsolicited notification (%d bytes)", len);

            // Forward to GATT if we have an active connection (use non-blocking async version)
            if (g_at_context.last_active_gatt_conn != 0) {
                extern int spp_at_server_send_response_async(uint16_t conn_handle, const uint8_t *data, uint16_t len);
                spp_at_server_send_response_async(g_at_context.last_active_gatt_conn, data, len);
                SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> ROUTE: Sent notification to GATT conn=%d", g_at_context.last_active_gatt_conn);
            } else {
                // Also try callback for MUX mode or other use cases
                if (g_data_callback != NULL) {
                    SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> ROUTE: Forwarding to callback (no GATT connection)");
                    g_data_callback(data, len, g_data_callback_user_data);
                } else {
                    SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, ">>> ROUTE: No active GATT connection, notification dropped");
                }
            }
            break;
        }
    }

    xSemaphoreGive(g_at_context_mutex);
}

/**
 * @brief Write data to AT UART port
 */
int tt_module_uart_write(const void *data, size_t len)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return -1;
    }

    if (data == NULL || len == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid parameters");
        return -1;
    }

    return uart_write_bytes(TT_UART_AT_PORT_NUM, (const char *)data, len);
}

/**
 * @brief Read data from AT UART port
 */
int tt_module_uart_read(void *data, size_t len, int timeout_ms)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return -1;
    }

    if (data == NULL || len == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid parameters");
        return -1;
    }

    return uart_read_bytes(TT_UART_AT_PORT_NUM, data, len, pdMS_TO_TICKS(timeout_ms));
}

/**
 * @brief Check if AT UART port is initialized
 */
bool tt_module_uart_is_initialized(void)
{
    return g_tt_module.initialized;
}

/**
 * @brief Get Next Event from Tiantong Module
 */
esp_err_t tt_module_get_event(tt_event_t *event, int timeout_ms)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_tt_module.event_queue == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Event queue not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (event == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Invalid event pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Receive event from queue
    if (xQueueReceive(g_tt_module.event_queue, event, pdMS_TO_TICKS(timeout_ms)) == pdPASS) {
        return ESP_OK;
    } else {
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief Reset Tiantong Module State to AT Command Mode
 * 
 * This function resets the module state and UART mode back to AT command mode
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_reset_state(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Reset SIMST detection flag
    g_simst_detected = false;
    g_tt_module_ready = false;  // Module needs to re-announce ready state

    // Update module state to initializing (need to re-detect SIMST)
    set_tt_state(TT_STATE_INITIALIZING);

    // Update UART mode to AT mode
    // The uart_at_rx_task will detect this change and automatically continue working
    if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_tt_module.uart_mode = TT_UART_MODE_AT;
        xSemaphoreGive(g_tt_module.mode_mutex);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART mode changed to AT (existing task will adapt)");
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module state reset to AT command mode");
    return ESP_OK;
}

/**
 * @brief Restart AT Command Task
 * 
 * This function restarts the AT command task after MUX mode failure
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_module_restart_at_task(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Check if AT task is already running
    if (g_tt_module.uart_at_task_handle != NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT task is already running");
        return ESP_OK;
    }

    // Create UART AT RX Task
    if (xTaskCreate(uart_at_rx_task, "uart_at_rx_task", 4096, NULL, 10, &g_tt_module.uart_at_task_handle) != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create UART AT RX task");
        return ESP_FAIL;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AT Command Task restarted successfully");
    return ESP_OK;
}

/**
 * @brief Power On Tiantong Module
 */
esp_err_t tt_module_power_on(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Use hardware layer to power on the module
    esp_err_t ret = tt_hw_power_on();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to power on Tiantong Module via hardware layer: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Power Off Tiantong Module
 */
esp_err_t tt_module_power_off(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Use hardware layer to power off the module
    esp_err_t ret = tt_hw_power_off();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to power off Tiantong Module via hardware layer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update status to USER_OFF with normal shutdown reason
    set_tt_state(TT_STATE_USER_OFF);
    g_tt_error_code = TT_ERROR_NONE;

    return ESP_OK;
}

/**
 * @brief Shutdown TT Module with reason code
 */
esp_err_t tt_module_shutdown_with_reason(uint8_t reason)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Use hardware layer to power off the module
    esp_err_t ret = tt_hw_power_off();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to power off Tiantong Module via hardware layer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update status to USER_OFF with specified reason
    set_tt_state(TT_STATE_USER_OFF);
    g_tt_error_code = reason;

    return ESP_OK;
}

/**
 * @brief Reset Tiantong Module
 */
esp_err_t tt_module_reset(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // First try software reset via AT command
    char response[TT_AT_RESP_BUF_SIZE];
    tt_at_result_t result = tt_module_send_at_cmd_wait("+CFUN=1,1", response, sizeof(response), TT_AT_CMD_TIMEOUT_MS);

    if (result != TT_AT_RESULT_OK) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Software reset failed, falling back to hardware reset");

        // If software reset fails, use hardware reset
        esp_err_t ret = tt_hw_reset();
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to reset module via hardware layer: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // Reset ready state flags - module needs to re-announce ready state after reset
    g_simst_detected = false;
    g_tt_module_ready = false;

    return ESP_OK;
}

/**
 * @brief Cleanup Tiantong Module State
 *
 * This function clears all module state including SIMST detection, MUX mode,
 * and data callbacks. It should be called when the module is powered off
 * or needs to be fully reinitialized.
 */
esp_err_t tt_module_cleanup(void)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Cleaning up Tiantong Module state...");

    // Clear SIMST detection flags
    g_simst_detected = false;
    g_tt_module_ready = false;

    // Clear data callback
    g_data_callback = NULL;
    g_data_callback_user_data = NULL;

    // Reset module state to UNINITIALIZED
    if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        set_tt_state(TT_STATE_USER_OFF);
        g_tt_module.uart_mode = TT_UART_MODE_AT;
        xSemaphoreGive(g_tt_module.mode_mutex);
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to acquire mode mutex during cleanup");
    }

    // Clear AT command context
    if (g_at_context_mutex != NULL) {
        xSemaphoreTake(g_at_context_mutex, pdMS_TO_TICKS(100));
        g_at_context.target = TT_AT_TARGET_NONE;
        xSemaphoreGive(g_at_context_mutex);
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module state cleaned up");

    return ESP_OK;
}

/**
 * @brief Enter OTA Update Mode
 */
esp_err_t tt_module_enter_ota_mode(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Acquire mutex to prevent concurrent operations
    if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to acquire mode mutex");
        return ESP_ERR_TIMEOUT;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Entering OTA mode...");

    // Stop normal operations
    // 1. Stop MUX mode if active
    if (g_tt_module.state == TT_STATE_WORKING) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Stopping MUX mode for OTA...");
        gsm0710_manager_stop();
        gsm0710_manager_deinit();
    }

    // 2. Stop AT command task
    if (g_tt_module.uart_at_task_handle != NULL) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Stopping AT command task...");
        vTaskDelete(g_tt_module.uart_at_task_handle);
        g_tt_module.uart_at_task_handle = NULL;
    }

    // 3. Reset modem to AT command mode
    tt_module_reset();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 4. Set state to OTA mode
    set_tt_state(TT_STATE_UPDATING);
    g_tt_module.uart_mode = TT_UART_MODE_AT;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Entered OTA mode successfully");

    // Release mutex before returning
    xSemaphoreGive(g_tt_module.mode_mutex);
    return ESP_OK;
}

/**
 * @brief Exit OTA Update Mode
 */
esp_err_t tt_module_exit_ota_mode(void)
{
    if (!g_tt_module.initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Acquire mutex
    if (xSemaphoreTake(g_tt_module.mode_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to acquire mode mutex");
        return ESP_ERR_TIMEOUT;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Exiting OTA mode...");

    // Reset modem after OTA
    tt_module_reset();
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Restore normal operation - will become WORKING after SIMST detected and MUX init
    set_tt_state(TT_STATE_INITIALIZING);
    g_tt_module.uart_mode = TT_UART_MODE_AT;

    // Restart AT command task
    xTaskCreate(uart_at_rx_task, "uart_at_task", 4096, NULL, 5, &g_tt_module.uart_at_task_handle);

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Exited OTA mode, restored to AT command mode");
    xSemaphoreGive(g_tt_module.mode_mutex);
    return ESP_OK;
}

/**
 * @brief Check if module is in OTA mode
 */
bool tt_module_is_in_ota_mode(void)
{
    return (g_tt_module.state == TT_STATE_UPDATING);
}

/**
 * @brief Register Data Receive Callback
 */
esp_err_t tt_module_register_data_callback(tt_data_received_cb_t callback, void *user_data)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Registering data callback: %p, user_data: %p", callback, user_data);

    g_data_callback = callback;
    g_data_callback_user_data = user_data;

    return ESP_OK;
}

/**
 * @brief Switch UART baud rate to high speed (921600)
 *
 * This function implements the automatic baud rate switching procedure:
 * 1. Send AT^BAUD=0 command (current baud rate)
 * 2. Wait for OK response
 * 3. Switch local UART to 921600
 * 4. Send AT command to verify (new baud rate)
 * 5. If no OK within 5 seconds, retry
 * 6. After 3 failed attempts, keep 115200 and log error
 *
 * @return ESP_OK on success, ESP_FAIL on failure (after all retries)
 */
static esp_err_t tt_switch_to_high_baud_rate(void)
{
    esp_err_t ret;
    char response[128];
    const char *baud_cmd = "^BAUD=0";
    const char *test_cmd = "AT";  // Simple test command

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "========== Starting baud rate switch to 921600 ==========");

    // Step 1: Send AT^BAUD=0 command (at current baud rate: 115200)
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 1] Sending AT%s command at 115200...", baud_cmd);
    ret = tt_module_send_at_cmd_wait(baud_cmd, response, sizeof(response),
                                     TT_BAUD_SWITCH_TIMEOUT_MS);
    if (ret != ESP_OK || !strstr(response, "OK")) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 1] Failed to get OK for AT^BAUD=0");
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 1] Response: %s", response);
        return ESP_FAIL;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 1] Received OK for AT^BAUD=0");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 2] Switching local UART to 921600...");

    // Step 2: Switch local UART to 921600
    ret = tt_hw_set_uart1_baud(TT_UART_AT_BAUD_HIGH);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 2] Failed to switch local UART to 921600");
        return ESP_FAIL;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 2] Local UART switched to 921600, waiting for stabilization...");
    vTaskDelay(pdMS_TO_TICKS(500));  // Increased delay to 500ms

    // Clear UART RX buffer to ensure no stale data
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 2] Clearing UART RX buffer...");
    uint8_t dummy_buf[128];
    uart_read_bytes(TT_UART_AT_PORT_NUM, dummy_buf, sizeof(dummy_buf), pdMS_TO_TICKS(100));

    // Ensure uart_at_rx_task is fully resumed before verification
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 2] Verifying uart_at_rx_task is ready...");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 3: Send AT command to verify connection at new baud rate
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 3] Sending AT command at 921600 to verify connection...");
    memset(response, 0, sizeof(response));
    ret = tt_module_send_at_cmd_wait(test_cmd, response, sizeof(response),
                                     TT_BAUD_SWITCH_TIMEOUT_MS);

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 3] tt_module_send_at_cmd_wait returned: %d", ret);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 3] Response received: %s", response);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[Step 3] Response length: %d", strlen(response));

    if (ret == ESP_OK && strstr(response, "OK")) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "========== Baud rate successfully switched to 921600! ==========");
        return ESP_OK;
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "========== No OK received at 921600, verification failed ==========");
        return ESP_FAIL;
    }
}

/* ========== TT Module Status Management ========== */

/* ========== Simplified State Management ========== */

/**
 * @brief Check if user control is allowed
 */
bool tt_module_is_user_control_allowed(void)
{
    tt_state_t state = g_tt_module.state;

    switch (state) {
        case TT_STATE_LOW_BATTERY_OFF:
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Cannot control: module is in LOW_BATTERY_OFF state");
            return false;
        case TT_STATE_UPDATING:
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Cannot control: module is UPDATING");
            return false;
        case TT_STATE_INITIALIZING:
            SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Cannot control: module is INITIALIZING");
            return false;
        case TT_STATE_HARDWARE_FAULT:
        case TT_STATE_USER_OFF:
        case TT_STATE_WORKING:
        default:
            return true;
    }
}

/**
 * @brief Set TT module state (simplified)
 */
static void set_tt_state(tt_state_t new_state)
{
    tt_state_t old_state = g_tt_module.state;
    g_tt_module.state = new_state;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT State: %d -> %d", old_state, new_state);
}

/* ========== TT Module Full Power Control APIs ========== */

esp_err_t tt_module_full_startup(void)
{
    if (g_tt_module_powered) {
        SYS_LOGW(TAG, "TT module already powered");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI(TAG, "=== TT Module Full Startup ===");

    esp_err_t ret;

    /* Step 1: Initialize (create queues, semaphores, etc.) */
    ret = tt_module_init(10);
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Failed to init TT module: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    SYS_LOGI(TAG, "[1/4] Module initialized");

    /* Step 2: Start communication (includes power on) */
    ret = tt_module_start();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Failed to start TT module: %s", esp_err_to_name(ret));
        tt_module_deinit();
        return ESP_FAIL;
    }
    SYS_LOGI(TAG, "[2/3] Communication started (includes power on)");

    /* Update power flag */
    g_tt_module_powered = true;

    /* Log final status - state will be updated to WORKING by tt_mux_init_task when fully initialized */
    tt_state_t state = tt_module_get_state();

    SYS_LOGI(TAG, "=== TT Module Startup Complete (State: %d) ===", state);
    return ESP_OK;
}

esp_err_t tt_module_full_shutdown(void)
{
    if (!g_tt_module_powered) {
        SYS_LOGW(TAG, "TT module already powered off");
        return ESP_ERR_INVALID_STATE;
    }

    /* Check if OTA is in progress */
    if (tt_module_is_in_ota_mode()) {
        SYS_LOGE(TAG, "Cannot shutdown during OTA update");
        return ESP_ERR_NOT_ALLOWED;
    }

    SYS_LOGI(TAG, "=== TT Module Full Shutdown ===");

    /* Step 1: Stop communication (includes power off) */
    tt_module_stop();
    SYS_LOGI(TAG, "[1/3] Communication stopped (includes power off)");

    /* Step 2: Cleanup state */
    tt_module_cleanup();
    SYS_LOGI(TAG, "[2/3] State cleaned");

    /* Step 3: Deinitialize */
    tt_module_deinit();
    SYS_LOGI(TAG, "[3/3] Module deinitialized");

    SYS_LOGI(TAG, "=== TT Module Shutdown Complete ===");
    return ESP_OK;
}

/* ========== User Power Control APIs ========== */

esp_err_t tt_module_user_power_on(void)
{
    esp_err_t ret;

    // Get battery voltage for logging (no blocking - phone call has highest priority)
    uint16_t voltage_mv = 0;
    bool voltage_read_ok = false;
    bq27220_handle_t battery_handle = power_manage_get_bq27220_handle();
    if (battery_handle != NULL) {
        voltage_mv = bq27220_get_voltage(battery_handle);
        voltage_read_ok = true;
    }

    // Step 1: Check if user control is allowed
    if (!tt_module_is_user_control_allowed()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Step 2: Clear NVS user manual off flag
    ret = user_params_set_tt_manual_off(false);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to clear user manual off flag: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 3: Check if already powered
    if (g_tt_module_powered) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT module already powered");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "=== TT Module User Power On ===");

    // Step 4: Initialize (create queues, semaphores, etc.) - same as full_startup
    ret = tt_module_init(10);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to init TT module: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // Step 5: Start communication (includes power on) - same as full_startup
    ret = tt_module_start();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to start TT module: %s", esp_err_to_name(ret));
        tt_module_deinit();
        return ESP_FAIL;
    }

    // Step 6: Update power flag
    // State remains INITIALIZING (set by tt_module_start), will become WORKING after MUX init completes
    g_tt_module_powered = true;

    // Notify sleep manager
    extern void sleep_manager_notify_tt_powered_on(void);
    sleep_manager_notify_tt_powered_on();

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "=== TT Module User Power On Complete (battery: %umV%s) ===",
        voltage_read_ok ? voltage_mv : 0, voltage_read_ok ? "" : " (unread)");

    return ESP_OK;
}

esp_err_t tt_module_user_power_off(void)
{
    // Note: No battery check needed - user should be able to power off anytime

    // Step 1: Check if user control is allowed
    if (!tt_module_is_user_control_allowed()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Step 2: Check if already powered off
    if (!g_tt_module_powered) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "TT module already powered off");
        return ESP_OK;
    }

    // Step 3: Set NVS user manual off flag (before shutdown)
    esp_err_t ret = user_params_set_tt_manual_off(true);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to set user manual off flag: %s", esp_err_to_name(ret));
        return ret;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "=== TT Module User Power Off ===");

    // Step 4: Stop communication (includes power off) - same as full_shutdown
    tt_module_stop();
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[1/3] Communication stopped");

    // Step 5: Cleanup state - same as full_shutdown
    tt_module_cleanup();
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[2/3] State cleaned");

    // Step 6: Deinitialize - same as full_shutdown
    tt_module_deinit();
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "[3/3] Module deinitialized");

    // Step 7: Clear power flag and ensure final state
    g_tt_module_powered = false;
    set_tt_state(TT_STATE_USER_OFF);  // Ensure final state is USER_OFF

    // Notify sleep manager
    extern void sleep_manager_notify_tt_powered_off(void);
    sleep_manager_notify_tt_powered_off();

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "=== TT Module User Power Off Complete ===");

    return ESP_OK;
}

bool tt_module_is_powered(void)
{
    return g_tt_module_powered;
}

/* ========== TT Module Status Query APIs ========== */

tt_module_status_t tt_module_get_status(void)
{
    // Map state to status for backward compatibility
    tt_state_t state = g_tt_module.state;

    switch (state) {
        case TT_STATE_USER_OFF:
        case TT_STATE_LOW_BATTERY_OFF:
            return TT_MODULE_STATUS_POWER_OFF;
        case TT_STATE_INITIALIZING:
            return TT_MODULE_STATUS_INITIALIZING;
        case TT_STATE_WORKING:
            return TT_MODULE_STATUS_WORKING;
        case TT_STATE_UPDATING:
            return TT_MODULE_STATUS_OTA_UPDATING;
        case TT_STATE_HARDWARE_FAULT:
            return TT_MODULE_STATUS_ERROR;
        default:
            return TT_MODULE_STATUS_POWER_OFF;
    }
}

esp_err_t tt_module_get_status_info(tt_status_info_t *info)
{
    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(info, 0, sizeof(tt_status_info_t));

    // Get state
    info->state = g_tt_module.state;

    // Get battery voltage
    bq27220_handle_t battery_handle = power_manage_get_bq27220_handle();
    if (battery_handle != NULL) {
        info->voltage_mv = bq27220_get_voltage(battery_handle);
    }

    // Get error code (only when state is HARDWARE_FAULT)
    if (info->state == TT_STATE_HARDWARE_FAULT) {
        info->error_code = g_tt_error_code;
    }

    // Note: Working sub-states (SIM status, network registration) are managed by client
    // Client should query these via AT commands when needed:
    //   - SIM status: AT+CPIN?
    //   - Network registration: AT+CREG?

    return ESP_OK;
}
