/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef SYSLOG_H
#define SYSLOG_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file syslog.h
 * @brief Unified system logging interface with module control and GATT output
 *
 * Features:
 * - Modular log control (level, enable/disable, GATT output per module)
 * - Dual output support (ESP LOG + GATT LOG)
 * - 2KB GATT log buffer with smooth bursting
 * - Voice GATT conflict avoidance (block + clear buffer)
 * - Remote configuration support
 * - Thread-safe with mutex protection
 */

// ============================================================
// Constants and Configuration
// ============================================================

// Maximum log message length
#define SYS_LOG_MAX_MSG_LEN  256

// GATT log buffer configuration
#define SYS_LOG_GATT_BUF_SIZE     2048   // 2KB ring buffer
#define SYS_LOG_GATT_CHUNK_SIZE    120    // Max chunk per send (MTU=128, minus ATT overhead)
#define SYS_LOG_GATT_SEND_STACK    3072   // Send task stack size
#define SYS_LOG_GATT_SEND_PRIO     5      // Send task priority

// ============================================================
// Data Types
// ============================================================

/**
 * @brief Log level enumeration
 */
typedef enum {
    SYS_LOG_LEVEL_DEBUG = 0,    ///< Debug information
    SYS_LOG_LEVEL_INFO,         ///< Informational
    SYS_LOG_LEVEL_WARN,         ///< Warning
    SYS_LOG_LEVEL_ERROR,        ///< Error
    SYS_LOG_LEVEL_FATAL,        ///< Fatal error
    SYS_LOG_LEVEL_NONE          ///< Logging disabled
} syslog_level_t;

/**
 * @brief Module ID enumeration
 */
typedef enum {
    SYS_LOG_MODULE_MAIN = 0x01,         ///< Main application module
    SYS_LOG_MODULE_BLE_GATT,            ///< BLE GATT module
    SYS_LOG_MODULE_AUDIO_PROC,          ///< Audio processing module
    SYS_LOG_MODULE_VOICE_PACKET,        ///< Voice packet handling module
    SYS_LOG_MODULE_AT_CMD,              ///< AT command module
    SYS_LOG_MODULE_SPP_AT,              ///< SPP AT pass-through module
    SYS_LOG_MODULE_TT_MODULE,           ///< Tiantong module
    SYS_LOG_MODULE_OTA,                 ///< OTA update module
    SYS_LOG_MODULE_ALL = 0xFF           ///< All modules
} syslog_module_t;

/**
 * @brief Module log configuration structure
 */
typedef struct {
    syslog_module_t module;     ///< Module ID
    syslog_level_t level;       ///< Log level for the module
    bool enabled;               ///< Whether logging is enabled
    bool gatt_output;           ///< Whether to output to GATT (per-module)
} syslog_module_config_t;

/**
 * @brief GATT log blocking state
 */
typedef enum {
    SYS_LOG_GATT_STATE_IDLE = 0,        ///< Idle, output allowed
    SYS_LOG_GATT_STATE_BLOCKED          ///< Blocked, output disabled
} syslog_gatt_state_t;

/**
 * @brief GATT blocking reason
 */
typedef enum {
    SYS_LOG_GATT_BLOCK_VOICE_ACTIVE = 0x01,  ///< Voice GATT is active
    SYS_LOG_GATT_BLOCK_MANUAL = 0x02,         ///< Manually disabled
    SYS_LOG_GATT_BLOCK_BANDWIDTH = 0x03       ///< Bandwidth limitation
} syslog_gatt_block_reason_t;

/**
 * @brief Remote command types
 */
typedef enum {
    // Basic configuration commands
    SYS_LOG_CMD_SET_GLOBAL_LEVEL = 0x01,    ///< Set global log level
    SYS_LOG_CMD_GET_GLOBAL_LEVEL,           ///< Get global log level

    // Module configuration commands
    SYS_LOG_CMD_SET_MODULE_CONFIG = 0x10,   ///< Set module config (level+enabled+gatt)
    SYS_LOG_CMD_GET_MODULE_CONFIG,          ///< Get module config
    SYS_LOG_CMD_GET_ALL_CONFIGS,            ///< Get all module configs

    // Module control commands
    SYS_LOG_CMD_ENABLE_MODULE = 0x20,       ///< Enable module logging
    SYS_LOG_CMD_DISABLE_MODULE,             ///< Disable module logging

    // GATT related commands
    SYS_LOG_CMD_SET_GATT_GLOBAL = 0x30,     ///< Set GATT global switch
    SYS_LOG_CMD_GET_GATT_GLOBAL,            ///< Get GATT global switch
    SYS_LOG_CMD_GET_GATT_STATE,             ///< Get GATT state
    SYS_LOG_CMD_GET_GATT_STATS,             ///< Get GATT buffer statistics

    // Batch operation commands
    SYS_LOG_CMD_SET_ALL_LEVELS = 0x40,      ///< Set all modules level
    SYS_LOG_CMD_ENABLE_ALL_GATT,            ///< Enable all modules GATT
    SYS_LOG_CMD_DISABLE_ALL_GATT,           ///< Disable all modules GATT
} syslog_cmd_type_t;

// ============================================================
// Initialization and Cleanup
// ============================================================

/**
 * @brief Initialize the syslog module
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t syslog_init(void);

/**
 * @brief Deinitialize the syslog module
 *
 * @return ESP_OK on success
 */
esp_err_t syslog_deinit(void);

// ============================================================
// Control Interface - For System Server
// ============================================================

/**
 * @brief Set global log level
 *
 * @param level New log level
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid level
 */
esp_err_t syslog_set_global_level(syslog_level_t level);

/**
 * @brief Get current global log level
 *
 * @return Current global log level
 */
syslog_level_t syslog_get_global_level(void);

/**
 * @brief Set log configuration for a specific module
 *
 * @param module Module ID (SYS_LOG_MODULE_ALL for all modules)
 * @param level Log level for the module
 * @param enabled Whether logging is enabled
 * @param gatt_output Whether to output to GATT
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid arguments
 */
esp_err_t syslog_set_module_config(syslog_module_t module,
                                   syslog_level_t level,
                                   bool enabled,
                                   bool gatt_output);

/**
 * @brief Get log configuration for a specific module
 *
 * @param module Module ID
 * @param config Pointer to store configuration
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid arguments
 */
esp_err_t syslog_get_module_config(syslog_module_t module,
                                   syslog_module_config_t *config);

/**
 * @brief Get all module configurations
 *
 * @param configs Configuration array pointer (must be pre-allocated)
 * @param max_count Maximum array capacity
 * @param actual_count Pointer to store actual module count
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on invalid arguments
 */
esp_err_t syslog_get_all_configs(syslog_module_config_t *configs,
                                 int max_count,
                                 int *actual_count);

// ============================================================
// GATT Connection and Conflict Avoidance
// ============================================================

/**
 * @brief Set BLE GATT connection handle for log output
 *
 * @param conn_handle BLE connection handle
 */
void syslog_set_gatt_conn_handle(uint16_t conn_handle);

/**
 * @brief Get current BLE GATT connection handle
 *
 * @return Current connection handle (0 if no active connection)
 */
uint16_t syslog_get_gatt_conn_handle(void);

/**
 * @brief Clear BLE GATT connection handle (when device disconnects)
 */
void syslog_clear_gatt_conn_handle(void);

/**
 * @brief Set GATT log global enable switch
 *
 * @param enabled true to enable, false to disable
 */
void syslog_set_gatt_global_enabled(bool enabled);

/**
 * @brief Check if GATT log is globally enabled
 *
 * @return true if enabled, false otherwise
 */
bool syslog_is_gatt_global_enabled(void);

/**
 * @brief Block GATT log output and clear buffer
 *
 * This function blocks GATT log output and clears the buffer.
 * Should be called when Voice GATT becomes active.
 *
 * @param reason Blocking reason
 */
void syslog_block_gatt_output(syslog_gatt_block_reason_t reason);

/**
 * @brief Unblock GATT log output
 *
 * This function unblocks GATT log output.
 * Should be called when Voice GATT stops.
 *
 * @param reason Blocking reason to clear
 */
void syslog_unblock_gatt_output(syslog_gatt_block_reason_t reason);

/**
 * @brief Check if GATT log is blocked
 *
 * @return true if blocked, false otherwise
 */
bool syslog_is_gatt_blocked(void);

/**
 * @brief Get current GATT log blocking state
 *
 * @return Current blocking state
 */
syslog_gatt_state_t syslog_get_gatt_state(void);

/**
 * @brief Get GATT buffer statistics
 *
 * @param used Pointer to store used bytes (can be NULL)
 * @param free Pointer to store free bytes (can be NULL)
 * @param dropped Pointer to store dropped bytes count (can be NULL)
 */
void syslog_get_gatt_buffer_stats(size_t *used, size_t *free, size_t *dropped);

// ============================================================
// Remote Configuration
// ============================================================

/**
 * @brief Process remote configuration command
 *
 * @param cmd_data Command data buffer
 * @param cmd_len Command data length
 * @param response Response buffer
 * @param resp_len Response length (input: buffer size, output: actual length)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t syslog_process_remote_config(const uint8_t *cmd_data,
                                       size_t cmd_len,
                                       uint8_t *response,
                                       size_t *resp_len);

// ============================================================
// Output Interface - For Module Usage
// ============================================================

/**
 * @brief System log interface that sends logs to both ESP_LOG and GATT_LOG
 *
 * These macros provide a unified logging interface that:
 * 1. Logs messages to the ESP32 console via ESP_LOGx macros
 * 2. Sends log notifications over BLE GATT (if enabled and not blocked)
 * 3. Respects the log level configuration from syslog module
 *
 * Usage: SYS_LOGE(TAG, "Error message: %d", error_code);
 */

// Generic log macros (uses MAIN module)
#define SYS_LOGD(tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_DEBUG)) { \
            ESP_LOGD(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define SYS_LOGI(tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_INFO)) { \
            ESP_LOGI(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_INFO, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define SYS_LOGW(tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_WARN)) { \
            ESP_LOGW(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_WARN, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define SYS_LOGE(tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_ERROR)) { \
            ESP_LOGE(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define SYS_LOGF(tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_FATAL)) { \
            ESP_LOGE(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(SYS_LOG_MODULE_MAIN, SYS_LOG_LEVEL_FATAL, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

// Module-specific system log macros
#define SYS_LOGD_MODULE(module, tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(module, SYS_LOG_LEVEL_DEBUG)) { \
            ESP_LOGD(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(module, SYS_LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define SYS_LOGI_MODULE(module, tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(module, SYS_LOG_LEVEL_INFO)) { \
            ESP_LOGI(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(module, SYS_LOG_LEVEL_INFO, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define SYS_LOGW_MODULE(module, tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(module, SYS_LOG_LEVEL_WARN)) { \
            ESP_LOGW(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(module, SYS_LOG_LEVEL_WARN, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define SYS_LOGE_MODULE(module, tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(module, SYS_LOG_LEVEL_ERROR)) { \
            ESP_LOGE(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(module, SYS_LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define SYS_LOGF_MODULE(module, tag, fmt, ...) \
    do { \
        if (syslog_is_output_allowed(module, SYS_LOG_LEVEL_FATAL)) { \
            ESP_LOGE(tag, fmt, ##__VA_ARGS__); \
            syslog_output_to_gatt_if_allowed(module, SYS_LOG_LEVEL_FATAL, tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

// ============================================================
// Internal Functions (not for direct use)
// ============================================================

/**
 * @brief Check if logging is allowed for a module and level
 *
 * @param module Module ID
 * @param level Log level
 * @return true if logging is allowed, false otherwise
 */
bool syslog_is_output_allowed(syslog_module_t module, syslog_level_t level);

/**
 * @brief Output to GATT if conditions are met (internal use)
 *
 * @param module Module ID
 * @param level Log level
 * @param tag Log tag
 * @param format Format string
 */
void syslog_output_to_gatt_if_allowed(syslog_module_t module,
                                       syslog_level_t level,
                                       const char *tag,
                                       const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* SYSLOG_H */
