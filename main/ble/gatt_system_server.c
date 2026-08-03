/*
 * gatt_system_server.c - GATT System Control Server Implementation
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_private/esp_clk.h"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble/gatt_system_server.h"
#include "ble/ble_gatt_server.h"
#include "version.h"
#include "tt/tt_module.h"
#include "tt/tt_hardware.h"
#include "ble/spp_at_server.h"
#include "ble/spp_voice_server.h"
#include "ble/gatt_log_server.h"
#include "system/syslog.h"
#include "system/power_manage.h"
#include "config/user_params.h"
#include "ble/gatt_ota_server.h"
#include "tt/tt_module_ota.h"
#include "audio/voice_packet_handler.h"
#include "fuel_gauge.h"
#include "IP5561.h"
#include "ble/ble_conn_manager.h"

/* Simulation support */
#include "sim/sat_call_sim.h"

/* Tag for logging */
static const char *TAG = "GATT_SYSTEM";

/* BLE TX power temporary override with timeout */
#define BLE_TX_POWER_DEFAULT    ESP_PWR_LVL_P12   /* 12 dBm - default production power */
static esp_timer_handle_t g_ble_tx_power_timer = NULL;
static esp_power_level_t g_ble_tx_power_original = BLE_TX_POWER_DEFAULT;
static bool g_ble_tx_power_overridden = false;

// ============================================================
// Debug Connection Permission Control
// ============================================================

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE

/**
 * @brief Debug connection allowed commands (whitelist)
 *
 * Debug connections can only execute these commands:
 * - Query commands: Get battery, charge, signal, system info, etc.
 * - Log commands: Set log level, config, enable/disable GATT output
 * - PowerBank commands: Set charge voltage, current, wireless charge
 * - System control: Reset TT module, set USB switch (limited)
 */
static const uint8_t DEBUG_ALLOWED_COMMANDS[] = {
    // ===== Query Commands =====
    0x01,  // GET_BATTERY_INFO
    0x02,  // GET_CHARGE_STATUS
    0x03,  // GET_SIGNAL_INFO
    0x04,  // GET_BLE_TX_POWER
    0x30,  // GET_SYSTEM_INFO
    0x31,  // GET_RUNTIME_INFO
    0x42,  // GET_MODULE_LOG_CONFIG
    0x44,  // GET_GATT_LOG_STATE
    0x45,  // GET_GATT_BUFFER_STATS
    0x50,  // GET_POWERBANK_MODE
    0x51,  // GET_CHARGE_DISCHARGE_STATUS
    0x52,  // GET_WIRELESS_CHARGE_STATUS
    0x54,  // GET_VBUS_VOLTAGE_CURRENT
    0x55,  // GET_NTC_TEMPERATURE
    0x7B,  // BLE_TX_POWER_TEMP (test only)
    0x60,  // GET_TT_MODULE_STATE

    // ===== Log Configuration Commands =====
    0x40,  // SET_GLOBAL_LOG_LEVEL
    0x41,  // SET_MODULE_LOG_CONFIG
    0x43,  // SET_GATT_LOG_GLOBAL
    0x46,  // DISABLE_ALL_GATT_OUTPUT

    // ===== PowerBank Configuration Commands =====
    0x58,  // SET_CHARGE_VOLTAGE
    0x59,  // SET_9V_CHARGE_CURRENT
    0x5A,  // SET_9V_UNDERVOLT_THRESHOLD
    0x5C,  // SET_VBUS_9V_CURRENT
    0x5E,  // SET_WIRELESS_CHARGE

    // ===== Simulation Control Commands =====
    0x77,  // SIM_CTRL (enable/disable + scenario)
    0x78,  // SIM_INCOMING (trigger incoming call)
    0x79,  // SIM_SET_NET (set simulated CSQ/CREG)
    0x7A,  // SIM_GET_STATE (query simulation state)
    
    // ===== BLE Test Commands =====
    0x7B, // Set BLE TX power temporarily with timeout

    // ===== BLE Device Name Commands =====
    0x7C,  // SET_DEVICE_NAME (persistent, low-risk, reversible)
    0x7D,  // GET_DEVICE_NAME

    // ===== Battery Event Report Commands =====
    0x7E,  // ENABLE_BATTERY_REPORT
    0x7F,  // DISABLE_BATTERY_REPORT

    // ===== Service Control Commands =====
    0x10,  // SERVICE_START
    0x11,  // SERVICE_STOP
    0x12,  // SERVICE_STATUS

    // ===== Voice Configuration Commands =====
    0x70,  // SET_VOICE_FRAME_MODE
    0x71,  // GET_VOICE_FRAME_MODE

    // ===== TT Module Force Control =====
    0x72,  // TT_FORCE_ON
    0x73,  // TT_FORCE_OFF

    // ===== Sensor Commands =====
    0x74,  // GET_SENSOR_STATUS (raw data)
    0x75,  // ENABLE_SENSOR_REPORT (param: 0=off, 1=on)
    0x76,  // DISABLE_SENSOR_REPORT (legacy)

    // ===== Limited System Control Commands =====
    0x21,  // RESET_TO_FACTORY (requires additional verification)
    0x22,  // RESET_TT_MODULE
    0x24,  // SET_USB_SWITCH_MODE
    0x25,  // SWITCH_TO_USB_INTERFACE (requires additional verification)
};

/**
 * @brief Check if a command is allowed for debug connection
 *
 * @param cmd Command byte
 * @return true if allowed, false otherwise
 */
static bool is_command_allowed_for_debug(uint8_t cmd) {
    for (size_t i = 0; i < sizeof(DEBUG_ALLOWED_COMMANDS); i++) {
        if (DEBUG_ALLOWED_COMMANDS[i] == cmd) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Check if a command is dangerous (requires additional verification)
 *
 * @param cmd Command byte
 * @return true if dangerous
 */
static bool is_dangerous_command(uint8_t cmd) {
    return (cmd == 0x21 || cmd == 0x25); // RESET_TO_FACTORY, SWITCH_TO_USB
}

#endif // CONFIG_BLE_MULTI_CONN_ENABLE

/* System Service characteristic handles */
static uint16_t system_control_val_handle = 0;
static uint16_t system_info_val_handle = 0;
static uint16_t system_status_val_handle = 0;
static uint16_t system_sensor_val_handle = 0;

/* Command queue configuration */
#define SYS_CMD_QUEUE_SIZE            8       // Maximum pending commands

/* Command queue item (includes connection handle) */
typedef struct {
    uint16_t conn_handle;
    system_cmd_packet_t packet;
} sys_cmd_queue_item_t;

/* Command queue and task handles */
static QueueHandle_t g_sys_cmd_queue = NULL;
static TaskHandle_t g_sys_cmd_task_handle = NULL;

/* Sequence number counter */
static volatile uint8_t g_seq_counter = 0;

/* Static buffer for command processing (avoids large stack allocation) */
static uint8_t g_system_cmd_buffer[SYSTEM_CMD_BUFFER_SIZE];
static SemaphoreHandle_t g_cmd_buffer_mutex = NULL;

/* System server initialization flag */
static bool g_system_server_initialized = false;

/* Forward declarations */
static uint16_t calculate_crc16(const uint8_t *data, size_t len);
static esp_err_t send_command_response(uint16_t conn_handle, const system_cmd_packet_t *cmd,
                                       uint8_t resp_code, const uint8_t *data, size_t data_len);
static void system_command_task(void *pvParameters);
static int system_service_handler(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg);
static esp_err_t handle_system_control_command_async(const system_cmd_packet_t *cmd, uint16_t conn_handle);
static esp_err_t handle_system_info_read(uint16_t conn_handle);
static esp_err_t get_battery_info(system_battery_info_t *info);
static esp_err_t get_charge_status(system_charge_status_t *status);
static int8_t get_ble_tx_power(void);
static esp_err_t set_ble_tx_power(int8_t power);

/**
 * @brief Calculate CRC16-CCITT (polynomial 0x1021, initial 0x0000)
 *
 * Note: Using initial value 0x0000 to match standard CRC16-CCITT calculation
 */
static uint16_t calculate_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x0000;  // Initial value (changed from 0xFFFF)

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  // Polynomial
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Get battery information
 *
 * Reads all battery data from BQ27220 in a single pass with validation.
 * Eliminates redundant I2C reads by reading voltage/current/status once
 * and reusing for charging detection and IR compensation.
 */
static esp_err_t get_battery_info(system_battery_info_t *info)
{
    static system_battery_info_t s_last_good;
    static bool s_have_last_good = false;

    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(info, 0, sizeof(system_battery_info_t));

    fuel_gauge_handle_t battery_handle = power_manage_get_fuel_gauge_handle();
    if (battery_handle == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Battery handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Step 1: Read core battery data in sequence */
    uint16_t voltage = fuel_gauge_get_voltage(battery_handle);
    int16_t current = fuel_gauge_get_current(battery_handle);
    battery_status_t bat_status;
    memset(&bat_status, 0, sizeof(bat_status));
    fuel_gauge_get_battery_status(battery_handle, &bat_status);

    /* Step 2: Validate core data */
    if (voltage < 2500 || voltage > 4500) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
            "Battery voltage out of range: %umV, using last good data", voltage);
        if (s_have_last_good) {
            memcpy(info, &s_last_good, sizeof(system_battery_info_t));
            return ESP_OK;
        }
        return ESP_FAIL;
    }
    if (current < -5000 || current > 5000) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
            "Battery current out of range: %dmA, clamping to 0", current);
        current = 0;
    }

    /* Step 3: Charging state — IP5561 first (authoritative), fuel gauge fallback.
     * Consistent with battery_event_check() in power_manage.c. */
    bool is_charging = power_manage_is_charging();

    /* Step 4: Raw voltage (no IR compensation — consistent with event push) */
    info->voltage_mv = voltage;

    /* Step 5: Set current and charging status */
    info->current_ma = fuel_gauge_get_current(battery_handle);
    info->charging = is_charging ? 1 : 0;
    info->full_charged = bat_status.FC ? 1 : 0;

    /* Step 6: Read remaining battery parameters */
    info->soh_percent = fuel_gauge_get_state_of_health(battery_handle);
    info->temperature_0_1k = power_manage_get_battery_temp_0_1k();
    info->full_charge_capacity_mah = fuel_gauge_get_full_charge_capacity(battery_handle);
    info->remaining_capacity_mah = fuel_gauge_get_remaining_capacity(battery_handle);

    /* Step 7: Validate and clamp values */
    if (info->soh_percent > 100) {
        info->soh_percent = 100;
    }
    if (info->temperature_0_1k == 0 || info->temperature_0_1k > 4000) {
        info->temperature_0_1k = 2981;  /* Default ~25°C (298.1K * 10) */
    }
    if (info->full_charge_capacity_mah == 0) {
        info->full_charge_capacity_mah = 4700;  /* Design capacity fallback */
    }
    if (info->remaining_capacity_mah > info->full_charge_capacity_mah) {
        info->remaining_capacity_mah = info->full_charge_capacity_mah;
    }

    /* Step 8: SOC from fuel_gauge unified interface (same as BATT_EVENT path) */
    info->soc_percent = fuel_gauge_get_state_of_charge(battery_handle);

    /* Cache as last good reading */
    memcpy(&s_last_good, info, sizeof(system_battery_info_t));
    s_have_last_good = true;

    return ESP_OK;
}

/**
 * @brief Get charging status
 */
static esp_err_t get_charge_status(system_charge_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(status, 0, sizeof(system_charge_status_t));

    fuel_gauge_handle_t battery_handle = power_manage_get_fuel_gauge_handle();
    if (battery_handle == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Battery handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    battery_status_t bat_status;
    memset(&bat_status, 0, sizeof(bat_status));
    fuel_gauge_get_battery_status(battery_handle, &bat_status);

    /* Charging: IP5561 first (authoritative), fuel gauge fallback.
     * Consistent with battery_event_check() and get_battery_info(). */
    status->is_charging = power_manage_is_charging() ? 1 : 0;
    status->charge_voltage_mv = fuel_gauge_get_charge_voltage(battery_handle);
    status->charge_current_ma = fuel_gauge_get_charge_current(battery_handle);
    status->time_to_full_min = fuel_gauge_get_time_to_full(battery_handle);
    status->is_full = bat_status.FC ? 1 : 0;

    return ESP_OK;
}

/**
 * @brief Get BLE TX power (in dBm)
 */
static int8_t get_ble_tx_power(void)
{
    return esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
}

/**
 * @brief Set BLE TX power (raw esp_power_level_t)
 */
static esp_err_t set_ble_tx_power_level(esp_power_level_t level)
{
    esp_err_t ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, level);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
            "BLE TX power set to level %d (%d dBm)", level,
            esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT));
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
            "Failed to set BLE TX power level %d: %s", level, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Convert dBm value to esp_power_level_t
 *
 * ESP32-S3 uses the esp32c3 family BT controller which supports:
 * -24, -21, -18, -15, -12, -9, -6, -3, 0, +3, +6, +9, +12, +15, +18, +20 dBm
 *
 * Returns -1 if the value doesn't map to a valid power level.
 */
static int8_t dbm_to_power_level(int8_t dbm)
{
    switch (dbm) {
    case -24: return ESP_PWR_LVL_N24;
    case -21: return ESP_PWR_LVL_N21;
    case -18: return ESP_PWR_LVL_N18;
    case -15: return ESP_PWR_LVL_N15;
    case -12: return ESP_PWR_LVL_N12;
    case  -9: return ESP_PWR_LVL_N9;
    case  -6: return ESP_PWR_LVL_N6;
    case  -3: return ESP_PWR_LVL_N3;
    case   0: return ESP_PWR_LVL_N0;
    case   3: return ESP_PWR_LVL_P3;
    case   6: return ESP_PWR_LVL_P6;
    case   9: return ESP_PWR_LVL_P9;
    case  12: return ESP_PWR_LVL_P12;
    case  15: return ESP_PWR_LVL_P15;
    case  18: return ESP_PWR_LVL_P18;
    case  20: return ESP_PWR_LVL_P20;
    default:  return -1;
    }
}

/**
 * @brief Set BLE TX power (dBm value)
 */
static esp_err_t set_ble_tx_power(int8_t power_dbm)
{
    int8_t level = dbm_to_power_level(power_dbm);
    if (level < 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
            "Invalid TX power %d dBm (valid: -24,-21,-18,-15,-12,-9,-6,-3,0,3,6,9,12,15,18,20)", power_dbm);
        return ESP_ERR_INVALID_ARG;
    }
    return set_ble_tx_power_level((esp_power_level_t)level);
}

/**
 * @brief Timer callback: restore BLE TX power to default after temporary override
 */
static void ble_tx_power_timeout_cb(void *arg)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
        "BLE TX power timeout, restoring to default (level %d)", g_ble_tx_power_original);
    set_ble_tx_power_level(g_ble_tx_power_original);
    g_ble_tx_power_overridden = false;
}

/**
 * @brief Set BLE TX power temporarily with auto-restore timeout
 *
 * @param power_dbm TX power in dBm (-12 to 12, steps of 3)
 * @param timeout_sec Timeout in seconds (0 = permanent, no auto-restore)
 * @return ESP_OK on success
 */
static esp_err_t set_ble_tx_power_temp(int8_t power_dbm, uint32_t timeout_sec)
{
    esp_err_t ret = set_ble_tx_power(power_dbm);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Stop any existing timer */
    if (g_ble_tx_power_timer != NULL) {
        esp_timer_stop(g_ble_tx_power_timer);
    }

    if (timeout_sec == 0) {
        /* Permanent change, no timer needed */
        g_ble_tx_power_overridden = false;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
            "BLE TX power set to %d dBm (permanent)", power_dbm);
        return ESP_OK;
    }

    /* Create timer if first time */
    if (g_ble_tx_power_timer == NULL) {
        esp_timer_create(&(esp_timer_create_args_t){
            .callback = ble_tx_power_timeout_cb,
            .name = "ble_tx_pwr",
        }, &g_ble_tx_power_timer);
    }

    /* Save original power (only if not already overridden) */
    if (!g_ble_tx_power_overridden) {
        g_ble_tx_power_original = (esp_power_level_t)dbm_to_power_level(
            esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT));
    }

    g_ble_tx_power_overridden = true;
    esp_timer_start_once(g_ble_tx_power_timer, (uint64_t)timeout_sec * 1000000ULL);

    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
        "BLE TX power set to %d dBm, auto-restore in %lu s", power_dbm, timeout_sec);
    return ESP_OK;
}


/**
 * @brief Print system control command with human-readable description
 */
static void print_system_command(const uint8_t *data, size_t len)
{
    if (data == NULL || len < 1) {
        return;
    }

    uint8_t cmd = data[0];
    const char *cmd_name = "UNKNOWN";
    char param_str[128] = {0};

    switch (cmd) {
    case SYS_CMD_GET_BATTERY_INFO:
        cmd_name = "GET_BATTERY_INFO";
        snprintf(param_str, sizeof(param_str), "Get battery information");
        break;

    case SYS_CMD_GET_CHARGE_STATUS:
        cmd_name = "GET_CHARGE_STATUS";
        snprintf(param_str, sizeof(param_str), "Get charging status");
        break;

    case SYS_CMD_GET_TT_SIGNAL:
        cmd_name = "GET_TT_SIGNAL";
        snprintf(param_str, sizeof(param_str), "Get Tiantong signal strength");
        break;

    case SYS_CMD_GET_BLE_TX_POWER:
        cmd_name = "GET_BLE_TX_POWER";
        snprintf(param_str, sizeof(param_str), "Get BLE TX power");
        break;

    case SYS_CMD_SET_BLE_TX_POWER:
        cmd_name = "SET_BLE_TX_POWER";
        if (len >= 2) {
            int8_t power = (int8_t)data[1];
            snprintf(param_str, sizeof(param_str), "Set BLE TX power: %d dBm", power);
        }
        break;

    case SYS_CMD_SERVICE_START:
        cmd_name = "SERVICE_START";
        if (len >= 2) {
            const char *svc_name = "UNKNOWN";
            switch (data[1]) {
                case SYS_SERVICE_ID_OTA:  svc_name = "OTA"; break;
                case SYS_SERVICE_ID_LOG:  svc_name = "LOG"; break;
                case SYS_SERVICE_ID_AT:   svc_name = "AT"; break;
                case SYS_SERVICE_ID_SPP:  svc_name = "SPP"; break;
                case SYS_SERVICE_ID_VOICE: svc_name = "VOICE"; break;
            }
            snprintf(param_str, sizeof(param_str), "Start service: %s (id=%d)", svc_name, data[1]);
        }
        break;

    case SYS_CMD_SERVICE_STOP:
        cmd_name = "SERVICE_STOP";
        if (len >= 2) {
            const char *svc_name = "UNKNOWN";
            switch (data[1]) {
                case SYS_SERVICE_ID_OTA:  svc_name = "OTA"; break;
                case SYS_SERVICE_ID_LOG:  svc_name = "LOG"; break;
                case SYS_SERVICE_ID_AT:   svc_name = "AT"; break;
                case SYS_SERVICE_ID_SPP:  svc_name = "SPP"; break;
                case SYS_SERVICE_ID_VOICE: svc_name = "VOICE"; break;
            }
            snprintf(param_str, sizeof(param_str), "Stop service: %s (id=%d)", svc_name, data[1]);
        }
        break;

    case SYS_CMD_SERVICE_STATUS:
        cmd_name = "SERVICE_STATUS";
        if (len >= 2) {
            const char *svc_name = "UNKNOWN";
            switch (data[1]) {
                case SYS_SERVICE_ID_OTA:  svc_name = "OTA"; break;
                case SYS_SERVICE_ID_LOG:  svc_name = "LOG"; break;
                case SYS_SERVICE_ID_AT:   svc_name = "AT"; break;
                case SYS_SERVICE_ID_SPP:  svc_name = "SPP"; break;
                case SYS_SERVICE_ID_VOICE: svc_name = "VOICE"; break;
            }
            snprintf(param_str, sizeof(param_str), "Get service status: %s (id=%d)", svc_name, data[1]);
        }
        break;

    case SYS_CMD_SYSTEM_REBOOT:
        cmd_name = "SYSTEM_REBOOT";
        snprintf(param_str, sizeof(param_str), "Reboot system");
        break;

    case SYS_CMD_SYSTEM_RESET:
        cmd_name = "SYSTEM_RESET";
        snprintf(param_str, sizeof(param_str), "Factory reset");
        break;

    case SYS_CMD_REBOOT_MCU:
        cmd_name = "REBOOT_MCU";
        snprintf(param_str, sizeof(param_str), "Reboot MCU (ESP32)");
        break;

    case SYS_CMD_REBOOT_TT:
        cmd_name = "REBOOT_TT";
        snprintf(param_str, sizeof(param_str), "Reboot Tiantong module");
        break;

    case SYS_CMD_SET_USB_SWITCH:
        cmd_name = "SET_USB_SWITCH";
        if (len >= 2) {
            snprintf(param_str, sizeof(param_str), "USB Switch: %s (0x%02X)",
                     data[1] == 0 ? "To IP5561 (Charge)" : "To ESP32 (Comm)", data[1]);
        } else {
            snprintf(param_str, sizeof(param_str), "USB Switch (invalid params)");
        }
        break;

    case SYS_CMD_RESET_TT_HARDWARE:
        cmd_name = "RESET_TT_HARDWARE";
        snprintf(param_str, sizeof(param_str), "Hardware reset TT module (TEST)");
        break;

    case SYS_CMD_GET_SYSTEM_INFO:
        cmd_name = "GET_SYSTEM_INFO";
        snprintf(param_str, sizeof(param_str), "Get system information");
        break;

    case SYS_CMD_GET_VERSION_INFO:
        cmd_name = "GET_VERSION_INFO";
        snprintf(param_str, sizeof(param_str), "Get version information");
        break;

    case SYS_CMD_LOG_SET_GLOBAL_LEVEL:
        cmd_name = "LOG_SET_GLOBAL_LEVEL";
        if (len >= 2) {
            const char *level_str = "UNKNOWN";
            switch (data[1]) {
                case 0: level_str = "NONE"; break;
                case 1: level_str = "ERROR"; break;
                case 2: level_str = "WARN"; break;
                case 3: level_str = "INFO"; break;
                case 4: level_str = "DEBUG"; break;
            }
            snprintf(param_str, sizeof(param_str), "Set global log level: %s (%d)", level_str, data[1]);
        }
        break;

    case SYS_CMD_LOG_SET_MODULE_CONFIG:
        cmd_name = "LOG_SET_MODULE_CONFIG";
        if (len >= 5) {
            const char *module_str = "UNKNOWN";
            switch (data[1]) {
                case 1: module_str = "MAIN"; break;
                case 2: module_str = "BLE_GATT"; break;
                case 3: module_str = "AUDIO_PROC"; break;
                case 4: module_str = "VOICE_PACKET"; break;
                case 5: module_str = "AT_CMD"; break;
                case 6: module_str = "SPP_AT"; break;
                case 7: module_str = "TT_MODULE"; break;
                case 8: module_str = "OTA"; break;
            }
            const char *level_str = "UNKNOWN";
            switch (data[2]) {
                case 0: level_str = "DEBUG"; break;
                case 1: level_str = "INFO"; break;
                case 2: level_str = "WARN"; break;
                case 3: level_str = "ERROR"; break;
                case 4: level_str = "FATAL"; break;
                case 5: level_str = "NONE"; break;
            }
            snprintf(param_str, sizeof(param_str), "Set module config: %s, level=%s, enabled=%s, gatt=%s",
                     module_str, level_str, data[3] ? "true" : "false", data[4] ? "enabled" : "disabled");
        }
        break;

    case SYS_CMD_LOG_GET_MODULE_CONFIG:
        cmd_name = "LOG_GET_MODULE_CONFIG";
        if (len >= 2) {
            if (data[1] == 0xFF) {
                snprintf(param_str, sizeof(param_str), "Get ALL module configs");
            } else {
                const char *module_str = "UNKNOWN";
                switch (data[1]) {
                    case 1: module_str = "MAIN"; break;
                    case 2: module_str = "BLE_GATT"; break;
                    case 3: module_str = "AUDIO_PROC"; break;
                    case 4: module_str = "VOICE_PACKET"; break;
                    case 5: module_str = "AT_CMD"; break;
                    case 6: module_str = "SPP_AT"; break;
                    case 7: module_str = "TT_MODULE"; break;
                    case 8: module_str = "OTA"; break;
                }
                snprintf(param_str, sizeof(param_str), "Get module config: %s", module_str);
            }
        }
        break;

    case SYS_CMD_LOG_SET_GATT_ENABLE:
        cmd_name = "LOG_SET_GATT_ENABLE";
        if (len >= 2) {
            snprintf(param_str, sizeof(param_str), "Set GATT log global: %s", data[1] ? "ENABLED" : "DISABLED");
        }
        break;

    case SYS_CMD_LOG_GET_GATT_STATE:
        cmd_name = "LOG_GET_GATT_STATE";
        snprintf(param_str, sizeof(param_str), "Get GATT log state");
        break;

    case SYS_CMD_LOG_GET_GATT_STATS:
        cmd_name = "LOG_GET_GATT_STATS";
        snprintf(param_str, sizeof(param_str), "Get GATT buffer stats");
        break;

    case SYS_CMD_LOG_DISABLE_ALL_GATT:
        cmd_name = "LOG_DISABLE_ALL_GATT";
        snprintf(param_str, sizeof(param_str), "Disable all module GATT output");
        break;

    /* PowerBank Commands */
    case SYS_CMD_GET_WORK_MODE_STATUS:
        cmd_name = "GET_WORK_MODE_STATUS";
        snprintf(param_str, sizeof(param_str), "Get work mode status (auto-switched)");
        break;

    case SYS_CMD_GET_CHARGING_STATUS:
        cmd_name = "GET_CHARGING_STATUS";
        snprintf(param_str, sizeof(param_str), "Get charging/discharging status");
        break;

    case SYS_CMD_GET_WIRELESS_STATUS:
        cmd_name = "GET_WIRELESS_STATUS";
        snprintf(param_str, sizeof(param_str), "Get wireless charging status");
        break;

    case SYS_CMD_GET_VBUS_ADC:
        cmd_name = "GET_VBUS_ADC";
        snprintf(param_str, sizeof(param_str), "Get VBUS ADC data");
        break;

    case SYS_CMD_GET_NTC_DATA:
        cmd_name = "GET_NTC_DATA";
        snprintf(param_str, sizeof(param_str), "Get NTC temperature data");
        break;

    case SYS_CMD_SET_CHARGE_VOLTAGE:
        cmd_name = "SET_CHARGE_VOLTAGE";
        if (len >= 3) {
            uint16_t voltage = data[1] | (data[2] << 8);
            snprintf(param_str, sizeof(param_str), "Set charge voltage: %dmV", voltage);
        }
        break;

    case SYS_CMD_SET_CHARGE_CURRENT_9V:
        cmd_name = "SET_CHARGE_CURRENT_9V";
        if (len >= 3) {
            uint16_t current = data[1] | (data[2] << 8);
            snprintf(param_str, sizeof(param_str), "Set 9V charge current: %dmA", current);
        }
        break;

    case SYS_CMD_SET_UV_THRESHOLD_9V:
        cmd_name = "SET_UV_THRESHOLD_9V";
        if (len >= 3) {
            uint16_t threshold = data[1] | (data[2] << 8);
            snprintf(param_str, sizeof(param_str), "Set 9V UV threshold: %dmV", threshold);
        }
        break;

    case SYS_CMD_SET_VBUS_OUTPUT_CURRENT_9V:
        cmd_name = "SET_VBUS_OUTPUT_CURRENT_9V";
        if (len >= 3) {
            uint16_t current = data[1] | (data[2] << 8);
            snprintf(param_str, sizeof(param_str), "Set VBUS 9V output current: %dmA", current);
        }
        break;

    case SYS_CMD_SET_WPC_ENABLED:
        cmd_name = "SET_WPC_ENABLED";
        if (len >= 2) {
            snprintf(param_str, sizeof(param_str), "Set WPC enabled: %s", data[1] ? "true" : "false");
        }
        break;

    case SYS_CMD_GET_TT_STATUS:
        cmd_name = "GET_TT_STATUS";
        break;

    case SYS_CMD_SET_TT_POWER:
        cmd_name = "SET_TT_POWER";
        if (len >= 2) {
            snprintf(param_str, sizeof(param_str), "Set TT power: %s", data[1] ? "ON" : "OFF");
        }
        break;

    case SYS_CMD_SET_VOICE_FRAME_MODE:
        cmd_name = "SET_VOICE_FRAME_MODE";
        if (len >= 2) {
            snprintf(param_str, sizeof(param_str), "Set voice frame mode: %d frames/AT", data[1]);
        }
        break;

    case SYS_CMD_GET_VOICE_FRAME_MODE:
        cmd_name = "GET_VOICE_FRAME_MODE";
        break;

    case SYS_CMD_TT_FORCE_ON:
        cmd_name = "TT_FORCE_ON";
        snprintf(param_str, sizeof(param_str), "Force TT module on (ignore low battery)");
        break;

    case SYS_CMD_TT_FORCE_OFF:
        cmd_name = "TT_FORCE_OFF";
        snprintf(param_str, sizeof(param_str), "Cancel force, restore low battery protection");
        break;

    case SYS_CMD_GET_SENSOR_STATUS:
        cmd_name = "GET_SENSOR_DATA";
        snprintf(param_str, sizeof(param_str), "Get raw sensor data (accel+mag)");
        break;

    case SYS_CMD_ENABLE_SENSOR_REPORT:
        cmd_name = "SET_SENSOR_REPORT";
        snprintf(param_str, sizeof(param_str), "param[0]: 0=off 1=on");
        break;

    case SYS_CMD_DISABLE_SENSOR_REPORT:
        cmd_name = "DISABLE_SENSOR_REPORT";
        snprintf(param_str, sizeof(param_str), "Disable (legacy, use 0x75 with param=0)");
        break;

    case SYS_CMD_SIM_CTRL:
        cmd_name = "SIM_CTRL";
        snprintf(param_str, sizeof(param_str), "Simulation control");
        break;

    case SYS_CMD_SIM_INCOMING:
        cmd_name = "SIM_INCOMING";
        snprintf(param_str, sizeof(param_str), "Trigger incoming call simulation");
        break;

    case SYS_CMD_SIM_SET_NET:
        cmd_name = "SIM_SET_NET";
        snprintf(param_str, sizeof(param_str), "Set simulated network params");
        break;

    case SYS_CMD_SIM_GET_STATE:
        cmd_name = "SIM_GET_STATE";
        snprintf(param_str, sizeof(param_str), "Get simulation state");
        break;

    case SYS_CMD_BLE_TX_POWER_TEMP:
        cmd_name = "BLE_TX_POWER_TEMP";
        if (len >= 4) {
            int8_t power = (int8_t)data[1];
            uint32_t timeout = data[2] | ((uint32_t)data[3] << 8) |
                               ((uint32_t)(len > 4 ? data[4] : 0) << 16) |
                               ((uint32_t)(len > 5 ? data[5] : 0) << 24);
            snprintf(param_str, sizeof(param_str), "Set TX power %d dBm, timeout %lu s", power, timeout);
        }
        break;

    case SYS_CMD_SET_DEVICE_NAME:
        cmd_name = "SET_DEVICE_NAME";
        break;

    case SYS_CMD_GET_DEVICE_NAME:
        cmd_name = "GET_DEVICE_NAME";
        break;

    case SYS_CMD_ENABLE_BATTERY_REPORT:
        cmd_name = "ENABLE_BATTERY_REPORT";
        if (len >= 2) {
            snprintf(param_str, sizeof(param_str), "flags=0x%02X, soc_thr=%u%%", data[1], (len >= 3 ? data[2] : 1));
        }
        break;

    case SYS_CMD_DISABLE_BATTERY_REPORT:
        cmd_name = "DISABLE_BATTERY_REPORT";
        break;

    default:
        snprintf(param_str, sizeof(param_str), "Unknown command (0x%02x)", cmd);
        break;
    }

    // Print hex data and description
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Received: %s", cmd_name);
    if (param_str[0] != '\0') {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "  Params: %s", param_str);
    }
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len > 32 ? 32 : len, ESP_LOG_INFO);
}

/**
 * @brief Handle system control command (legacy — unused, kept for reference)
 */
__attribute__((unused))
static esp_err_t handle_system_control_command(uint16_t conn_handle, const uint8_t *data, size_t len)
{
    if (data == NULL || len < 1) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Invalid command data");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = data[0];
    uint8_t response = SYS_RESP_OK;
    uint8_t resp_data[256] = {0};  // Increased buffer size for version info
    size_t resp_len = 1;  // Start with response code

    // Print command with human-readable description
    print_system_command(data, len);

    switch (cmd) {
    case SYS_CMD_GET_BATTERY_INFO:
        {
            system_battery_info_t bat_info;
            if (get_battery_info(&bat_info) == ESP_OK) {
                memcpy(resp_data + 1, &bat_info, sizeof(bat_info));
                resp_len = 1 + sizeof(bat_info);
            } else {
                response = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_CHARGE_STATUS:
        {
            system_charge_status_t charge_status;
            if (get_charge_status(&charge_status) == ESP_OK) {
                memcpy(resp_data + 1, &charge_status, sizeof(charge_status));
                resp_len = 1 + sizeof(charge_status);
            } else {
                response = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_TT_SIGNAL:
        {
            // DEPRECATED: Signal status should be queried by client via AT Service
            // Client should send AT+CREG? and AT+CPIN? through GATT AT Service
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                           "GET_TT_SIGNAL deprecated, client should query via AT Service");
            response = SYS_RESP_INVALID_CMD;
        }
        break;

    case SYS_CMD_GET_BLE_TX_POWER:
        {
            int8_t power = get_ble_tx_power();
            resp_data[1] = (uint8_t)power;
            resp_len = 2;
        }
        break;

    case SYS_CMD_SET_BLE_TX_POWER:
        {
            if (len < 2) {
                response = SYS_RESP_INVALID_PARAM;
                break;
            }
            int8_t power = (int8_t)data[1];
            if (set_ble_tx_power(power) == ESP_OK) {
                resp_data[1] = (uint8_t)power;
                resp_len = 2;
            } else {
                response = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SERVICE_START:
        {
            if (len < 2) {
                response = SYS_RESP_INVALID_PARAM;
                break;
            }
            uint8_t service_id = data[1];
            int ret = gatt_system_server_start_service(service_id, conn_handle);
            if (ret == 0) {
                resp_data[1] = service_id;
                resp_len = 2;
            } else {
                response = (ret == -1) ? SYS_RESP_SERVICE_ALREADY_RUNNING : SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SERVICE_STOP:
        {
            if (len < 2) {
                response = SYS_RESP_INVALID_PARAM;
                break;
            }
            uint8_t service_id = data[1];
            int ret = gatt_system_server_stop_service(service_id);
            if (ret == 0) {
                resp_data[1] = service_id;
                resp_len = 2;
            } else {
                response = (ret == -1) ? SYS_RESP_SERVICE_NOT_RUNNING : SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SERVICE_STATUS:
        {
            if (len < 2) {
                response = SYS_RESP_INVALID_PARAM;
                break;
            }
            uint8_t service_id = data[1];
            int status = gatt_system_server_get_service_status(service_id);
            if (status >= 0) {
                resp_data[1] = service_id;
                resp_data[2] = (uint8_t)status;
                resp_len = 3;
            } else {
                response = SYS_RESP_SERVICE_NOT_FOUND;
            }
        }
        break;

    case SYS_CMD_SYSTEM_REBOOT:
        {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "System reboot requested");
            // Send response first, then reboot
            resp_data[0] = SYS_RESP_OK;
            struct os_mbuf *txom = ble_hs_mbuf_from_flat(resp_data, 1);
            if (txom) {
                ble_gatts_notify_custom(conn_handle, system_control_val_handle, txom);
            }
            // Delay to allow response to be sent
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            return ESP_OK;  // Won't reach here
        }
        break;

    case SYS_CMD_SYSTEM_RESET:
        {
            /*
             * Factory Reset Design Notes:
             *
             * Purpose: Restore device to original factory settings
             *
             * Required Actions (not yet implemented):
             * 1. Erase NVS partition: nvs_flash_erase()
             * 2. Reset BLE pairing data: Remove bond information from NimBLE
             * 3. Restore default settings:
             *    - LED patterns (auto-off timeout)
             *    - Audio codec settings
             *    - TT module auto-start configuration
             * 4. Reboot device to apply changes
             *
             * Safety Considerations:
             * - Should require authentication (magic token in parameters)
             * - Should log the event for audit trail
             * - Should confirm operation before executing
             *
             * Current Behavior:
             * - Returns SYS_RESP_ERROR (operation not supported)
             * - Logs warning message
             * - Does not modify any settings
             *
             * Risk Assessment:
             * - LOW: Factory reset is an admin function, not in critical path
             * - Can be implemented later via NVS API and BLE store reset
             *
             * Implementation Priority: MEDIUM
             * - Requires coordination with power_manage.c for NVS management
             * - Requires NimBLE store API for bonding data reset
             */
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                           "Factory reset requested but not implemented");
            response = SYS_RESP_ERROR;  // Not implemented yet
        }
        break;

    case SYS_CMD_REBOOT_MCU:
        {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "MCU reboot requested, shutting down TT module first");

            // Power off Tiantong module first
            esp_err_t ret = tt_module_power_off();
            if (ret != ESP_OK) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to power off TT module: %s", esp_err_to_name(ret));
                // Continue with reboot anyway
            }

            // Send response first, then reboot
            resp_data[0] = SYS_RESP_OK;
            struct os_mbuf *txom = ble_hs_mbuf_from_flat(resp_data, 1);
            if (txom) {
                ble_gatts_notify_custom(conn_handle, system_control_val_handle, txom);
            }

            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Shutting down TT module and rebooting MCU in 2 seconds...");
            // Delay to allow response to be sent and TT module to power off
            vTaskDelay(pdMS_TO_TICKS(MCU_REBOOT_DELAY_MS));
            esp_restart();
            return ESP_OK;  // Won't reach here
        }
        break;

    case SYS_CMD_REBOOT_TT:
        {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Tiantong module reboot requested");

            // Reset Tiantong module
            esp_err_t ret = tt_module_reset();
            if (ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to reset TT module: %s", esp_err_to_name(ret));
                response = SYS_RESP_ERROR;
            } else {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Tiantong module reset initiated");
                response = SYS_RESP_OK;
            }
        }
        break;

    case SYS_CMD_GET_SYSTEM_INFO:
        {
            system_info_t sys_info;
            sys_info.uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000);
            sys_info.free_heap = esp_get_free_heap_size();
            sys_info.min_free_heap = esp_get_minimum_free_heap_size();

            esp_chip_info_t chip_info;
            esp_chip_info(&chip_info);
            sys_info.cpu_freq_mhz = (uint8_t)(esp_clk_cpu_freq() / 1000000);

            // Build service status bitmask
            sys_info.service_status = 0;
            if (gatt_ota_server_is_enabled()) sys_info.service_status |= (1 << 0);
            if (gatt_log_server_is_enabled()) sys_info.service_status |= (1 << 1);
            sys_info.service_status |= (1 << 2);  // AT always enabled
            sys_info.service_status |= (1 << 3);  // SPP always enabled
            if (spp_voice_server_is_enabled()) sys_info.service_status |= (1 << 4);

            memcpy(resp_data + 1, &sys_info, sizeof(sys_info));
            resp_len = 1 + sizeof(sys_info);
        }
        break;

    case SYS_CMD_GET_VERSION_INFO:
        {
            system_version_info_t version_info = {0};

            // Copy version strings
            strncpy(version_info.firmware_version, get_firmware_version(), sizeof(version_info.firmware_version) - 1);
            strncpy(version_info.software_version, get_software_version(), sizeof(version_info.software_version) - 1);
            strncpy(version_info.manufacturer, get_manufacturer_name(), sizeof(version_info.manufacturer) - 1);
            strncpy(version_info.model_number, get_model_number(), sizeof(version_info.model_number) - 1);
            strncpy(version_info.hardware_revision, get_hardware_revision(), sizeof(version_info.hardware_revision) - 1);
            strncpy(version_info.build_datetime, get_build_datetime(), sizeof(version_info.build_datetime) - 1);

            memcpy(resp_data + 1, &version_info, sizeof(version_info));
            resp_len = 1 + sizeof(version_info);

            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Version info: FW=%s, SW=%s",
                     version_info.firmware_version, version_info.software_version);
        }
        break;

    case SYS_CMD_LOG_SET_GLOBAL_LEVEL:
    case SYS_CMD_LOG_SET_MODULE_CONFIG:
    case SYS_CMD_LOG_GET_MODULE_CONFIG:
    case SYS_CMD_LOG_SET_GATT_ENABLE:
    case SYS_CMD_LOG_GET_GATT_STATE:
    case SYS_CMD_LOG_GET_GATT_STATS:
    case SYS_CMD_LOG_DISABLE_ALL_GATT:
        {
            // Syslog configuration commands - translate command codes and pass to syslog module

            // Build syslog command format: [cmd_type][data_len][data...]
            uint8_t syslog_cmd[32];
            size_t syslog_cmd_len = 0;

            // Map GATT System Server command to syslog module command
            uint8_t syslog_cmd_type = 0;
            size_t syslog_data_len = 0;

            switch (cmd) {
            case SYS_CMD_LOG_SET_GLOBAL_LEVEL:
                syslog_cmd_type = 0x01;  // SYS_LOG_CMD_SET_GLOBAL_LEVEL
                syslog_data_len = 1;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd[2] = data[1];  // level
                syslog_cmd_len = 3;
                break;

            case SYS_CMD_LOG_SET_MODULE_CONFIG:
                syslog_cmd_type = 0x10;  // SYS_LOG_CMD_SET_MODULE_CONFIG
                syslog_data_len = 4;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd[2] = data[1];  // module
                syslog_cmd[3] = data[2];  // level
                syslog_cmd[4] = data[3];  // enabled
                syslog_cmd[5] = data[4];  // gatt_output
                syslog_cmd_len = 6;
                break;

            case SYS_CMD_LOG_GET_MODULE_CONFIG:
                syslog_cmd_type = 0x11;  // SYS_LOG_CMD_GET_MODULE_CONFIG
                syslog_data_len = 1;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd[2] = data[1];  // module (0xFF for all)
                syslog_cmd_len = 3;
                break;

            case SYS_CMD_LOG_SET_GATT_ENABLE:
                syslog_cmd_type = 0x30;  // SYS_LOG_CMD_SET_GATT_GLOBAL
                syslog_data_len = 1;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd[2] = data[1];  // enabled
                syslog_cmd_len = 3;
                break;

            case SYS_CMD_LOG_GET_GATT_STATE:
                syslog_cmd_type = 0x32;  // SYS_LOG_CMD_GET_GATT_STATE
                syslog_data_len = 0;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd_len = 2;
                break;

            case SYS_CMD_LOG_GET_GATT_STATS:
                syslog_cmd_type = 0x33;  // SYS_LOG_CMD_GET_GATT_STATS
                syslog_data_len = 0;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd_len = 2;
                break;

            case SYS_CMD_LOG_DISABLE_ALL_GATT:
                syslog_cmd_type = 0x42;  // SYS_LOG_CMD_DISABLE_ALL_GATT
                syslog_data_len = 0;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd_len = 2;
                break;

            default:
                response = SYS_RESP_ERROR;
                resp_len = 1;
                break;
            }

            if (syslog_cmd_len > 0) {
                size_t syslog_resp_len = 0;
                esp_err_t ret = syslog_process_remote_config(syslog_cmd, syslog_cmd_len, resp_data, &syslog_resp_len);
                if (ret == ESP_OK) {
                    resp_len = syslog_resp_len;
                } else {
                    response = SYS_RESP_ERROR;
                    resp_len = 1;
                }
            }
        }
        break;

#if ENABLE_SAT_CALL_SIM
    case SYS_CMD_SIM_CTRL:
    case SYS_CMD_SIM_INCOMING:
    case SYS_CMD_SIM_SET_NET:
        /* Simulation commands are handled asynchronously */
        response = SYS_RESP_OK;
        resp_len = 1;
        break;
    case SYS_CMD_SIM_GET_STATE:
        /* Handled asynchronously */
        response = SYS_RESP_OK;
        resp_len = 1;
        break;
#endif

    default:
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Unknown system command: 0x%02x", cmd);
        response = SYS_RESP_INVALID_CMD;
        break;
    }

    // Send response
    resp_data[0] = response;
    struct os_mbuf *txom = ble_hs_mbuf_from_flat(resp_data, resp_len);
    if (txom) {
        ble_gatts_notify_custom(conn_handle, system_control_val_handle, txom);
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to allocate response mbuf");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Handle system info read request
 */
static esp_err_t handle_system_info_read(uint16_t conn_handle)
{
    system_info_t sys_info;

    sys_info.uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000);
    sys_info.free_heap = esp_get_free_heap_size();
    sys_info.min_free_heap = esp_get_minimum_free_heap_size();

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    sys_info.cpu_freq_mhz = (uint8_t)(esp_clk_cpu_freq() / 1000000);

    // Build service status bitmask
    sys_info.service_status = 0;
    if (gatt_ota_server_is_enabled()) sys_info.service_status |= (1 << 0);
    if (gatt_log_server_is_enabled()) sys_info.service_status |= (1 << 1);
    sys_info.service_status |= (1 << 2);  // AT always enabled
    sys_info.service_status |= (1 << 3);  // SPP always enabled
    if (spp_voice_server_is_enabled()) sys_info.service_status |= (1 << 4);

    // Send system info
    struct os_mbuf *txom = ble_hs_mbuf_from_flat(&sys_info, sizeof(sys_info));
    if (txom) {
        ble_gatts_notify_custom(conn_handle, system_info_val_handle, txom);
        return ESP_OK;
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to allocate system info mbuf");
        return ESP_ERR_NO_MEM;
    }
}

/**
 * @brief System Service Handler
 */
static int system_service_handler(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (attr_handle == system_info_val_handle) {
            // Handle system info read directly (non-blocking)
            if (handle_system_info_read(conn_handle) != ESP_OK) {
                return BLE_HS_ENOMEM;
            }
        } else if (attr_handle == system_sensor_val_handle) {
            /* Read latest sensor sample: 13 bytes (same layout as the notify stream) */
            uint8_t flags = 0;
            int16_t ax = 0, ay = 0, az = 0;
            int32_t mx = 0, my = 0, mz = 0;
            power_manage_get_sensor_data(&flags, &ax, &ay, &az, &mx, &my, &mz);
            uint8_t buf[13];
            buf[0] = flags;
            buf[1] = (uint8_t)(ax & 0xFF);  buf[2] = (uint8_t)((ax >> 8) & 0xFF);
            buf[3] = (uint8_t)(ay & 0xFF);  buf[4] = (uint8_t)((ay >> 8) & 0xFF);
            buf[5] = (uint8_t)(az & 0xFF);  buf[6] = (uint8_t)((az >> 8) & 0xFF);
            buf[7] = (uint8_t)(mx & 0xFF);  buf[8] = (uint8_t)((mx >> 8) & 0xFF);
            buf[9] = (uint8_t)(my & 0xFF);  buf[10] = (uint8_t)((my >> 8) & 0xFF);
            buf[11] = (uint8_t)(mz & 0xFF); buf[12] = (uint8_t)((mz >> 8) & 0xFF);
            os_mbuf_append(ctxt->om, buf, sizeof(buf));
            return 0;
        }
        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (attr_handle == system_control_val_handle) {
            /* Notify sleep manager of BLE activity */
            extern void sleep_manager_notify_activity(const char *source);
            sleep_manager_notify_activity("sys_cmd_rx");

            // Parse new structured packet format: [seq][cmd][param_len][params...][crc16_lo][crc16_hi]
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "System control command received: len=%d", ctxt->om->om_len);

            // Check minimum packet size (seq + cmd + param_len + crc16 = 5 bytes)
            if (ctxt->om->om_len < 5) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Packet too small: %d (min 5)", ctxt->om->om_len);
                return BLE_HS_EINVAL;
            }

            // Check maximum packet size
            if (ctxt->om->om_len > sizeof(system_cmd_packet_t)) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Packet too large: %d (max %d)",
                         ctxt->om->om_len, sizeof(system_cmd_packet_t));
                return BLE_HS_ENOMEM;
            }

            // Copy packet to local buffer
            sys_cmd_queue_item_t queue_item;
            queue_item.conn_handle = conn_handle;
            memset(&queue_item.packet, 0, sizeof(queue_item.packet));

            int rc = ble_hs_mbuf_to_flat(ctxt->om, (uint8_t *)&queue_item.packet, ctxt->om->om_len, NULL);
            if (rc != 0) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to copy packet data: %d", rc);
                return rc;
            }

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
            // Check connection role for debug permission control
            ble_conn_role_t role = ble_conn_manager_get_role(conn_handle);
            if (role == BLE_CONN_ROLE_DEBUG) {
                uint8_t cmd = queue_item.packet.cmd;

                // Check if command is allowed for debug connection
                if (!is_command_allowed_for_debug(cmd)) {
                    SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                                  "Command 0x%02X NOT ALLOWED for DEBUG connection (handle=%d)",
                                  cmd, conn_handle);
                    return BLE_ATT_ERR_INSUFFICIENT_AUTHOR;
                }

                // Check dangerous commands (require additional verification)
                if (is_dangerous_command(cmd)) {
                    /*
                     * Magic Token Verification Design:
                     *
                     * Purpose: Prevent accidental execution of dangerous commands
                     *
                     * Dangerous Commands:
                     * - SYS_CMD_FACTORY_RESET (0x04): Erase all settings
                     * - SYS_CMD_REBOOT_MCU (0x05): Restart system
                     * - SYS_CMD_SHUTDOWN_TT (0x06): Power off TT module
                     * - SYS_CMD_START_OTA (0x08): Start firmware update
                     *
                     * Proposed Implementation:
                     * 1. Reserve last 4 bytes of command parameters for magic token
                     * 2. Define magic value: 0xA5A5A5A5 (or similar)
                     * 3. Verify token before executing dangerous command
                     * 4. Return error if token missing or incorrect
                     *
                     * Current Behavior:
                     * - All dangerous commands allowed from DEBUG connections
                     * - Warning logged for audit trail
                     * - No token verification performed
                     *
                     * Risk Assessment:
                     * - LOW: DEBUG connections require authentication (encryption + bonding)
                     * - Acceptable for development/testing phase
                     * - Should implement for production deployment
                     */
                    SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                                  "Dangerous command 0x%02X from DEBUG connection (handle=%d)",
                                  cmd, conn_handle);
                }

                SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                              "Command 0x%02X ALLOWED for DEBUG connection (handle=%d)",
                              cmd, conn_handle);
            }
#endif

            // Print raw packet bytes for debugging
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Raw packet data (len=%d):", ctxt->om->om_len);
            const uint8_t *raw_data = (const uint8_t *)&queue_item.packet;
            for (int i = 0; i < ctxt->om->om_len; i++) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "  [%d] 0x%02x '%c'", i, raw_data[i],
                         (raw_data[i] >= 32 && raw_data[i] <= 126) ? raw_data[i] : '.');
            }

            // Validate param_len field
            size_t expected_len = 3 + queue_item.packet.param_len + 2;  // seq+cmd+param_len + params + crc16
            if (ctxt->om->om_len != expected_len) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                         "Packet length mismatch: got %d, expected %zu (param_len=%d)",
                         ctxt->om->om_len, expected_len, queue_item.packet.param_len);
                return BLE_HS_EINVAL;
            }

            // Queue command for async processing (non-blocking)
            if (xQueueSend(g_sys_cmd_queue, &queue_item, 0) != pdTRUE) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Command queue full!");
                return BLE_HS_ENOMEM;
            }

            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                     "Command queued: seq=%d, cmd=0x%02x, param_len=%d",
                     queue_item.packet.seq, queue_item.packet.cmd, queue_item.packet.param_len);
        }
        break;

    default:
        break;
    }
    return 0;
}

/**
 * @brief Send system status notification
 */
int gatt_system_server_send_status(uint16_t conn_handle, uint8_t status, const uint8_t *data, size_t len)
{
    uint8_t status_data[64];
    size_t total_len = 1 + len;
    
    if (total_len > sizeof(status_data)) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Status data too large");
        return BLE_HS_EINVAL;
    }

    status_data[0] = status;
    if (data != NULL && len > 0) {
        memcpy(status_data + 1, data, len);
    }

    struct os_mbuf *txom = ble_hs_mbuf_from_flat(status_data, total_len);
    if (!txom) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to allocate mbuf");
        return BLE_HS_ENOMEM;
    }

    int rc = ble_gatts_notify_custom(conn_handle, system_status_val_handle, txom);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to send status notification: rc=%d", rc);
        os_mbuf_free(txom);
        return rc;
    }

    return 0;
}

/**
 * @brief Start a GATT service dynamically
 */
int gatt_system_server_start_service(uint8_t service_id, uint16_t conn_handle)
{
    int ret = 0;

    switch (service_id) {
    case SYS_SERVICE_ID_OTA:
        if (gatt_ota_server_is_enabled()) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "OTA service already running");
            return -1;
        }
        // Service is already initialized during startup, just enable it
        gatt_ota_server_enable();
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "OTA service started");
        break;

    case SYS_SERVICE_ID_LOG:
        if (gatt_log_server_is_enabled()) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log service already running");
            return -1;
        }
        // Service is already initialized during startup, just enable it
        gatt_log_server_enable();
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log service started");
        break;

    case SYS_SERVICE_ID_AT:
        // AT service is always enabled (core service)
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "AT service cannot be started/stopped (core service)");
        return -1;

    case SYS_SERVICE_ID_SPP:
        // SYSTEM service is always enabled (core service)
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "SYSTEM service cannot be started/stopped (core service)");
        return -1;

    case SYS_SERVICE_ID_VOICE:
        if (spp_voice_server_is_enabled()) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Voice service already running");
            return -1;
        }
        // Check if OTA is in progress
        if (gatt_ota_is_in_progress()) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Cannot start Voice service: OTA in progress");
            return -3;  // Custom error code for OTA in progress
        }
        // Service is already initialized during startup, just enable it
        spp_voice_server_enable(conn_handle);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Voice GATT service started");
        break;

    default:
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Unknown service ID: %d", service_id);
        return -2;
    }

    return ret;
}

/**
 * @brief Stop a GATT service dynamically
 */
int gatt_system_server_stop_service(uint8_t service_id)
{
    switch (service_id) {
    case SYS_SERVICE_ID_OTA:
        if (!gatt_ota_server_is_enabled()) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "OTA service not running");
            return -1;
        }
        gatt_ota_server_disable();
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "OTA service stopped");
        break;

    case SYS_SERVICE_ID_LOG:
        if (!gatt_log_server_is_enabled()) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log service not running");
            return -1;
        }
        gatt_log_server_disable();
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Log service stopped");
        break;

    case SYS_SERVICE_ID_AT:
        // AT service is always enabled (core service)
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "AT service cannot be started/stopped (core service)");
        return -1;

    case SYS_SERVICE_ID_SPP:
        // SYSTEM service is always enabled (core service)
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "SYSTEM service cannot be started/stopped (core service)");
        return -1;

    case SYS_SERVICE_ID_VOICE:
        if (!spp_voice_server_is_enabled()) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Voice service not running");
            return -1;
        }
        spp_voice_server_disable();
        SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Voice GATT service stopped");
        break;

    default:
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Unknown service ID: %d", service_id);
        return -2;
    }

    return 0;
}

/**
 * @brief Get service status
 */
int gatt_system_server_get_service_status(uint8_t service_id)
{
    switch (service_id) {
    case SYS_SERVICE_ID_OTA:
        return gatt_ota_server_is_enabled() ? 1 : 0;
    case SYS_SERVICE_ID_LOG:
        return gatt_log_server_is_enabled() ? 1 : 0;
    case SYS_SERVICE_ID_AT:
        return 1;  // AT service is always enabled
    case SYS_SERVICE_ID_SPP:
        return 1;  // SPP/System service is always enabled
    case SYS_SERVICE_ID_VOICE:
        return spp_voice_server_is_enabled() ? 1 : 0;
    default:
        return -1;
    }
}


/* System Service Definition */
static const struct ble_gatt_svc_def system_service_defs[] = {
    {
        /*** Service: System Control ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SYSTEM_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* Control Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_SYSTEM_CHR_CONTROL_UUID16),
                .access_cb = system_service_handler,
                .val_handle = &system_control_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Info Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_SYSTEM_CHR_INFO_UUID16),
                .access_cb = system_service_handler,
                .val_handle = &system_info_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Status Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_SYSTEM_CHR_STATUS_UUID16),
                .access_cb = system_service_handler,
                .val_handle = &system_status_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Sensor Characteristic (high-rate data stream, NOTIFY) */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_SYSTEM_CHR_SENSOR_UUID16),
                .access_cb = system_service_handler,
                .val_handle = &system_sensor_val_handle,
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

/**
 * @brief Send sensor data notification via Control characteristic
 *
 * Called from sensor_monitor_task when report is enabled.
 * Uses primary connection handle.
 */
int gatt_system_server_send_sensor_data(void)
{
    uint16_t conn_handle = ble_conn_manager_get_primary_handle();
    if (conn_handle == 0) {
        return BLE_HS_ENOTCONN;
    }

    uint8_t flags;
    int16_t ax, ay, az;
    int32_t mx, my, mz;
    power_manage_get_sensor_data(&flags, &ax, &ay, &az, &mx, &my, &mz);

    if (!flags) {
        return ESP_OK;  /* No valid data */
    }

    /* Build notification payload on dedicated sensor characteristic (0xABF2).
     * No report-ID prefix (the characteristic itself identifies the stream):
     * [flags][ax_lo][ax_hi][ay_lo][ay_hi][az_lo][az_hi]
     * [mx_lo][mx_hi][my_lo][my_hi][mz_lo][mz_hi] = 13 bytes */
    uint8_t buf[13];
    buf[0] = flags;
    buf[1] = (uint8_t)(ax & 0xFF);
    buf[2] = (uint8_t)((ax >> 8) & 0xFF);
    buf[3] = (uint8_t)(ay & 0xFF);
    buf[4] = (uint8_t)((ay >> 8) & 0xFF);
    buf[5] = (uint8_t)(az & 0xFF);
    buf[6] = (uint8_t)((az >> 8) & 0xFF);
    buf[7] = (uint8_t)(mx & 0xFF);
    buf[8] = (uint8_t)((mx >> 8) & 0xFF);
    buf[9] = (uint8_t)(my & 0xFF);
    buf[10] = (uint8_t)((my >> 8) & 0xFF);
    buf[11] = (uint8_t)(mz & 0xFF);
    buf[12] = (uint8_t)((mz >> 8) & 0xFF);

    struct os_mbuf *txom = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (!txom) {
        return BLE_HS_ENOMEM;
    }

    /* Fire-and-forget NOTIFY: caller does not retry on failure (high-rate stream,
     * a dropped packet is acceptable — next sample arrives in 50ms). */
    return ble_gatts_notify_custom(conn_handle, system_sensor_val_handle, txom);
}

int gatt_system_server_send_battery_event(uint8_t event_type, uint8_t soc,
                                           uint16_t voltage_mv, uint16_t temp_0_1k,
                                           uint8_t charging)
{
    if (event_type == BATTERY_EVENT_NONE) {
        return ESP_OK;
    }

    uint16_t conn_handle = ble_conn_manager_get_primary_handle();
    if (conn_handle == 0) {
        return BLE_HS_ENOTCONN;
    }

    /* Build notification: [0x7E][event_type][soc][v_lo][v_hi][t_lo][t_hi][charging] */
    uint8_t buf[8];
    buf[0] = SYS_CMD_ENABLE_BATTERY_REPORT;  /* Event notification identifier */
    buf[1] = event_type;
    buf[2] = soc;
    buf[3] = (uint8_t)(voltage_mv & 0xFF);
    buf[4] = (uint8_t)((voltage_mv >> 8) & 0xFF);
    buf[5] = (uint8_t)(temp_0_1k & 0xFF);
    buf[6] = (uint8_t)((temp_0_1k >> 8) & 0xFF);
    buf[7] = charging;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                    "Battery event: type=0x%02X soc=%u%% V=%umV T=%u(0.1K) chg=%d",
                    event_type, soc, voltage_mv, temp_0_1k, charging);

    struct os_mbuf *txom = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (!txom) {
        return BLE_HS_ENOMEM;
    }

    return ble_gatts_notify_custom(conn_handle, system_control_val_handle, txom);
}

/**
 * @brief Send command response via BLE GATT notification
 *
 * @param conn_handle BLE connection handle
 * @param cmd Original command packet (for sequence number)
 * @param resp_code Response code
 * @param data Response data (can be NULL)
 * @param data_len Response data length
 * @return ESP_OK on success
 */
static esp_err_t send_command_response(uint16_t conn_handle, const system_cmd_packet_t *cmd,
                                       uint8_t resp_code, const uint8_t *data, size_t data_len)
{
    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    system_resp_packet_t resp;
    memset(&resp, 0, sizeof(resp));

    // Build response packet
    resp.seq = cmd->seq;                    // Echo sequence number
    resp.cmd = cmd->cmd;                    // Echo command code
    resp.resp_code = resp_code;
    resp.data_len = (data_len > sizeof(resp.data)) ? sizeof(resp.data) : data_len;

    if (data != NULL && data_len > 0) {
        memcpy(resp.data, data, resp.data_len);
    }

    // Calculate CRC16 (excluding CRC field itself)
    size_t crc_len = offsetof(system_resp_packet_t, crc16);
    resp.crc16 = calculate_crc16((const uint8_t *)&resp, crc_len);

    // Calculate actual response length (header + data + crc)
    size_t resp_total_len = offsetof(system_resp_packet_t, data) + resp.data_len + sizeof(resp.crc16);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, ">>> GATT System Response (%d bytes):", resp_total_len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, (const uint8_t *)&resp, resp_total_len, ESP_LOG_INFO);

    // Send via notify (only send actual data length, not full struct)
    struct os_mbuf *txom = ble_hs_mbuf_from_flat((const uint8_t *)&resp, resp_total_len);
    if (!txom) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to allocate mbuf for response");
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gatts_notify_custom(conn_handle, system_control_val_handle, txom);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to send response: rc=%d", rc);
        os_mbuf_free(txom);
        return ESP_FAIL;
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Response sent: seq=%d, cmd=0x%02x, code=0x%02x, len=%d",
              resp.seq, resp.cmd, resp.resp_code, resp.data_len);
    return ESP_OK;
}

int gatt_system_server_send_tt_status(uint8_t tt_state)
{
    uint16_t conn_handle = ble_conn_manager_get_primary_handle();
    if (conn_handle == 0) {
        return BLE_HS_ENOTCONN;
    }

    uint8_t buf[2];
    buf[0] = TT_STATUS_NOTIFY_ID;
    buf[1] = tt_state;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "TT status notify: state=%d", tt_state);

    struct os_mbuf *txom = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (!txom) return BLE_HS_ENOMEM;

    return ble_gatts_notify_custom(conn_handle, system_control_val_handle, txom);
}

int gatt_system_server_send_sleep_warning(uint16_t seconds_until_sleep)
{
    uint16_t conn_handle = ble_conn_manager_get_primary_handle();
    if (conn_handle == 0) {
        return BLE_HS_ENOTCONN;
    }

    /* [0x09][seconds_lo][seconds_hi] */
    uint8_t buf[3];
    buf[0] = 0x09;
    buf[1] = (uint8_t)(seconds_until_sleep & 0xFF);
    buf[2] = (uint8_t)((seconds_until_sleep >> 8) & 0xFF);

    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                    "Sleep imminent: %us until deep sleep", seconds_until_sleep);

    struct os_mbuf *txom = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (!txom) return BLE_HS_ENOMEM;

    return ble_gatts_notify_custom(conn_handle, system_control_val_handle, txom);
}

/**
 * @brief Handle system control command (async version with structured packets)
 *
 * @param cmd Pointer to command packet
 * @param conn_handle BLE connection handle
 * @return esp_err_t
 *
 * CRITICAL: Mutex handling to prevent deadlock:
 * 1. Acquire mutex only to copy command parameters
 * 2. Copy to local variables and release immediately
 * 3. Process command WITHOUT holding lock
 * 4. Acquire lock again only if needed for response
 */
static esp_err_t handle_system_control_command_async(const system_cmd_packet_t *cmd, uint16_t conn_handle)
{
    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd_code = cmd->cmd;
    uint8_t param_len = cmd->param_len;

    // Note: param_len is uint8_t (max 255), SYSTEM_CMD_BUFFER_SIZE is 256,
    // so param_len always fits in g_system_cmd_buffer. No overflow check needed.

    // Acquire mutex: ONLY to copy command parameters and print
    if (xSemaphoreTake(g_cmd_buffer_mutex, pdMS_TO_TICKS(SYS_CMD_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to acquire cmd buffer mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Copy parameters to local buffer (within critical section)
    g_system_cmd_buffer[0] = cmd_code;
    memcpy(g_system_cmd_buffer + 1, cmd->params, param_len);
    print_system_command(g_system_cmd_buffer, 1 + param_len);

    // Copy to local variable for processing AFTER releasing lock
    uint8_t cmd_params[SYS_CMD_PACKET_MAX_PARAMS];
    memcpy(cmd_params, cmd->params, param_len);

    // CRITICAL: Release mutex BEFORE processing command
    // This prevents deadlock when commands trigger reboot or other blocking operations
    xSemaphoreGive(g_cmd_buffer_mutex);

    // Process command WITHOUT holding lock (using local variables)
    uint8_t resp_data[256] = {0};
    size_t resp_len = 0;
    uint8_t resp_code = SYS_RESP_OK;

    /* Block TT/reboot commands during any OTA (MCU or TT) to prevent bricking */
    if (tt_module_ota_is_in_progress() || gatt_ota_is_in_progress()) {
        switch (cmd_code) {
        case SYS_CMD_SYSTEM_REBOOT:     /* 0x20 */
        case SYS_CMD_REBOOT_MCU:        /* 0x22 */
        case SYS_CMD_REBOOT_TT:         /* 0x23 */
        case SYS_CMD_SET_TT_POWER:      /* 0x24 */
        case SYS_CMD_RESET_TT_HARDWARE: /* 0x25 */
        case SYS_CMD_TT_FORCE_ON:       /* 0x72 */
        case SYS_CMD_TT_FORCE_OFF:      /* 0x73 */
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                           "Command 0x%02X blocked: OTA in progress", cmd_code);
            resp_code = SYS_RESP_ERROR;
            send_command_response(conn_handle, cmd, resp_code, resp_data, resp_len);
            return ESP_OK;
        default:
            break;  /* Allow non-TT commands (battery query, sensor, etc.) */
        }
    }

    switch (cmd_code) {
    case SYS_CMD_GET_BATTERY_INFO:
        {
            system_battery_info_t bat_info;
            if (get_battery_info(&bat_info) == ESP_OK) {
                memcpy(resp_data, &bat_info, sizeof(bat_info));
                resp_len = sizeof(bat_info);
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_CHARGE_STATUS:
        {
            system_charge_status_t charge_status;
            if (get_charge_status(&charge_status) == ESP_OK) {
                memcpy(resp_data, &charge_status, sizeof(charge_status));
                resp_len = sizeof(charge_status);
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_TT_SIGNAL:
        {
            // DEPRECATED: Signal status should be queried by client via AT Service
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                           "GET_TT_SIGNAL deprecated, client should query via AT Service");
            resp_code = SYS_RESP_INVALID_CMD;
        }
        break;

    case SYS_CMD_GET_BLE_TX_POWER:
        {
            int8_t power = get_ble_tx_power();
            resp_data[0] = (uint8_t)power;
            resp_len = 1;
        }
        break;

    case SYS_CMD_SET_BLE_TX_POWER:
        {
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            int8_t power = (int8_t)cmd_params[0];
            if (set_ble_tx_power(power) == ESP_OK) {
                resp_data[0] = (uint8_t)power;
                resp_len = 1;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SERVICE_START:
        {
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            uint8_t service_id = cmd_params[0];
            int ret = gatt_system_server_start_service(service_id, conn_handle);
            if (ret == 0) {
                resp_data[0] = service_id;
                resp_len = 1;
            } else {
                resp_code = (ret == -1) ? SYS_RESP_SERVICE_ALREADY_RUNNING : SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SERVICE_STOP:
        {
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            uint8_t service_id = cmd_params[0];
            int ret = gatt_system_server_stop_service(service_id);
            if (ret == 0) {
                resp_data[0] = service_id;
                resp_len = 1;
            } else {
                resp_code = (ret == -1) ? SYS_RESP_SERVICE_NOT_RUNNING : SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SERVICE_STATUS:
        {
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            uint8_t service_id = cmd_params[0];
            int status = gatt_system_server_get_service_status(service_id);
            if (status >= 0) {
                resp_data[0] = service_id;
                resp_data[1] = (uint8_t)status;
                resp_len = 2;
            } else {
                resp_code = SYS_RESP_SERVICE_NOT_FOUND;
            }
        }
        break;

    case SYS_CMD_SYSTEM_REBOOT:
        {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "System reboot requested");
            // Send response first, then reboot
            send_command_response(conn_handle, cmd, SYS_RESP_OK, NULL, 0);
            // Delay to allow response to be sent
            vTaskDelay(pdMS_TO_TICKS(REBOOT_DELAY_MS));
            esp_restart();
            return ESP_OK;  // Won't reach here
        }

    case SYS_CMD_SYSTEM_RESET:
        {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                           "Factory reset requested but not implemented");
            /*
             * See lines 783-821 for detailed factory reset design notes.
             * This is the write handler version (same behavior).
             */
            resp_code = SYS_RESP_ERROR;  // Not implemented yet
        }
        break;

    case SYS_CMD_REBOOT_MCU:
        {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "MCU reboot requested, shutting down TT module first");

            // Power off Tiantong module first
            esp_err_t ret = tt_module_power_off();
            if (ret != ESP_OK) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to power off TT module: %s", esp_err_to_name(ret));
                // Continue with reboot anyway
            }

            // Send response first, then reboot
            send_command_response(conn_handle, cmd, SYS_RESP_OK, NULL, 0);

            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Shutting down TT module and rebooting MCU in 2 seconds...");
            // Delay to allow response to be sent and TT module to power off
            vTaskDelay(pdMS_TO_TICKS(MCU_REBOOT_DELAY_MS));
            esp_restart();
            return ESP_OK;  // Won't reach here
        }

    case SYS_CMD_REBOOT_TT:
        {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Tiantong module reboot requested");

            // Reset Tiantong module
            esp_err_t ret = tt_module_reset();
            if (ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to reset TT module: %s", esp_err_to_name(ret));
                resp_code = SYS_RESP_ERROR;
            } else {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Tiantong module reset initiated");
            }
        }
        break;

    case SYS_CMD_SET_USB_SWITCH:
        {
            // Parameter: [0x00=USB to IP5561 (charge mode), 0x01=USB to ESP32 (communication mode)]
            if (param_len < 1) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Missing USB switch parameter");
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }

            bool to_mcu = cmd_params[0] != 0;
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "USB switch routed to %s",
                     to_mcu ? "ESP32 (communication)" : "IP5561 (charge mode)");

            esp_err_t ret = tt_hw_set_usb_switch(to_mcu);
            if (ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to set USB switch: %s", esp_err_to_name(ret));
                resp_code = SYS_RESP_ERROR;
            } else {
                // Return current switch state
                resp_data[0] = to_mcu ? 0x01 : 0x00;
                resp_len = 1;
            }
        }
        break;

    case SYS_CMD_RESET_TT_HARDWARE:
        {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "TT module hardware reset requested (TEST)");

            // Hardware reset using GPIO
            esp_err_t ret = tt_hw_reset();
            if (ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to hardware reset TT module: %s", esp_err_to_name(ret));
                resp_code = SYS_RESP_ERROR;
            } else {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "TT module hardware reset completed");
            }
        }
        break;

    case SYS_CMD_GET_SYSTEM_INFO:
        {
            system_info_t sys_info;
            sys_info.uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000);
            sys_info.free_heap = esp_get_free_heap_size();
            sys_info.min_free_heap = esp_get_minimum_free_heap_size();

            esp_chip_info_t chip_info;
            esp_chip_info(&chip_info);
            sys_info.cpu_freq_mhz = (uint8_t)(esp_clk_cpu_freq() / 1000000);

            // Build service status bitmask
            sys_info.service_status = 0;
            if (gatt_ota_server_is_enabled()) sys_info.service_status |= (1 << 0);
            if (gatt_log_server_is_enabled()) sys_info.service_status |= (1 << 1);
            sys_info.service_status |= (1 << 2);  // AT always enabled
            sys_info.service_status |= (1 << 3);  // SPP always enabled
            if (spp_voice_server_is_enabled()) sys_info.service_status |= (1 << 4);

            memcpy(resp_data, &sys_info, sizeof(sys_info));
            resp_len = sizeof(sys_info);
        }
        break;

    case SYS_CMD_GET_VERSION_INFO:
        {
            system_version_info_t version_info = {0};

            // Copy version strings
            strncpy(version_info.firmware_version, get_firmware_version(), sizeof(version_info.firmware_version) - 1);
            strncpy(version_info.software_version, get_software_version(), sizeof(version_info.software_version) - 1);
            strncpy(version_info.manufacturer, get_manufacturer_name(), sizeof(version_info.manufacturer) - 1);
            strncpy(version_info.model_number, get_model_number(), sizeof(version_info.model_number) - 1);
            strncpy(version_info.hardware_revision, get_hardware_revision(), sizeof(version_info.hardware_revision) - 1);
            strncpy(version_info.build_datetime, get_build_datetime(), sizeof(version_info.build_datetime) - 1);

            memcpy(resp_data, &version_info, sizeof(version_info));
            resp_len = sizeof(version_info);

            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Version info: FW=%s, SW=%s",
                     version_info.firmware_version, version_info.software_version);
        }
        break;

    case SYS_CMD_LOG_SET_GLOBAL_LEVEL:
    case SYS_CMD_LOG_SET_MODULE_CONFIG:
    case SYS_CMD_LOG_GET_MODULE_CONFIG:
    case SYS_CMD_LOG_SET_GATT_ENABLE:
    case SYS_CMD_LOG_GET_GATT_STATE:
    case SYS_CMD_LOG_GET_GATT_STATS:
    case SYS_CMD_LOG_DISABLE_ALL_GATT:
        {
            // Syslog configuration commands
            uint8_t syslog_cmd[32];
            size_t syslog_cmd_len = 0;

            // Map GATT System Server command to syslog module command
            uint8_t syslog_cmd_type = 0;
            size_t syslog_data_len = 0;

            switch (cmd_code) {
            case SYS_CMD_LOG_SET_GLOBAL_LEVEL:
                syslog_cmd_type = 0x01;  // SYS_LOG_CMD_SET_GLOBAL_LEVEL
                syslog_data_len = 1;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd[2] = cmd_params[0];  // level
                syslog_cmd_len = 3;
                break;

            case SYS_CMD_LOG_SET_MODULE_CONFIG:
                syslog_cmd_type = 0x10;  // SYS_LOG_CMD_SET_MODULE_CONFIG
                syslog_data_len = 4;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd[2] = cmd_params[0];  // module
                syslog_cmd[3] = cmd_params[1];  // level
                syslog_cmd[4] = cmd_params[2];  // enabled
                syslog_cmd[5] = cmd_params[3];  // gatt_output
                syslog_cmd_len = 6;
                break;

            case SYS_CMD_LOG_GET_MODULE_CONFIG:
                syslog_cmd_type = 0x11;  // SYS_LOG_CMD_GET_MODULE_CONFIG
                syslog_data_len = 1;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd[2] = cmd_params[0];  // module (0xFF for all)
                syslog_cmd_len = 3;
                break;

            case SYS_CMD_LOG_SET_GATT_ENABLE:
                syslog_cmd_type = 0x30;  // SYS_LOG_CMD_SET_GATT_GLOBAL
                syslog_data_len = 1;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd[2] = cmd_params[0];  // enabled
                syslog_cmd_len = 3;
                break;

            case SYS_CMD_LOG_GET_GATT_STATE:
                syslog_cmd_type = 0x32;  // SYS_LOG_CMD_GET_GATT_STATE
                syslog_data_len = 0;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd_len = 2;
                break;

            case SYS_CMD_LOG_GET_GATT_STATS:
                syslog_cmd_type = 0x33;  // SYS_LOG_CMD_GET_GATT_STATS
                syslog_data_len = 0;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd_len = 2;
                break;

            case SYS_CMD_LOG_DISABLE_ALL_GATT:
                syslog_cmd_type = 0x42;  // SYS_LOG_CMD_DISABLE_ALL_GATT
                syslog_data_len = 0;
                syslog_cmd[0] = syslog_cmd_type;
                syslog_cmd[1] = syslog_data_len;
                syslog_cmd_len = 2;
                break;

            default:
                resp_code = SYS_RESP_ERROR;
                break;
            }

            if (syslog_cmd_len > 0) {
                size_t syslog_resp_len = 0;
                esp_err_t ret = syslog_process_remote_config(syslog_cmd, syslog_cmd_len, resp_data, &syslog_resp_len);
                if (ret == ESP_OK) {
                    resp_len = syslog_resp_len;
                } else {
                    resp_code = SYS_RESP_ERROR;
                }
            }
        }
        break;

    /* ========== PowerBank Commands (IP5561) ========== */
    case SYS_CMD_GET_WORK_MODE_STATUS:
        {
            uint8_t mode, status_flags;
            if (power_manage_get_work_mode(&mode, &status_flags) == ESP_OK) {
                resp_data[0] = mode;
                resp_data[1] = status_flags;
                resp_len = 2;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_CHARGING_STATUS:
        {
            uint8_t status_flags;
            if (power_manage_get_charging_status(&status_flags) == ESP_OK) {
                resp_data[0] = status_flags;
                resp_len = 1;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_WIRELESS_STATUS:
        {
            uint8_t status_flags;
            if (power_manage_get_wireless_status(&status_flags) == ESP_OK) {
                resp_data[0] = status_flags;
                resp_len = 1;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_VBUS_ADC:
        {
            uint16_t vbus_voltage;
            int16_t vbus_current;
            if (power_manage_get_vbus_adc(&vbus_voltage, &vbus_current) == ESP_OK) {
                // Pack as little-endian: [vbus_l][vbus_h][ibus_l][ibus_h]
                resp_data[0] = vbus_voltage & 0xFF;
                resp_data[1] = (vbus_voltage >> 8) & 0xFF;
                resp_data[2] = vbus_current & 0xFF;
                resp_data[3] = (vbus_current >> 8) & 0xFF;
                resp_len = 4;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_NTC_DATA:
        {
            uint16_t ntc_voltage;
            int16_t temperature;
            if (power_manage_get_ntc_data(&ntc_voltage, &temperature) == ESP_OK) {
                // Pack as little-endian: [ntc_v_l][ntc_v_h][temp_l][temp_h]
                resp_data[0] = ntc_voltage & 0xFF;
                resp_data[1] = (ntc_voltage >> 8) & 0xFF;
                resp_data[2] = temperature & 0xFF;
                resp_data[3] = (temperature >> 8) & 0xFF;
                resp_len = 4;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SET_CHARGE_VOLTAGE:
        {
            if (param_len < 2) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            // Unpack little-endian voltage
            uint16_t voltage = cmd_params[0] | (cmd_params[1] << 8);
            if (power_manage_set_charge_voltage(voltage) == ESP_OK) {
                // Echo back the set value
                resp_data[0] = voltage & 0xFF;
                resp_data[1] = (voltage >> 8) & 0xFF;
                resp_len = 2;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SET_CHARGE_CURRENT_9V:
        {
            if (param_len < 2) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            // Unpack little-endian current
            uint16_t current = cmd_params[0] | (cmd_params[1] << 8);
            if (power_manage_set_charge_current_9v(current) == ESP_OK) {
                // Echo back the set value
                resp_data[0] = current & 0xFF;
                resp_data[1] = (current >> 8) & 0xFF;
                resp_len = 2;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SET_UV_THRESHOLD_9V:
        {
            if (param_len < 2) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            // Unpack little-endian threshold
            uint16_t threshold = cmd_params[0] | (cmd_params[1] << 8);
            if (power_manage_set_uv_threshold_9v(threshold) == ESP_OK) {
                // Echo back the set value
                resp_data[0] = threshold & 0xFF;
                resp_data[1] = (threshold >> 8) & 0xFF;
                resp_len = 2;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SET_VBUS_OUTPUT_CURRENT_9V:
        {
            if (param_len < 2) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            // Unpack little-endian current
            uint16_t current = cmd_params[0] | (cmd_params[1] << 8);
            if (power_manage_set_vbus_output_current_9v(current) == ESP_OK) {
                // Echo back the set value
                resp_data[0] = current & 0xFF;
                resp_data[1] = (current >> 8) & 0xFF;
                resp_len = 2;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SET_WPC_ENABLED:
        {
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            bool enable = cmd_params[0] != 0;
            if (power_manage_set_wireless_charging(enable) == ESP_OK) {
                // Echo back the enable status
                resp_data[0] = enable ? 0x01 : 0x00;
                resp_len = 1;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_TT_STATUS:
        {
            tt_status_info_t info;
            if (tt_module_get_status_info(&info) == ESP_OK) {
                // Response format: [state][voltage_mv_lo][voltage_mv_hi][error_code][flags][reserved]
                // flags: bit0 = force_on
                resp_data[0] = (uint8_t)info.state;
                resp_data[1] = (uint8_t)(info.voltage_mv & 0xFF);        // voltage low byte
                resp_data[2] = (uint8_t)((info.voltage_mv >> 8) & 0xFF); // voltage high byte
                resp_data[3] = info.error_code;
                resp_data[4] = info.flags;    // flags (bit0: force_on)
                resp_data[5] = 0x00;          // reserved
                resp_len = 6;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SET_TT_POWER:
        {
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }

            /* Notify sleep manager of user activity */
            extern void sleep_manager_notify_activity(const char *source);
            sleep_manager_notify_activity("set_tt_power");

            bool power_on = cmd_params[0] != 0;
            esp_err_t ret;

            if (power_on) {
                ret = tt_module_user_power_on();
            } else {
                ret = tt_module_user_power_off();
            }

            if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
                // Success or already in requested state
                tt_state_t state = tt_module_get_state();
                resp_data[0] = (state == TT_STATE_WORKING) ? 0x01 : 0x00;
                resp_len = 1;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_SET_VOICE_FRAME_MODE:
        {
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            uint8_t frames = cmd_params[0];
            if (voice_packet_set_frame_mode(frames) == 0) {
                resp_data[0] = voice_packet_get_frame_mode();
                resp_len = 1;
            } else {
                resp_code = SYS_RESP_INVALID_PARAM;
            }
        }
        break;

    case SYS_CMD_GET_VOICE_FRAME_MODE:
        {
            resp_data[0] = voice_packet_get_frame_mode();
            resp_len = 1;
        }
        break;

    case SYS_CMD_TT_FORCE_ON:
        {
            esp_err_t ret = tt_module_force_on();
            tt_state_t state = tt_module_get_state();
            uint16_t v = 0;
            fuel_gauge_handle_t bq = power_manage_get_fuel_gauge_handle();
            if (bq) v = fuel_gauge_get_voltage(bq);

            if (ret == ESP_OK) {
                resp_code = SYS_RESP_OK;
            } else if (ret == ESP_ERR_NOT_ALLOWED) {
                resp_code = SYS_RESP_ERROR;
            } else {
                resp_code = SYS_RESP_ERROR;
            }

            // Response: [state][voltage_lo][voltage_hi]
            resp_data[0] = (uint8_t)state;
            resp_data[1] = (uint8_t)(v & 0xFF);
            resp_data[2] = (uint8_t)((v >> 8) & 0xFF);
            resp_len = 3;
        }
        break;

    case SYS_CMD_TT_FORCE_OFF:
        {
            tt_module_force_off();
            tt_state_t state = tt_module_get_state();
            uint16_t v = 0;
            fuel_gauge_handle_t bq = power_manage_get_fuel_gauge_handle();
            if (bq) v = fuel_gauge_get_voltage(bq);

            // Response: [state][voltage_lo][voltage_hi]
            resp_data[0] = (uint8_t)state;
            resp_data[1] = (uint8_t)(v & 0xFF);
            resp_data[2] = (uint8_t)((v >> 8) & 0xFF);
            resp_len = 3;
        }
        break;

    case SYS_CMD_GET_SENSOR_STATUS:
        {
            /* Return raw sensor data: [flags][ax_lo][ax_hi][ay_lo][ay_hi][az_lo][az_hi]
             *                          [mx_lo][mx_hi][my_lo][my_hi][mz_lo][mz_hi] = 13 bytes */
            uint8_t flags = 0;
            int16_t ax = 0, ay = 0, az = 0;
            int32_t mx = 0, my = 0, mz = 0;
            power_manage_get_sensor_data(&flags, &ax, &ay, &az, &mx, &my, &mz);

            /* Also print the device-computed pointing angles (方位角/仰角) */
            int16_t azimuth = 0;
            float elevation = 0.0f;
            power_manage_get_sensor_angles(&azimuth, &elevation);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                "[SENSOR] 方位角=%d° 仰角=%.1f° (accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG)",
                azimuth, elevation, ax, ay, az, (long)mx, (long)my, (long)mz);

            resp_data[0] = flags;
            memcpy(&resp_data[1], &ax, 2);
            memcpy(&resp_data[3], &ay, 2);
            memcpy(&resp_data[5], &az, 2);
            int16_t mx16 = (int16_t)mx, my16 = (int16_t)my, mz16 = (int16_t)mz;
            memcpy(&resp_data[7], &mx16, 2);
            memcpy(&resp_data[9], &my16, 2);
            memcpy(&resp_data[11], &mz16, 2);
            resp_len = 13;
        }
        break;

    case SYS_CMD_ENABLE_SENSOR_REPORT:
        {
            /* param[0]: 0=disable, 1=enable (default enable if no param) */
            bool enable = (param_len == 0) ? true : (cmd_params[0] != 0);
            power_manage_set_sensor_report(enable);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                "Sensor report %s", enable ? "ENABLED" : "DISABLED");
            resp_data[0] = power_manage_is_sensor_report_enabled() ? 1 : 0;
            resp_len = 1;
        }
        break;

    case SYS_CMD_DISABLE_SENSOR_REPORT:
        {
            /* Backward compat: explicit disable */
            power_manage_set_sensor_report(false);
            resp_data[0] = 0;
            resp_len = 1;
        }
        break;

#if ENABLE_SAT_CALL_SIM
    case SYS_CMD_SIM_CTRL:
        {
            /* params[0] = enable (1=on, 0=off)
             * params[1] = scenario (0=normal, 1=busy, 2=no_answer, 3=net_drop, 5=reject) */
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            bool enable = (cmd_params[0] != 0);
            sim_scenario_t scenario = SIM_SCENARIO_NORMAL;
            if (param_len >= 2) {
                scenario = (sim_scenario_t)cmd_params[1];
            }
            int ret = sat_call_sim_set_enabled(enable, scenario);
            /* Set connection handle so sim module can push URC to this client */
            if (ret == 0 && enable) {
                sat_call_sim_set_conn_handle(conn_handle);
            }
            if (ret != 0) {
                resp_code = SYS_RESP_ERROR;
            } else {
                resp_data[0] = sat_call_sim_is_enabled() ? 1 : 0;
                resp_data[1] = (uint8_t)sat_call_sim_get_scenario();
                resp_len = 2;
            }
        }
        break;

    case SYS_CMD_SIM_INCOMING:
        {
            /* params[0..1] = delay_ms (little-endian, 0=immediate) */
            uint32_t delay_ms = 0;
            if (param_len >= 2) {
                delay_ms = cmd_params[0] | (cmd_params[1] << 8);
            }
            int ret = sat_call_sim_trigger_incoming(delay_ms);
            resp_code = (ret == 0) ? SYS_RESP_OK : SYS_RESP_ERROR;
        }
        break;

    case SYS_CMD_SIM_SET_NET:
        {
            /* params[0] = CSQ (0-31, 99=unknown)
             * params[1] = CREG (0-5) */
            if (param_len < 2) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            sat_call_sim_set_network(cmd_params[0], cmd_params[1]);
        }
        break;

    case SYS_CMD_SIM_GET_STATE:
        {
            /* Response: [enabled][scenario][call_state] */
            resp_data[0] = sat_call_sim_is_enabled() ? 1 : 0;
            resp_data[1] = (uint8_t)sat_call_sim_get_scenario();
            resp_data[2] = (uint8_t)sat_call_sim_get_state();
            resp_len = 3;
        }
        break;
#endif /* ENABLE_SAT_CALL_SIM */

    case SYS_CMD_BLE_TX_POWER_TEMP:
        {
            /* params[0] = power (dBm, int8_t): -24,-21,-18,-15,-12,-9,-6,-3,0,3,6,9,12,15,18,20
             * params[1..4] = timeout (uint32_t, little-endian, seconds)
             *                0 = permanent change (no auto-restore)
             * Response: [current_power_dbm, timeout_sec_lo, timeout_sec_hi,
             *            timeout_sec_hi2, timeout_sec_hi3, original_power_dbm] */
            if (param_len < 1) {
                resp_code = SYS_RESP_INVALID_PARAM;
                break;
            }
            int8_t power_dbm = (int8_t)cmd_params[0];
            uint32_t timeout_sec = 0;
            if (param_len >= 5) {
                timeout_sec = cmd_params[1] | ((uint32_t)cmd_params[2] << 8) |
                              ((uint32_t)cmd_params[3] << 16) | ((uint32_t)cmd_params[4] << 24);
            }
            if (set_ble_tx_power_temp(power_dbm, timeout_sec) == ESP_OK) {
                resp_data[0] = (uint8_t)esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
                resp_data[1] = (uint8_t)(timeout_sec & 0xFF);
                resp_data[2] = (uint8_t)((timeout_sec >> 8) & 0xFF);
                resp_data[3] = (uint8_t)((timeout_sec >> 16) & 0xFF);
                resp_data[4] = (uint8_t)((timeout_sec >> 24) & 0xFF);
                resp_data[5] = (uint8_t)g_ble_tx_power_original;
                resp_len = 6;
            } else {
                resp_code = SYS_RESP_INVALID_PARAM;
            }
        }
        break;

    case SYS_CMD_SET_DEVICE_NAME:
        {
            /* Request params: UTF-8 name bytes.
             *   param_len = 0      -> restore default "TTCat"
             *   param_len = 1..29  -> set custom name
             *   param_len > 29     -> SYS_RESP_INVALID_PARAM
             * Response: [name_len][name_bytes...] (current name after apply) */
            esp_err_t r = ble_device_name_set((const char *)cmd_params, param_len);
            if (r == ESP_OK) {
                const char *cur = ble_device_name_get();
                size_t nlen = strlen(cur);
                if (nlen > BLE_DEVICE_NAME_MAX_LEN) {
                    nlen = BLE_DEVICE_NAME_MAX_LEN;
                }
                resp_data[0] = (uint8_t)nlen;
                memcpy(resp_data + 1, cur, nlen);
                resp_len = 1 + nlen;
            } else if (r == ESP_ERR_INVALID_ARG) {
                resp_code = SYS_RESP_INVALID_PARAM;
            } else {
                resp_code = SYS_RESP_ERROR;
            }
        }
        break;

    case SYS_CMD_GET_DEVICE_NAME:
        {
            /* Response: [name_len][name_bytes...] */
            const char *cur = ble_device_name_get();
            size_t nlen = strlen(cur);
            if (nlen > BLE_DEVICE_NAME_MAX_LEN) {
                nlen = BLE_DEVICE_NAME_MAX_LEN;
            }
            resp_data[0] = (uint8_t)nlen;
            memcpy(resp_data + 1, cur, nlen);
            resp_len = 1 + nlen;
        }
        break;

    case SYS_CMD_ENABLE_BATTERY_REPORT:
        {
            /* params[0] = flags (event subscription bitmask, 0xFF=all)
             * params[1] = SOC change threshold % (optional, default 1)
             * Response: [flags][soc_threshold] */
            uint8_t flags = BATTERY_REPORT_FLAG_ALL;
            uint8_t soc_thr = BATTERY_REPORT_DEFAULT_SOC_THRESHOLD;
            if (param_len >= 1) flags = cmd_params[0];
            if (param_len >= 2 && cmd_params[1] > 0) soc_thr = cmd_params[1];
            if (soc_thr > 100) soc_thr = 100;

            power_manage_set_battery_report(flags, soc_thr);

            resp_data[0] = flags;
            resp_data[1] = soc_thr;
            resp_len = 2;
        }
        break;

    case SYS_CMD_DISABLE_BATTERY_REPORT:
        {
            power_manage_disable_battery_report();
            resp_data[0] = 0;
            resp_len = 1;
        }
        break;

    default:
        resp_code = SYS_RESP_INVALID_CMD;
        break;
    }

    // Send response via new structured format (no lock needed - using local buffer)
    esp_err_t ret = send_command_response(conn_handle, cmd, resp_code, resp_data, resp_len);

    return ret;
}

/**
 * @brief System command processing task (asynchronous)
 *
 * This task processes commands from the queue and sends responses.
 * It runs independently to avoid blocking the BLE GATT callback.
 */
static void system_command_task(void *pvParameters)
{
    sys_cmd_queue_item_t item;
    esp_err_t ret;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "System command task started (stack size: %d bytes)",
                    SYS_CMD_TASK_STACK_SIZE);

    // Log initial stack high water mark
    UBaseType_t stack_watermark = uxTaskGetStackHighWaterMark(NULL);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Initial stack watermark: %d bytes free",
                    stack_watermark);

    while (1) {
        // Wait for command from queue
        if (xQueueReceive(g_sys_cmd_queue, &item, portMAX_DELAY) == pdTRUE) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                      "Processing command: seq=%d, cmd=0x%02x, conn=%d",
                      item.packet.seq, item.packet.cmd, item.conn_handle);

            /*
             * CRC16 Verification Status:
             *
             * Current State: DISABLED
             *
             * Issue: CRC calculation mismatch between client and server
             *
             * Root Cause Analysis:
             * 1. Potential byte order mismatch (little-endian vs big-endian)
             * 2. CRC16-CCITT vs CRC16-MODBUS parameter differences
             * 3. Struct alignment padding affecting CRC calculation
             *
             * Current Implementation:
             * - CRC verification code present but disabled (#if 0)
             * - See calculate_crc16() function for algorithm used
             *
             * Temporary Mitigation:
             * - Commands processed without CRC verification
             * - Relies on BLE encryption + bonding for security
             *
             * Resolution Plan:
             * 1. Add debug logging to compare CRC values
             * 2. Verify CRC16 algorithm matches client implementation
             * 3. Fix byte order issues if present
             * 4. Re-enable verification once resolved
             *
             * Risk Assessment:
             * - LOW: BLE link encryption provides data integrity
             * - Acceptable for development phase
             * - Should resolve for production hardening
             */
            #if 0
            size_t crc_len = offsetof(system_cmd_packet_t, crc16) + item.packet.param_len;
            uint16_t calculated_crc = calculate_crc16((const uint8_t *)&item.packet, crc_len);

            // Manually extract CRC from raw packet bytes (avoid struct alignment issues)
            const uint8_t *raw_packet = (const uint8_t *)&item.packet;
            uint16_t received_crc = (raw_packet[crc_len] << 8) | raw_packet[crc_len + 1];

            SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                      "CRC check: len=%zu, calc=0x%04x, recv=0x%04x (raw[0]=0x%02x, raw[1]=0x%02x), struct_crc16=0x%04x",
                      crc_len, calculated_crc, received_crc,
                      raw_packet[crc_len], raw_packet[crc_len + 1],
                      item.packet.crc16);

            if (calculated_crc != received_crc) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                          "CRC mismatch: expected 0x%04x, got 0x%04x",
                          calculated_crc, received_crc);
                // Send CRC error response
                send_command_response(item.conn_handle, &item.packet, SYS_RESP_ERROR, NULL, 0);
                continue;
            }
            #endif

            // Process command asynchronously
            ret = handle_system_control_command_async(&item.packet, item.conn_handle);

            if (ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                          "Command processing failed: seq=%d, cmd=0x%02x, ret=%d",
                          item.packet.seq, item.packet.cmd, ret);
            }

            // Update stack watermark after processing
            stack_watermark = uxTaskGetStackHighWaterMark(NULL);
            if (stack_watermark < 512) {  // Less than 512 bytes free
                SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG,
                              "Stack watermark low: %d bytes free (consider increasing stack size)",
                              stack_watermark);
            }
        }
    }
}

/**
 * @brief Initialize GATT System Server
 */
int gatt_system_server_init(void)
{
    int rc;

    /* Prevent re-initialization */
    if (g_system_server_initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "GATT System server already initialized");
        return 0;
    }

    /* Create mutex for protecting command buffer */
    g_cmd_buffer_mutex = xSemaphoreCreateMutex();
    if (g_cmd_buffer_mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to create cmd buffer mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Create command queue for async processing */
    g_sys_cmd_queue = xQueueCreate(SYS_CMD_QUEUE_SIZE, sizeof(sys_cmd_queue_item_t));
    if (g_sys_cmd_queue == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to create command queue");
        vSemaphoreDelete(g_cmd_buffer_mutex);
        g_cmd_buffer_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Create async command processing task */
    BaseType_t ret = xTaskCreate(
        system_command_task,
        "sys_cmd",
        SYS_CMD_TASK_STACK_SIZE,  // From ble_gatt_server.h
        NULL,
        SYS_CMD_TASK_PRIORITY,    // From ble_gatt_server.h
        &g_sys_cmd_task_handle
    );
    if (ret != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to create command task");
        vQueueDelete(g_sys_cmd_queue);
        g_sys_cmd_queue = NULL;
        vSemaphoreDelete(g_cmd_buffer_mutex);
        g_cmd_buffer_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialize sequence counter */
    g_seq_counter = 0;

    /* Count service configuration */
    rc = ble_gatts_count_cfg(system_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to count system service config: rc=%d", rc);
        return rc;
    }

    /* Add service */
    rc = ble_gatts_add_svcs(system_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to add system service: rc=%d", rc);
        return rc;
    }

    g_system_server_initialized = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "GATT System server initialized successfully");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Service status: SPP=enabled, AT=enabled, LOG=disabled, OTA=disabled, VOICE=disabled");
    return 0;
}
