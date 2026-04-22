/*
 * gatt_system_server.h - GATT System Control Server
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef GATT_SYSTEM_SERVER_H
#define GATT_SYSTEM_SERVER_H

#include <stdint.h>
#include "host/ble_hs.h"

/* System Service UUIDs */
#define BLE_SVC_SYSTEM_UUID16                0xABFC
#define BLE_SVC_SYSTEM_CHR_CONTROL_UUID16    0xABFD  // Control characteristic (system commands)
#define BLE_SVC_SYSTEM_CHR_INFO_UUID16       0xABFE  // Info characteristic (read system info)
#define BLE_SVC_SYSTEM_CHR_STATUS_UUID16     0xABFF  // Status characteristic (notify status changes)

/* System Control Commands */
#define SYS_CMD_GET_BATTERY_INFO         0x01  // Get battery information
#define SYS_CMD_GET_CHARGE_STATUS        0x02  // Get charging status
#define SYS_CMD_GET_TT_SIGNAL            0x03  // Get Tiantong module signal strength
#define SYS_CMD_GET_BLE_TX_POWER        0x04  // Get BLE TX power
#define SYS_CMD_SET_BLE_TX_POWER        0x05  // Set BLE TX power
#define SYS_CMD_SERVICE_START           0x10  // Start a service
#define SYS_CMD_SERVICE_STOP            0x11  // Stop a service
#define SYS_CMD_SERVICE_STATUS          0x12  // Get service status
#define SYS_CMD_SYSTEM_REBOOT           0x20  // Reboot system
#define SYS_CMD_SYSTEM_RESET            0x21  // Factory reset
#define SYS_CMD_REBOOT_MCU              0x22  // Reboot MCU (ESP32)
#define SYS_CMD_REBOOT_TT               0x23  // Reboot Tiantong module
#define SYS_CMD_SET_USB_SWITCH          0x24  // Set USB switch (0=USB to IP5561/charge, 1=USB to ESP32/comm)
#define SYS_CMD_RESET_TT_HARDWARE       0x25  // Hardware reset Tiantong module (for testing)
#define SYS_CMD_GET_SYSTEM_INFO         0x30  // Get system information
#define SYS_CMD_GET_VERSION_INFO        0x31  // Get version information

/* Syslog Configuration Commands */
#define SYS_CMD_LOG_SET_GLOBAL_LEVEL    0x40  // Set global log level
#define SYS_CMD_LOG_SET_MODULE_CONFIG   0x41  // Set module log config (level, gatt_output)
#define SYS_CMD_LOG_GET_MODULE_CONFIG   0x42  // Get module log config
#define SYS_CMD_LOG_SET_GATT_ENABLE     0x43  // Set GATT log global enable
#define SYS_CMD_LOG_GET_GATT_STATE      0x44  // Get GATT log state
#define SYS_CMD_LOG_GET_GATT_STATS      0x45  // Get GATT log buffer stats
#define SYS_CMD_LOG_DISABLE_ALL_GATT    0x46  // Disable all module GATT output

/* PowerBank Commands (IP5561) */
#define SYS_CMD_GET_WORK_MODE_STATUS    0x50  // Get work mode status (auto-switched by IP5561)
#define SYS_CMD_GET_CHARGING_STATUS     0x51  // Get charging/discharging status
#define SYS_CMD_GET_WIRELESS_STATUS     0x52  // Get wireless charging status
#define SYS_CMD_GET_VBUS_ADC            0x54  // Get VBUS ADC data (voltage/current)
#define SYS_CMD_GET_NTC_DATA            0x55  // Get NTC temperature data
#define SYS_CMD_SET_CHARGE_VOLTAGE      0x58  // Set charge voltage (4200-4400mV)
#define SYS_CMD_SET_CHARGE_CURRENT_9V   0x59  // Set 9V charge current limit
#define SYS_CMD_SET_UV_THRESHOLD_9V     0x5A  // Set 9V undervoltage threshold
#define SYS_CMD_SET_VBUS_OUTPUT_CURRENT_9V  0x5C  // Set VBUS 9V output current limit
#define SYS_CMD_SET_WPC_ENABLED         0x5E  // Enable/disable wireless charging

/* TT Module Status and Control Commands */
#define SYS_CMD_GET_TT_STATUS           0x60  // Get TT Module status
#define SYS_CMD_SET_TT_POWER            0x61  // Set TT Module power (control)

/* Voice Configuration Commands */
#define SYS_CMD_SET_VOICE_FRAME_MODE    0x70  // Set uplink voice frame mode (1 or 3)
#define SYS_CMD_GET_VOICE_FRAME_MODE    0x71  // Get uplink voice frame mode

/* System Command/Response Packet Configuration */
#define SYS_CMD_PACKET_MAX_PARAMS       96      // Maximum parameter bytes
#define SYS_CMD_PACKET_MAX_SIZE         (1 + 1 + SYS_CMD_PACKET_MAX_PARAMS + 2)  // seq + cmd + params + crc16

/* System Command Packet Structure */
typedef struct {
    uint8_t seq;                       // Sequence number (0-255, wraps around)
    uint8_t cmd;                       // Command code
    uint8_t param_len;                 // Parameter length
    uint8_t params[SYS_CMD_PACKET_MAX_PARAMS];  // Parameters
    uint16_t crc16;                    // CRC16-CCITT checksum
} __attribute__((packed)) system_cmd_packet_t;

/* System Response Packet Structure */
typedef struct {
    uint8_t seq;                       // Sequence number (echoed from command)
    uint8_t cmd;                       // Command code (echoed from command)
    uint8_t resp_code;                 // Response code
    uint8_t data_len;                  // Response data length
    uint8_t data[SYS_CMD_PACKET_MAX_PARAMS];     // Response data
    uint16_t crc16;                    // CRC16-CCITT checksum
} __attribute__((packed)) system_resp_packet_t;

/* Service IDs for dynamic control */
#define SYS_SERVICE_ID_OTA              0x01  // OTA service
#define SYS_SERVICE_ID_LOG              0x02  // Log service
#define SYS_SERVICE_ID_AT               0x03  // AT service (always running)
#define SYS_SERVICE_ID_SPP              0x04  // SPP voice service (voice data transmission)
#define SYS_SERVICE_ID_VOICE            0x05  // Voice task control (voice_packet_send_task)

/* System Response Codes */
#define SYS_RESP_OK                     0x00
#define SYS_RESP_ERROR                  0x01
#define SYS_RESP_INVALID_CMD            0x02
#define SYS_RESP_INVALID_PARAM          0x03
#define SYS_RESP_SERVICE_NOT_FOUND      0x04
#define SYS_RESP_SERVICE_ALREADY_RUNNING 0x05
#define SYS_RESP_SERVICE_NOT_RUNNING    0x06

/* Battery Information Structure */
typedef struct {
    uint16_t voltage_mv;          // Battery voltage in mV
    int16_t current_ma;           // Battery current in mA (positive=charging, negative=discharging)
    uint16_t soc_percent;         // State of charge (0-100%)
    uint16_t soh_percent;         // State of health (0-100%)
    uint16_t temperature_0_1k;    // Temperature in 0.1°K
    uint16_t full_charge_capacity_mah;  // Full charge capacity in mAh
    uint16_t remaining_capacity_mah;   // Remaining capacity in mAh
    uint8_t charging;             // 1 if charging, 0 if not
    uint8_t full_charged;          // 1 if fully charged, 0 if not
} __attribute__((packed)) system_battery_info_t;

/* Charging Status Structure */
typedef struct {
    uint8_t is_charging;          // 1 if charging, 0 if not
    uint8_t is_full;              // 1 if fully charged, 0 if not
    uint16_t charge_voltage_mv;   // Charge voltage in mV
    uint16_t charge_current_ma;   // Charge current in mA
    uint16_t time_to_full_min;    // Time to full charge in minutes
} __attribute__((packed)) system_charge_status_t;

/* Tiantong Module Signal Information */
typedef struct {
    int8_t rssi;                  // Signal strength in dBm
    uint8_t reg_status;           // Registration status
    uint8_t sim_status;           // SIM status
    uint8_t network_type;         // Network type
} __attribute__((packed)) system_tt_signal_t;

/* System Information Structure */
typedef struct {
    uint32_t uptime_sec;          // System uptime in seconds
    uint32_t free_heap;           // Free heap memory in bytes
    uint32_t min_free_heap;       // Minimum free heap in bytes
    uint8_t cpu_freq_mhz;         // CPU frequency in MHz
    uint8_t service_status;       // Bitmask of service status (bit 0=OTA, bit 1=LOG, bit 2=AT, bit 3=SPP, bit 4=VOICE)
} __attribute__((packed)) system_info_t;

/* Version Information Structure */
typedef struct {
    char firmware_version[16];   // Firmware version string (e.g., "1.0.0")
    char software_version[24];   // Software version string (e.g., "TTSat-1.0.0")
    char manufacturer[16];        // Manufacturer name
    char model_number[16];        // Model number
    char hardware_revision[8];   // Hardware revision
    char build_datetime[16];      // Build date and time (YYYY-MM-DD)
} __attribute__((packed)) system_version_info_t;

/**
 * @brief Initialize the GATT System server
 *
 * @return 0 on success, negative error code otherwise
 */
int gatt_system_server_init(void);

/**
 * @brief Send system status notification
 *
 * @param conn_handle BLE connection handle
 * @param status Status code
 * @param data Optional status data
 * @param len Data length
 * @return 0 on success, negative error code otherwise
 */
int gatt_system_server_send_status(uint16_t conn_handle, uint8_t status, const uint8_t *data, size_t len);

/**
 * @brief Start a GATT service dynamically
 *
 * @param service_id Service ID (SYS_SERVICE_ID_*)
 * @return 0 on success, negative error code otherwise
 */
int gatt_system_server_start_service(uint8_t service_id);

/**
 * @brief Stop a GATT service dynamically
 *
 * @param service_id Service ID (SYS_SERVICE_ID_*)
 * @return 0 on success, negative error code otherwise
 */
int gatt_system_server_stop_service(uint8_t service_id);

/**
 * @brief Get service status
 *
 * @param service_id Service ID (SYS_SERVICE_ID_*)
 * @return 1 if running, 0 if stopped, -1 if error
 */
int gatt_system_server_get_service_status(uint8_t service_id);

#endif /* GATT_SYSTEM_SERVER_H */
