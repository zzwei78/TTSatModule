/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== IP5561 Protection Feature Switches ========== */
/* Set to 1 to enable, 0 to disable. Defaults can be overridden in sdkconfig. */
#ifndef IP5561_ENABLE_LIGHT_LOAD_PROT
#define IP5561_ENABLE_LIGHT_LOAD_PROT    0   /* Light-load auto-shutdown */
#endif
#ifndef IP5561_ENABLE_CHARGE_TIMEOUT
#define IP5561_ENABLE_CHARGE_TIMEOUT     0   /* Charge safety timeout */
#endif
#ifndef IP5561_ENABLE_NTC_PROT
#define IP5561_ENABLE_NTC_PROT           0   /* NTC temperature protection */
#endif
#ifndef IP5561_ENABLE_BOOST_PROT
#define IP5561_ENABLE_BOOST_PROT         0   /* Boost OCP/OVP protection */
#endif
#include "IP5561.h"
#include "fuel_gauge.h"
#include "da228ec.h"
#include "sc7a20h.h"
#include "mmc5603.h"
#include "config/hardware_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Battery Voltage Thresholds for TT Module Power Control ========== */

/**
 * @brief TT module power OFF threshold
 *
 * When battery voltage drops below this threshold, TT module will be powered off.
 * Unit: millivolts (mV)
 */
#define POWER_MANAGE_TT_MODULE_V_OFF_MV     3100    /* 3.1V - Turn OFF threshold (discharging) */

/**
 * @brief TT module power ON threshold
 *
 * When battery voltage rises to or above this threshold, TT module will be powered on.
 * This threshold is higher than V_OFF to provide hysteresis and prevent oscillation.
 * Unit: millivolts (mV)
 */
#define POWER_MANAGE_TT_MODULE_V_ON_MV      3300    /* 3.3V - Turn ON threshold (discharging) */

/**
 * @brief TT module power OFF threshold during charging
 *
 * When battery voltage drops below this threshold while charging, TT module will be powered off.
 * This threshold is higher than V_OFF to compensate for charging IR voltage drop.
 * Unit: millivolts (mV)
 */
#define POWER_MANAGE_TT_MODULE_V_OFF_CHG_MV  3700    /* 3.7V - Turn OFF threshold (charging) */

/**
 * @brief TT module power ON threshold during charging
 *
 * When battery voltage rises to or above this threshold while charging, TT module will be powered on.
 * This threshold is higher than V_ON to compensate for charging IR voltage drop.
 * Unit: millivolts (mV)
 */
#define POWER_MANAGE_TT_MODULE_V_ON_CHG_MV   3800    /* 3.8V - Turn ON threshold (charging) */

/**
 * @brief Battery internal resistance for IR compensation
 *
 * Typical Li-ion battery internal resistance: 100-200mΩ
 * Used to compensate charging IR voltage drop.
 * Unit: milliohms (mΩ)
 */
#define POWER_MANAGE_BATT_INTERNAL_R_MOHM    10      /* 10mΩ internal resistance (consistent with fuel_gauge.c) */

/**
 * @brief Maximum IR compensation voltage
 *
 * Limit the maximum IR voltage drop compensation to prevent over-compensation.
 * Unit: millivolts (mV)
 */
#define POWER_MANAGE_MAX_IR_COMP_MV         300     /* Maximum 300mV IR compensation */

/**
 * @brief Low battery warning threshold
 *
 * When battery voltage drops below this threshold (during charging), a warning will be logged.
 * This provides early warning before the module is powered off.
 * Unit: millivolts (mV)
 */
#define POWER_MANAGE_TT_MODULE_V_WARN_MV    3700    /* 3.7V - Low voltage warning */

/**
 * @brief Hysteresis voltage
 *
 * Voltage difference between ON and OFF thresholds to prevent rapid switching.
 * Unit: millivolts (mV)
 */
#define POWER_MANAGE_TT_MODULE_V_HYSTERESIS_MV  100    /* 0.1V hysteresis */

/**
 * @brief Power management initialization
 *
 * This function initializes the I2C bus and both power management devices:
 * - IP5561: Charging management chip
 * - BQ27220: Battery fuel gauge
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if I2C bus initialization failed
 *     - ESP_FAIL if device initialization failed
 */
esp_err_t power_manage_init(void);

/**
 * @brief Deinitialize power management
 *
 * @return
 *     - ESP_OK if successful
 */
esp_err_t power_manage_deinit(void);

/**
 * @brief Start all power monitor tasks
 *
 * Starts monitor tasks for both IP5561 and BQ27220.
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if not initialized
 *     - ESP_FAIL if tasks already running
 */
esp_err_t power_manage_task_start(void);

/**
 * @brief Stop all power monitor tasks
 *
 * @return
 *     - ESP_OK if successful
 */
esp_err_t power_manage_task_stop(void);

/**
 * @brief Get IP5561 device handle
 *
 * @return IP5561 handle, or NULL if not initialized
 */
ip5561_handle_t power_manage_get_ip5561_handle(void);

/**
 * @brief Get fuel gauge device handle
 *
 * @return Fuel gauge handle, or NULL if not initialized
 */
fuel_gauge_handle_t power_manage_get_fuel_gauge_handle(void);

/**
 * @brief Check if battery is currently charging
 *
 * Uses IP5561 charging flag (authoritative) when available,
 * falls back to fuel gauge current direction if IP5561 is not initialized.
 *
 * @return true if charging, false otherwise
 */
bool power_manage_is_charging(void);

/**
 * @brief Get I2C bus handle
 *
 * @return I2C bus handle, or NULL if not initialized
 */
i2c_master_bus_handle_t power_manage_get_i2c_bus(void);

/**
 * @brief Get DA228EC accelerometer handle
 *
 * @return DA228EC handle, or NULL if not detected at boot
 */
da228ec_handle_t power_manage_get_da228ec_handle(void);

/**
 * @brief Get MMC5603 magnetometer handle
 *
 * @return MMC5603 handle, or NULL if not detected at boot
 */
mmc5603_handle_t power_manage_get_mmc5603_handle(void);

/**
 * @brief Get sensor presence flags
 *
 * @return bitmask: bit0=accel, bit1=mag
 */
uint8_t power_manage_get_sensor_flags(void);

/**
 * @brief Get cached sensor data
 *
 * @param[out] flags sensor valid flags (bit0=accel, bit1=mag)
 * @param[out] ax,ay,az accelerometer data in mg (NULL ok if not needed)
 * @param[out] mx,my,mz magnetometer data in mG (NULL ok if not needed)
 */
void power_manage_get_sensor_data(uint8_t *flags,
                                   int16_t *ax, int16_t *ay, int16_t *az,
                                   int32_t *mx, int32_t *my, int32_t *mz);

/**
 * @brief Get computed pointing angles from cached sensor data
 *
 * 方位角 (azimuth):  horizontal compass heading, 0=North, 90=East, 180=South, 270=West
 * 仰角 (elevation): vertical tilt angle, 0=flat, + = tilted up, - = tilted down (range -90~+90)
 *
 * @param[out] azimuth   computed heading in degrees (NULL ok)
 * @param[out] elevation computed tilt in degrees (NULL ok)
 */
void power_manage_get_sensor_angles(int16_t *azimuth, float *elevation);

/**
 * @brief Enable/disable sensor data report via BLE
 */
void power_manage_set_sensor_report(bool enable);
bool power_manage_is_sensor_report_enabled(void);

/* ========== Battery Event Report ========== */

/**
 * @brief Enable battery event notifications
 *
 * When enabled, the power monitor task checks every 10 seconds for:
 * - SOC change >= threshold → push event
 * - Charging start/stop/full → push event
 * - Temperature high/low → push event (with hysteresis)
 *
 * @param flags        Event subscription bitmask (BATTERY_REPORT_FLAG_*)
 * @param soc_threshold SOC change threshold in percent (1-100)
 */
void power_manage_set_battery_report(uint8_t flags, uint8_t soc_threshold);

/**
 * @brief Disable battery event notifications
 */
void power_manage_disable_battery_report(void);

/* ========== PowerBank API (IP5561) ========== */

/**
 * @brief PowerBank work mode
 */
typedef enum {
    POWER_MANAGE_MODE_STANDBY = 0,  /* Standby mode */
    POWER_MANAGE_MODE_CHARGE = 1,   /* Charging mode (input to battery) */
    POWER_MANAGE_MODE_BOOST = 2     /* Power bank mode (output from battery) */
} power_manage_work_mode_t;

/**
 * @brief Get work mode status (auto-switched by IP5561)
 *
 * @param mode Output: work mode (0=standby, 1=charge, 2=boost)
 * @param status_flags Output: status flags (bit0=charge_en, bit1=boost_en, bit2=vbus_present)
 * @return ESP_OK on success
 */
esp_err_t power_manage_get_work_mode(uint8_t *mode, uint8_t *status_flags);

/**
 * @brief Get charging/discharging status
 *
 * @param status_flags Output: status flags
 *                        bit0=charger_en, bit1=boost_en, bit2=charging,
 *                        bit3=discharging, bit4=charge_done
 * @return ESP_OK on success
 */
esp_err_t power_manage_get_charging_status(uint8_t *status_flags);

/**
 * @brief Get wireless charging status
 *
 * @param status_flags Output: status flags
 *                        bit0=wpc_en, bit1=wpc_present, bit2=wpc_charging, bit3=wpc_done
 * @return ESP_OK on success
 */
esp_err_t power_manage_get_wireless_status(uint8_t *status_flags);

/**
 * @brief Get VBUS ADC data
 *
 * @param vbus_voltage Output: VBUS voltage in mV
 * @param vbus_current Output: VBUS current in mA (positive=input, negative=output)
 * @return ESP_OK on success
 */
esp_err_t power_manage_get_vbus_adc(uint16_t *vbus_voltage, int16_t *vbus_current);

/**
 * @brief Get NTC temperature data
 *
 * @param ntc_voltage Output: NTC voltage in mV
 * @param temperature Output: Temperature in 0.1°C units
 * @return ESP_OK on success
 */
esp_err_t power_manage_get_ntc_data(uint16_t *ntc_voltage, int16_t *temperature);

/**
 * @brief Get real battery temperature (from IP5561 NTC thermistor)
 *
 * CW2217E has no real temp sensor; battery temp comes from the IP5561 NTC1 pin.
 * @return Temperature in 0.1°K (same format as fuel_gauge_get_temperature)
 */
uint16_t power_manage_get_battery_temp_0_1k(void);

/**
 * @brief Set charge voltage
 *
 * @param voltage_mv Charge voltage in mV (4200-4400)
 * @return ESP_OK on success
 */
esp_err_t power_manage_set_charge_voltage(uint16_t voltage_mv);

/**
 * @brief Set 9V charge current limit
 *
 * @param current_ma Current limit in mA (500-3500)
 * @return ESP_OK on success
 */
esp_err_t power_manage_set_charge_current_9v(uint16_t current_ma);

/**
 * @brief Set 9V undervoltage threshold
 *
 * @param threshold_mv Threshold in mV (6000-9000)
 * @return ESP_OK on success
 */
esp_err_t power_manage_set_uv_threshold_9v(uint16_t threshold_mv);

/**
 * @brief Set VBUS 9V output current limit
 *
 * @param current_ma Output current limit in mA (500-2000)
 * @return ESP_OK on success
 */
esp_err_t power_manage_set_vbus_output_current_9v(uint16_t current_ma);

/**
 * @brief Enable/disable wireless charging
 *
 * @param enable true to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t power_manage_set_wireless_charging(bool enable);

/**
 * @brief Check if monitor tasks are running
 *
 * @return true if running, false otherwise
 */
bool power_manage_task_is_running(void);

/* ========== Wireless Charger Control Functions ========== */

/**
 * @brief Enable or disable wireless charger
 *
 * @param enable true to enable wireless charger, false to disable
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if IP5561 not initialized
 */
esp_err_t power_manage_set_wpc_enable(bool enable);

/**
 * @brief Get wireless charger enable status
 *
 * @return true if wireless charger enabled, false otherwise
 */
bool power_manage_get_wpc_enable(void);

/**
 * @brief Get wireless charger status
 *
 * @param status Pointer to store wireless charger status structure
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if IP5561 not initialized or status is NULL
 */
esp_err_t power_manage_get_wpc_status(ip5561_wpc_status_t *status);

/**
 * @brief Check if wireless power is present
 *
 * @return true if wireless power present, false otherwise
 */
bool power_manage_is_wpc_present(void);

/**
 * @brief Check if wireless charging is in progress
 *
 * @return true if wireless charging, false otherwise
 */
bool power_manage_is_wpc_charging(void);

/* ========== IP5561 Control API ========== */

/**
 * @brief IP5561 long press wakeup (2s KEY pulse)
 *
 * Wakes IP5561 from deep shutdown. Required for battery-only operation.
 */
void power_manage_ip5561_long_wakeup(void);

/**
 * @brief IP5561 double-click shutdown (simulate two short KEY presses)
 *
 * Triggers IP5561 power-off by simulating a physical double-click on KEY.
 */
void power_manage_ip5561_double_click_shutdown(void);

/**
 * @brief Enable/disable IP5561 VBUS boost output
 *
 * @param enable true to enable VBUS output, false to disable
 * @return ESP_OK on success
 */
esp_err_t power_manage_ip5561_set_vbus_output(bool enable);

/* ========== Battery SOC Calibration API ========== */

/**
 * @brief Get battery state of charge with IR compensation
 *
 * This function calculates SOC based on compensated voltage to improve accuracy
 * during charging. When charging, the terminal voltage is higher than the actual
 * battery voltage due to internal resistance (IR drop). This function compensates
 * for this effect to provide a more accurate SOC estimate.
 *
 * Calculation:
 * - If charging: V_comp = V_term - (|I_chg| × R_internal)
 * - If discharging: V_comp = V_term (no compensation)
 * - SOC% = (V_comp - 3.0V) / (4.35V - 3.0V) × 100
 *
 * @param soc_percent Output: calibrated SOC percentage (0-100)
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if soc_percent is NULL
 *     - ESP_ERR_INVALID_STATE if BQ27220 not initialized
 */
esp_err_t power_manage_get_calibrated_soc(uint16_t *soc_percent);

/**
 * @brief Get battery voltage with IR compensation (V_comp)
 *
 * This function returns the compensated battery voltage, which accounts for
 * internal resistance (IR) voltage drop during charging. When charging,
 * the terminal voltage is higher than the actual battery voltage due to IR drop.
 *
 * Calculation:
 * - If charging: V_comp = V_term - (|I_chg| × R_internal)
 * - If discharging: V_comp = V_term (no compensation)
 *
 * This provides a more accurate representation of the actual battery state
 * for reporting to clients.
 *
 * @param[out] voltage_mv Pointer to store compensated voltage in mV
 * @param[out] is_charging Pointer to store charging state (true=charging, false=discharging)
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if voltage_mv or is_charging is NULL
 *     - ESP_ERR_INVALID_STATE if BQ27220 not initialized
 */
esp_err_t power_manage_get_battery_voltage_compensated(uint16_t *voltage_mv, bool *is_charging);

/**
 * @brief Get cached average battery current (updated every 60s by BQ27220 monitor)
 *
 * Calculated from CoulombCounter delta over 60 seconds.
 * Positive = discharging, negative = charging.
 *
 * @return int16_t Average current in mA, 0 if not yet calculated
 */
int16_t power_manage_get_avg_current(void);

/* ========== Boost Power Manager (V2.0 only) ========== */
#ifdef SUPPORT_HARDWARE_V2

typedef enum {
    BOOST_CONSUMER_TT_MODULE = 0,   /* Tiantong module */
    BOOST_CONSUMER_MCU       = 1,   /* ESP32 low-battery survival */
    BOOST_CONSUMER_COUNT
} boost_consumer_t;

/**
 * @brief Request boost power (increments ref count, turns on boost if first)
 */
esp_err_t power_manage_boost_request(boost_consumer_t who);

/**
 * @brief Release boost power (decrements ref count, turns off if last)
 */
esp_err_t power_manage_boost_release(boost_consumer_t who);

/**
 * @brief Check if boost power is currently active
 */
bool power_manage_boost_is_active(void);

/**
 * @brief Prepare boost GPIOs for deep sleep (turn off + hold state)
 */
void power_manage_boost_deep_sleep_prepare(void);

/**
 * @brief Prepare IP5561 GPIOs for deep sleep
 *
 * Resets I2C1 SDA/SCL (GPIO6/GPIO14) to high-Z and holds KEY (GPIO8) LOW.
 * Called from sleep_manager before entering deep sleep.
 */
void power_manage_ip5561_deep_sleep_prepare(void);

#endif /* SUPPORT_HARDWARE_V2 */

#ifdef __cplusplus
}
#endif
