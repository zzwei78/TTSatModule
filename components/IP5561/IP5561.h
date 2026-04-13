/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IP5561 charging status structure
 */
typedef struct {
    bool chg_enabled;        /* Charger enabled */
    bool vbus_valid;         /* VBUS power present */
    bool chg_done;           /* Charging completed */
    bool charging;           /* Charging in progress */
    uint8_t chg_status;      /* Raw status register value */
} ip5561_chg_status_t;

/**
 * @brief IP5561 wireless charger status structure
 */
typedef struct {
    bool wpc_enabled;        /* Wireless charger enabled */
    bool wpc_present;        /* Wireless power present */
    bool wpc_charging;       /* Wireless charging in progress */
    bool wpc_done;           /* Wireless charging completed */
    uint8_t wpc_ctl0;        /* Raw VWPC_CTL0 register value */
} ip5561_wpc_status_t;

/**
 * @brief IP5561 configuration structure
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;   /* I2C bus handle */
    uint32_t scl_freq_hz;              /* I2C frequency (default: 200kHz) */
} ip5561_config_t;

typedef void *ip5561_handle_t;

/**
 * @brief IP5561 wakeup callback function type
 *
 * This callback is invoked before each I2C communication to ensure the chip is awake.
 * Wakeup sequence should be: LOW -> HIGH -> LOW (each state ~10ms)
 */
typedef void (*ip5561_wakeup_callback_t)(void);

/**
 * @brief Set wakeup callback for IP5561
 *
 * When set, this callback will be invoked before every I2C read/write operation
 * to ensure the IP5561 chip is in awake state.
 *
 * @param handle[in] Handle to the IP5561 device
 * @param callback[in] Wakeup callback function, pass NULL to disable auto wakeup
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_wakeup_callback(ip5561_handle_t handle, ip5561_wakeup_callback_t callback);

/**
 * @brief Create a new IP5561 handle
 *
 * @param config[in] Configuration structure containing I2C bus handle and IP5561 config
 *
 * @return ip5561_handle_t Handle to the IP5561 device, or NULL if creation failed
 */
ip5561_handle_t ip5561_create(const ip5561_config_t *config);

/**
 * @brief Delete a IP5561 handle
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_delete(ip5561_handle_t handle);

/* ========== System Control Functions ========== */

/**
 * @brief Enable or disable charging
 *
 * @param handle[in] Handle to the IP5561 device
 * @param enable[in] true to enable charging, false to disable
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_charge_enable(ip5561_handle_t handle, bool enable);

/**
 * @brief Get charging enable status
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return true if charging enabled, false otherwise
 */
bool ip5561_get_charge_enable(ip5561_handle_t handle);

/**
 * @brief Enable or disable boost mode
 *
 * @param handle[in] Handle to the IP5561 device
 * @param enable[in] true to enable boost mode, false to disable
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_boost_enable(ip5561_handle_t handle, bool enable);

/**
 * @brief Get system control register 0
 *
 * @param handle[in] Handle to the IP5561 device
 * @param sys_ctl0[out] Pointer to store system control value
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or sys_ctl0 is NULL
 */
esp_err_t ip5561_get_sys_ctl0(ip5561_handle_t handle, uint8_t *sys_ctl0);

/**
 * @brief Set system control register 0
 *
 * @param handle[in] Handle to the IP5561 device
 * @param sys_ctl0[in] System control value to set
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_sys_ctl0(ip5561_handle_t handle, uint8_t sys_ctl0);

/**
 * @brief Get system control register 1
 *
 * @param handle[in] Handle to the IP5561 device
 * @param sys_ctl1[out] Pointer to store system control 1 value
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or sys_ctl1 is NULL
 */
esp_err_t ip5561_get_sys_ctl1(ip5561_handle_t handle, uint8_t *sys_ctl1);

/**
 * @brief Set system control register 1
 *
 * @param handle[in] Handle to the IP5561 device
 * @param sys_ctl1[in] System control 1 value to set
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_sys_ctl1(ip5561_handle_t handle, uint8_t sys_ctl1);

/**
 * @brief Configure light load detection for boost mode
 *
 * This function controls whether IP5561 automatically disables boost output
 * when the load is light (e.g., external device is fully charged).
 *
 * @param handle[in] Handle to the IP5561 device
 * @param enable_current_detect true to enable current-based light load detection
 * @param enable_power_detect true to enable power-based light load detection
 * @param timeout_mode Light load timeout mode (0-3)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_configure_light_load(ip5561_handle_t handle, bool enable_current_detect,
                                       bool enable_power_detect, uint8_t timeout_mode);

/**
 * @brief Disable all fast charge protocols (QC/PD/FCPE)
 *
 * This function disables automatic fast charge protocol detection.
 * Use this when you only want 5V standard charging without protocol negotiation.
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_disable_fast_charge(ip5561_handle_t handle);

/**
 * @brief Print diagnostic register values for debugging
 *
 * This function reads and prints key control registers to help diagnose
 * charging issues.
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_print_diagnostics(ip5561_handle_t handle);

/**
 * @brief Disable boost output protections
 *
 * Disables various protection mechanisms that may cause boost output to shut down:
 * - Light load detection (already disabled by configure_light_load)
 * - Timeout protections
 * - VBUS undervoltage protection (boost mode)
 *
 * WARNING: Use with caution. Disabling protections may cause overheating or damage.
 *
 * @param handle[in] Handle to the IP5561 device
 * @param disable_ntc If true, also disable NTC temperature protection (DANGEROUS!)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_disable_boost_protections(ip5561_handle_t handle, bool disable_ntc);

/**
 * @brief Disable NTC temperature protection
 *
 * WARNING: ⚠️ DANGEROUS! Only for testing/debugging!
 * Disabling temperature protection can cause chip damage or fire hazard.
 * Monitor chip temperature carefully when NTC is disabled.
 *
 * @param handle[in] Handle to the IP5561 device
 * @param disable true to disable NTC, false to enable NTC
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_ntc_enable(ip5561_handle_t handle, bool enable);

/**
 * @brief Disable NTC charge temperature protection
 *
 * This function disables charging temperature protection by clearing
 * the charge high temperature (ChgHT, 45°C) and charge low temperature
 * (ChgLT, 0°C) protection bits in the NTC_ENABLE register (0xFD).
 *
 * WARNING: ⚠️ DANGEROUS! Only for testing/debugging!
 * Disabling temperature protection can cause battery damage or fire hazard.
 * Monitor battery and chip temperature carefully when temperature protection is disabled.
 *
 * The function preserves other NTC settings (bit 0, 4, 5, 6, 7) and only
 * clears bits 2 and 3:
 * - Bit 2: En_chg_ht - Charge high temp protection (45°C)
 * - Bit 3: En_chg_lt - Charge low temp protection (0°C)
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_disable_ntc_temperature_protection(ip5561_handle_t handle);

/* ========== Wireless Power Charging (WPC) Functions ========== */

/**
 * @brief Enable or disable wireless charger
 *
 * @param handle[in] Handle to the IP5561 device
 * @param enable[in] true to enable wireless charger, false to disable
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_wpc_enable(ip5561_handle_t handle, bool enable);

/**
 * @brief Get wireless charger enable status
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return true if wireless charger enabled, false otherwise
 */
bool ip5561_get_wpc_enable(ip5561_handle_t handle);

/**
 * @brief Get wireless charger status
 *
 * @param handle[in] Handle to the IP5561 device
 * @param status[out] Pointer to store wireless charger status structure
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or status is NULL
 */
esp_err_t ip5561_get_wpc_status(ip5561_handle_t handle, ip5561_wpc_status_t *status);

/**
 * @brief Check if wireless power is present
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return true if wireless power present, false otherwise
 */
bool ip5561_is_wpc_present(ip5561_handle_t handle);

/**
 * @brief Check if wireless charging is in progress
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return true if wireless charging, false otherwise
 */
bool ip5561_is_wpc_charging(ip5561_handle_t handle);

/**
 * @brief Get OCP (Over Current Protection) status
 *
 * This function reads the OCP_STATE register (0xFC) to check if any
 * over current protection has been triggered.
 *
 * @param handle[in] Handle to the IP5561 device
 * @param ocp_state[out] Pointer to store OCP state register value
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or ocp_state is NULL
 */
esp_err_t ip5561_get_ocp_state(ip5561_handle_t handle, uint8_t *ocp_state);

/* ========== Charging Control Functions ========== */

/**
 * @brief Set charge voltage in mV
 *
 * @param handle[in] Handle to the IP5561 device
 * @param voltage_mv[in] Charge voltage in mV (4200-4400mV)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL or voltage out of range
 */
esp_err_t ip5561_set_charge_voltage(ip5561_handle_t handle, uint16_t voltage_mv);

/**
 * @brief Set 9V input charging current limit
 *
 * This function sets the CHG_CTL5 register (0x6F) to control the maximum
 * charging current when using 9V fast charge input.
 *
 * @param handle[in] Handle to the IP5561 device
 * @param current_ma Charging current limit in mA (e.g., 3500 for 3.5A)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_9v_charge_current(ip5561_handle_t handle, uint16_t current_ma);

/**
 * @brief Set 9V input undervoltage loop threshold
 *
 * This function sets the CHG_CTL1 register (0x0C) to control the undervoltage
 * protection threshold when using 9V fast charge input.
 *
 * @param handle[in] Handle to the IP5561 device
 * @param uv_threshold_mv Undervoltage threshold in mV (e.g., 7500 for 7.5V)
 *                      When VBUS drops below this threshold, charging stops.
 *                      Valid range: typically 6000-9000 mV
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_9v_uv_threshold(ip5561_handle_t handle, uint16_t uv_threshold_mv);

/**
 * @brief Set VBUS 9V output current limit (boost mode)
 *
 * This function sets the VBUS_9V register (0xBB) to control the maximum
 * output current when boost mode is enabled and outputting 9V.
 *
 * @param handle[in] Handle to the IP5561 device
 * @param current_ma Output current limit in mA (e.g., 1000 for 1A)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_set_vbus_9v_current(ip5561_handle_t handle, uint16_t current_ma);

/* ========== Status Reading Functions ========== */

/**
 * @brief Get charging status
 *
 * @param handle[in] Handle to the IP5561 device
 * @param status[out] Pointer to store charging status structure
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or status is NULL
 */
esp_err_t ip5561_get_charge_status(ip5561_handle_t handle, ip5561_chg_status_t *status);

/**
 * @brief Check if charging is in progress
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return true if charging, false otherwise
 */
bool ip5561_is_charging(ip5561_handle_t handle);

/**
 * @brief Check if charging is completed
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return true if charge complete, false otherwise
 */
bool ip5561_is_charge_complete(ip5561_handle_t handle);

/**
 * @brief Check if VBUS power is present
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return true if VBUS present, false otherwise
 */
bool ip5561_is_vbus_present(ip5561_handle_t handle);

/**
 * @brief Check if battery is present
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return true if battery present, false otherwise
 */
bool ip5561_is_battery_present(ip5561_handle_t handle);

/* ========== ADC Reading Functions ========== */

/**
 * @brief Get battery voltage in mV
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return uint16_t Battery voltage in ADC units
 */
uint16_t ip5561_get_battery_voltage(ip5561_handle_t handle);

/**
 * @brief Get battery current in mA
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return int16_t Battery current in ADC units
 */
int16_t ip5561_get_battery_current(ip5561_handle_t handle);

/**
 * @brief Get battery percentage
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return uint8_t Battery percentage (0-100%)
 */
uint8_t ip5561_get_battery_percent(ip5561_handle_t handle);

/* ========== VBUS/VOUT Reading ========== */

/**
 * @brief Get VBUS voltage in ADC units
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return uint16_t VBUS voltage in ADC units
 */
uint16_t ip5561_get_vbus_voltage(ip5561_handle_t handle);

/**
 * @brief Get VBUS current in ADC units
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return int16_t VBUS current in ADC units (positive = charging/output)
 */
int16_t ip5561_get_ibus_current(ip5561_handle_t handle);

/**
 * @brief Get VOUT voltage in ADC units
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return uint16_t VOUT voltage in ADC units
 */
uint16_t ip5561_get_vout_voltage(ip5561_handle_t handle);

/**
 * @brief Configure NTC1 for temperature reading
 *
 * This function configures the NTC1 thermistor for manual temperature reading
 * according to IP5561 datasheet section 3.6. It should be called before reading
 * NTC temperature using ip5561_get_ntc_temperature().
 *
 * @param handle[in] Handle to the IP5561 device
 * @param current_source Current source selection:
 *                       0 = Internal state machine auto control
 *                       2 = 80uA
 *                       3 = 20uA
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL or current_source invalid
 */
esp_err_t ip5561_configure_ntc1(ip5561_handle_t handle, uint8_t current_source);

/**
 * @brief Get NTC1 thermistor voltage in mV
 *
 * This function reads the NTC1 thermistor ADC value and converts it to voltage.
 * The NTC is typically used for battery temperature monitoring.
 *
 * @param handle[in] Handle to the IP5561 device
 *
 * @return uint16_t NTC1 voltage in ADC units (raw value)
 */
uint16_t ip5561_get_ntc_voltage(ip5561_handle_t handle);

/**
 * @brief Get NTC1 temperature in degrees Celsius
 *
 * This function reads the NTC1 ADC value and converts it to temperature.
 * Uses NTC thermistor parameters (R25, Beta) for calculation.
 *
 * The function temporarily configures NTC1 with 80uA current source for measurement,
 * then restores the original NTC_ENABLE register value.
 *
 * Typical NTC configuration:
 * - Circuit: NTC1_PIN -- NTC thermistor -- GND
 * - NTC: 10kΩ @ 25°C, B = 3435K
 * - Current source: 80uA
 *
 * @param handle[in] Handle to the IP5561 device
 * @param temp_c[out] Pointer to store temperature in Celsius
 * @param adc_value[out] Optional pointer to store raw ADC value (can be NULL)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or temp_c is NULL
 */
esp_err_t ip5561_get_ntc_temperature(ip5561_handle_t handle, int16_t *temp_c, uint16_t *adc_value);

/* ========== Register Level Access ========== */

/**
 * @brief Read a byte from IP5561 register
 *
 * @param handle[in] Handle to the IP5561 device
 * @param reg[in] Register address
 * @param value[out] Pointer to store read value
 * @param is_stat[in] true to use I2C 0xEA (status/ADC), false to use 0xE8 (control)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or value is NULL
 */
esp_err_t ip5561_read_reg(ip5561_handle_t handle, uint8_t reg, uint8_t *value, bool is_stat);

/**
 * @brief Write a byte to IP5561 register
 *
 * @param handle[in] Handle to the IP5561 device
 * @param reg[in] Register address
 * @param value[in] Value to write
 * @param is_stat[in] true to use I2C 0xEA (status/ADC), false to use 0xE8 (control)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t ip5561_write_reg(ip5561_handle_t handle, uint8_t reg, uint8_t value, bool is_stat);

/**
 * @brief Read multiple bytes from IP5561 registers
 *
 * @param handle[in] Handle to the IP5561 device
 * @param reg[in] Starting register address
 * @param data[out] Buffer to store read data
 * @param len[in] Number of bytes to read
 * @param is_stat[in] true to use I2C 0xEA (status/ADC), false to use 0xE8 (control)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or data is NULL
 */
esp_err_t ip5561_read_regs(ip5561_handle_t handle, uint8_t reg, uint8_t *data, size_t len, bool is_stat);

/**
 * @brief Write multiple bytes to IP5561 registers
 *
 * @param handle[in] Handle to the IP5561 device
 * @param reg[in] Starting register address
 * @param data[in] Buffer containing data to write
 * @param len[in] Number of bytes to write
 * @param is_stat[in] true to use I2C 0xEA (status/ADC), false to use 0xE8 (control)
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or data is NULL
 */
esp_err_t ip5561_write_regs(ip5561_handle_t handle, uint8_t reg, const uint8_t *data, size_t len, bool is_stat);

#ifdef __cplusplus
}
#endif
