/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "fuel_gauge.h"  /* Shared types: battery_status_t, operation_status_t, gauging_config_t, parameter_cedv_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t i2c_bus;  // I2C bus handle
    const gauging_config_t *cfg;  // Pointer to the gauging_config_t structure
    const parameter_cedv_t *cedv;  // Pointer to the CEDV structure
} bq27220_config_t;

typedef void *bq27220_handle_t;

/**
 * @brief Create a new BQ27220 handle
 *
 * @param config[in] Configuration structure containing I2C bus handle and gauge parameters
 *
 * @return bq27220_handle_t Handle to the BQ27220 device, or NULL if creation failed
 */
bq27220_handle_t bq27220_create(const bq27220_config_t *config);

/**
 * @brief Delete a BQ27220 handle
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if bq_handle is NULL
 */
esp_err_t bq27220_delete(bq27220_handle_t bq_handle);

/**
 * @brief Get the firmware version
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Firmware version
 */
uint16_t bq27220_get_fw_version(bq27220_handle_t bq_handle);

/**
 * @brief Get the hardware version
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Hardware version
 */
uint16_t bq27220_get_hw_version(bq27220_handle_t bq_handle);

/**
 * @brief Seal the BQ27220 device to prevent unauthorized access to configuration data
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if bq_handle is NULL
 *     - ESP_FAIL if operation failed
 */
esp_err_t bq27220_seal(bq27220_handle_t bq_handle);

/**
 * @brief Unseal the BQ27220 device to allow access to configuration data
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if bq_handle is NULL
 *     - ESP_FAIL if operation failed
 */
esp_err_t bq27220_unseal(bq27220_handle_t bq_handle);

/**
 * @brief Set a 16-bit parameter in the BQ27220 device
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 * @param address[in] Parameter address
 * @param value[in] Value to set
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if bq_handle is NULL
 *     - ESP_FAIL if operation failed
 */
esp_err_t bq27220_set_parameter_u16(bq27220_handle_t bq_handle, uint16_t address, uint16_t value);

/**
 * @brief Get a 16-bit parameter from the BQ27220 device
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 * @param address[in] Parameter address
 *
 * @return uint16_t Parameter value
 */
uint16_t bq27220_get_parameter_u16(bq27220_handle_t bq_handle, uint16_t address);

/**
 * @brief Get the battery voltage in millivolts (mV)
 *
 * @param handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Battery voltage in mV
 */
uint16_t bq27220_get_voltage(bq27220_handle_t handle);

/**
 * @brief Get the instantaneous current in milliamperes (mA)
 *
 * @param handle[in] Handle to the BQ27220 device
 *
 * @return int16_t Battery current in mA (positive for charging, negative for discharging)
 */
int16_t bq27220_get_current(bq27220_handle_t handle);

/**
 * @brief Get the average current in milliamperes (mA)
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return int16_t Average battery current in mA (positive for charging, negative for discharging)
 */
int16_t bq27220_get_avgcurrent(bq27220_handle_t bq_handle);

/**
 * @brief Get the battery cycle count
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Number of charge/discharge cycles
 */
uint16_t bq27220_get_cycle_count(bq27220_handle_t bq_handle);

/**
 * @brief Get the battery status
 *
 * @param handle[in] Handle to the BQ27220 device
 * @param battery_status[out] Pointer to store the battery status
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or battery_status is NULL
 */
esp_err_t bq27220_get_battery_status(bq27220_handle_t handle, battery_status_t *battery_status);

/**
 * @brief Get the operation status
 *
 * @param handle[in] Handle to the BQ27220 device
 * @param operation_status[out] Pointer to store the operation status
 *
 * @return
 *     - ESP_OK if successful
 *     - ESP_ERR_INVALID_ARG if handle or operation_status is NULL
 */
esp_err_t bq27220_get_operation_status(bq27220_handle_t handle, operation_status_t *operation_status);

/**
 * @brief Get the temperature in units of 0.1°K
 *
 * @param handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Temperature in 0.1°K
 */
uint16_t bq27220_get_temperature(bq27220_handle_t handle);

/**
 * @brief Get the compensated full charge capacity in mAh
 *
 * @param handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Full charge capacity in mAh
 */
uint16_t bq27220_get_full_charge_capacity(bq27220_handle_t handle);

/**
 * @brief Get the design capacity in mAh
 *
 * @param handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Design capacity in mAh
 */
uint16_t bq27220_get_design_capacity(bq27220_handle_t handle);

/**
 * @brief Get the remaining capacity in mAh
 *
 * @param handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Remaining capacity in mAh
 */
uint16_t bq27220_get_remaining_capacity(bq27220_handle_t handle);

/**
 * @brief Get the predicted remaining battery capacity in percents
 *
 * @param handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t State of charge in percent (0-100%)
 */
uint16_t bq27220_get_state_of_charge(bq27220_handle_t handle);

/**
 * @brief Get the ratio of full charge capacity over design capacity in percents
 *
 * @param handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t State of health in percent (0-100%)
 */
uint16_t bq27220_get_state_of_health(bq27220_handle_t handle);

/**
 * @brief Get the charge voltage in millivolts (mV)
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Charge voltage in mV
 */
uint16_t bq27220_get_charge_voltage(bq27220_handle_t bq_handle);

/**
 * @brief Get the charge current in milliamperes (mA)
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Charge current in mA
 */
uint16_t bq27220_get_charge_current(bq27220_handle_t bq_handle);

/**
 * @brief Get the average power in milliwatts (mW)
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return int16_t Average power in mW (positive for charging, negative for discharging)
 */
int16_t bq27220_get_average_power(bq27220_handle_t bq_handle);

/**
 * @brief Get the predicted time to empty in minutes
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Time to empty in minutes
 */
uint16_t bq27220_get_time_to_empty(bq27220_handle_t bq_handle);

/**
 * @brief Get the predicted time to full in minutes
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Time to full in minutes
 */
uint16_t bq27220_get_time_to_full(bq27220_handle_t bq_handle);

/**
 * @brief Get the maximum load current in milliamperes (mA)
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return int16_t Maximum load current in mA
 */
int16_t bq27220_get_maxload_current(bq27220_handle_t bq_handle);

/**
 * @brief Get the standby current in milliamperes (mA)
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return int16_t Standby current in mA
 */
int16_t bq27220_get_standby_current(bq27220_handle_t bq_handle);

/**
 * @brief Get the raw coulomb count
 *
 * Returns an unsigned integer representing coulombs transferred.
 * Counter increments during discharge, decrements during charge.
 * Resets to zero when FC (Full Charge) bit is set.
 *
 * @param bq_handle[in] Handle to the BQ27220 device
 *
 * @return uint16_t Raw coulomb count
 */
uint16_t bq27220_get_raw_coulomb_count(bq27220_handle_t bq_handle);

#ifdef __cplusplus
}
#endif
