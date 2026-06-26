/*
 * CW2217E Fuel Gauge Driver
 *
 * Cellwise CW2217E/CW221X driver using native cw2217e_* function names.
 * Shared data types are sourced from fuel_gauge.h.
 *
 * I2C address: 0x64 (7-bit)
 * Expected chip ID: 0xA0
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "fuel_gauge.h"  /* Shared types: battery_status_t, operation_status_t, gauging_config_t, parameter_cedv_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ========== CW2217E native types ========== */

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    const gauging_config_t *cfg;
    const parameter_cedv_t *cedv;
} cw2217e_config_t;

typedef void *cw2217e_handle_t;

/* ========== CW2217E native API ========== */

cw2217e_handle_t cw2217e_create(const cw2217e_config_t *config);
esp_err_t cw2217e_delete(cw2217e_handle_t handle);
uint16_t cw2217e_get_fw_version(cw2217e_handle_t handle);
uint16_t cw2217e_get_hw_version(cw2217e_handle_t handle);
esp_err_t cw2217e_seal(cw2217e_handle_t handle);
esp_err_t cw2217e_unseal(cw2217e_handle_t handle);
esp_err_t cw2217e_set_parameter_u16(cw2217e_handle_t handle, uint16_t address, uint16_t value);
uint16_t cw2217e_get_parameter_u16(cw2217e_handle_t handle, uint16_t address);
uint16_t cw2217e_get_voltage(cw2217e_handle_t handle);
int16_t  cw2217e_get_current(cw2217e_handle_t handle);
int16_t  cw2217e_get_avgcurrent(cw2217e_handle_t handle);
uint16_t cw2217e_get_cycle_count(cw2217e_handle_t handle);
esp_err_t cw2217e_get_battery_status(cw2217e_handle_t handle, battery_status_t *battery_status);
esp_err_t cw2217e_get_operation_status(cw2217e_handle_t handle, operation_status_t *operation_status);
uint16_t cw2217e_get_temperature(cw2217e_handle_t handle);
uint16_t cw2217e_get_full_charge_capacity(cw2217e_handle_t handle);
uint16_t cw2217e_get_design_capacity(cw2217e_handle_t handle);
uint16_t cw2217e_get_remaining_capacity(cw2217e_handle_t handle);
uint16_t cw2217e_get_state_of_charge(cw2217e_handle_t handle);
uint16_t cw2217e_get_state_of_health(cw2217e_handle_t handle);
uint16_t cw2217e_get_charge_voltage(cw2217e_handle_t handle);
uint16_t cw2217e_get_charge_current(cw2217e_handle_t handle);
int16_t  cw2217e_get_average_power(cw2217e_handle_t handle);
uint16_t cw2217e_get_time_to_empty(cw2217e_handle_t handle);
uint16_t cw2217e_get_time_to_full(cw2217e_handle_t handle);
int16_t  cw2217e_get_maxload_current(cw2217e_handle_t handle);
int16_t  cw2217e_get_standby_current(cw2217e_handle_t handle);

/* Chip ID monitor for hardware testing */
void cw2217e_start_chip_id_monitor(cw2217e_handle_t handle);
void cw2217e_start_chip_id_monitor_from_bus(i2c_master_bus_handle_t i2c_bus);
void cw2217e_stop_chip_id_monitor(void);

/* ========== CW2217E Extended API (from reference code) ========== */

/**
 * @brief Get raw coulomb count (stub - CW2217E does not support this)
 * @return Always 0
 */
uint16_t cw2217e_get_raw_coulomb_count(cw2217e_handle_t handle);

/**
 * @brief Read chip ID from CW2217E
 * @return 0 on success, -1 on I2C error
 */
int cw2217e_get_chip_id(cw2217e_handle_t handle, uint8_t *chip_id);

/**
 * @brief Put CW2217E into sleep mode (low power)
 */
esp_err_t cw2217e_sleep(cw2217e_handle_t handle);

/**
 * @brief Wake up CW2217E from sleep mode
 */
esp_err_t cw2217e_wakeup(cw2217e_handle_t handle);

/**
 * @brief Read MODE_CONFIG register (0x08)
 * @return register value, or 0xFF on error
 *         0x00 = ACTIVE, 0xF0 = SLEEP, 0x30 = RESTART
 */
uint8_t cw2217e_get_mode_config(cw2217e_handle_t handle);

/**
 * @brief Write host temperature to CW2217E for compensation
 * @param temperature Temperature in Celsius (-40 ~ 87)
 */
esp_err_t cw2217e_write_temperature(cw2217e_handle_t handle, int temperature);

/**
 * @brief Get temperature in 0.1°C (CW2217E native format)
 * @return Temperature in 0.1°C, e.g. 250 = 25.0°C
 */
int cw2217e_get_temperature_celsius(cw2217e_handle_t handle);

/**
 * @brief Read IC_STATE register (bit2: VOL_CUR_READY, bit3: TEMP_READY)
 */
uint8_t cw2217e_get_ic_state(cw2217e_handle_t handle);

/**
 * @brief Dump all CW2217E registers (0x00-0xFF) to log
 */
void cw2217e_dump_register(cw2217e_handle_t handle);

#ifdef __cplusplus
}
#endif
