/*
 * Fuel Gauge Abstraction Layer
 *
 * Unified API for BQ27220 (TI) and CW2217E (Cellwise) battery fuel gauges.
 * Runtime auto-detection: probes I2C address 0x55 (BQ27220) then 0x64 (CW2217E).
 *
 * Shared data types live here and are consumed by the underlying drivers.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Shared data types (used by bq27220 and cw2217e drivers) ========== */

typedef union {
    struct {
        // Low byte, Low bit first
        bool DSG : 1;       /* The device is in DISCHARGE */
        bool SYSDWN : 1;    /* System down bit indicating the system should shut down */
        bool TDA : 1;       /* Terminate Discharge Alarm */
        bool BATTPRES : 1;  /* Battery Present detected */
        bool AUTH_GD : 1;   /* Detect inserted battery */
        bool OCVGD : 1;     /* Good OCV measurement taken */
        bool TCA : 1;       /* Terminate Charge Alarm */
        bool RSVD : 1;      /* Reserved */
        // High byte, Low bit first
        bool CHGINH : 1;    /* Charge inhibit */
        bool FC : 1;        /* Full-charged is detected */
        bool OTD : 1;       /* Overtemperature in discharge condition is detected */
        bool OTC : 1;       /* Overtemperature in charge condition is detected */
        bool SLEEP : 1;     /* Device is operating in SLEEP mode when set */
        bool OCVFAIL : 1;   /* Status bit indicating that the OCV reading failed due to current */
        bool OCVCOMP : 1;   /* An OCV measurement update is complete */
        bool FD : 1;        /* Full-discharge is detected */
    };
    uint16_t full;
} battery_status_t;

ESP_STATIC_ASSERT(sizeof(battery_status_t) == 2, "Incorrect structure size");

typedef struct {
    // Low byte, Low bit first
    bool CALMD : 1;         /* Calibration mode enabled */
    uint8_t SEC : 2;        /* Current security access */
    bool EDV2 : 1;          /* EDV2 threshold exceeded */
    bool VDQ : 1;           /* Indicates if Current discharge cycle is NOT qualified or qualified for an FCC updated */
    bool INITCOMP : 1;      /* gauge initialization is complete */
    bool SMTH : 1;          /* RemainingCapacity is scaled by smooth engine */
    bool BTPINT : 1;        /* BTP threshold has been crossed */
    // High byte, Low bit first
    uint8_t RSVD1 : 2;
    bool CFGUPDATE : 1;     /* Gauge is in CONFIG UPDATE mode */
    uint8_t RSVD0 : 5;
} operation_status_t;

ESP_STATIC_ASSERT(sizeof(operation_status_t) == 2, "Incorrect structure size");

typedef union {
    struct {
        // Low byte, Low bit first
        bool CCT : 1;       /* 0 = Use CC % of DesignCapacity() (default), 1 = Use CC % of FullChargeCapacity() */
        bool CSYNC : 1;     /* Sync RemainingCapacity() with FullChargeCapacity() at valid charge termination */
        bool RSVD0 : 1;
        bool EDV_CMP : 1;
        bool SC : 1;
        bool FIXED_EDV0 : 1;
        uint8_t RSVD1 : 2;
        // High byte, Low bit first
        bool FCC_LIM : 1;
        bool RSVD2 : 1;
        bool FC_FOR_VDQ : 1; /* full charge voltage for VDQ */
        bool IGNORE_SD : 1; /* ignore self-discharge */
        bool SME0 : 1;
        uint8_t RSVD3 : 3;
    };
    uint16_t full;
} gauging_config_t;
ESP_STATIC_ASSERT(sizeof(gauging_config_t) == 2, "Incorrect structure size");

/* CEDV (Compensated End-of-Discharge-Voltage) parameter set
 * These values are loaded into RAM to configure the BQ27220 gas-gauge for a specific cell chemistry.
 */
typedef struct {
    uint16_t full_charge_cap;   /* Learned Full-Charge Capacity (mAh) – actual usable capacity */
    uint16_t design_cap;        /* Design Capacity (mAh) */
    uint16_t reserve_cap;       /* Reserve Capacity (mAh) */
    uint16_t near_full;         /* Near Full threshold (mAh) */
    uint16_t self_discharge_rate; /* Self-discharge rate index (0-255) → %/day = value × 0.0025 % */
    uint16_t EDV0;              /* End-of-Discharge Voltage 0 % SOC (mV) */
    uint16_t EDV1;              /* End-of-Discharge Voltage 3 % SOC (mV) */
    uint16_t EDV2;              /* End-of-Discharge Voltage Battery-Low % SOC (mV) */
    uint16_t EMF;               /* This value is the no-load cell voltage higher than the highest cell EDV threshold computed */
    uint16_t C0;                /* This value is the no-load, capacity related EDV adjustment factor */
    uint16_t R0;                /* This value is the first order rate dependency factor, accounting for battery impedance adjustment. */
    uint16_t T0;                /* This value adjusts the variation of impedance with battery temperature. */
    uint16_t R1;                /* This value adjusts the variation of impedance with battery capacity. */
    uint8_t  TC;                /* This value adjusts the variation of impedance for cold temperatures (T < 23°C). */
    uint8_t  C1;                /* This value is the desired reserved battery capacity remaining at EDV0. */

    /* Voltage vs Depth-of-Discharge (DOD) table (mV) */
    uint16_t DOD0;              /* 0 % DOD (full charge) */
    uint16_t DOD10;             /* 10 % DOD */
    uint16_t DOD20;             /* 20 % DOD */
    uint16_t DOD30;             /* 30 % DOD */
    uint16_t DOD40;             /* 40 % DOD */
    uint16_t DOD50;             /* 50 % DOD */
    uint16_t DOD60;             /* 60 % DOD */
    uint16_t DOD70;             /* 70 % DOD */
    uint16_t DOD80;             /* 80 % DOD */
    uint16_t DOD90;             /* 90 % DOD */
    uint16_t DOD100;            /* 100 % DOD (empty) */
} parameter_cedv_t;

/* ========== Fuel Gauge Unified API ========== */

/* Detected fuel gauge chip type */
typedef enum {
    FUEL_GAUGE_UNKNOWN = 0,
    FUEL_GAUGE_BQ27220,
    FUEL_GAUGE_CW2217E,
} fuel_gauge_type_t;

typedef void *fuel_gauge_handle_t;

/* Configuration
 * - gauging_config and cedv_params are used by BQ27220 only; CW2217E ignores them.
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    const gauging_config_t *gauging_config;  /* BQ27220 only */
    const parameter_cedv_t *cedv_params;     /* BQ27220 only */
} fuel_gauge_config_t;

/**
 * @brief Create a fuel gauge handle with runtime auto-detection
 *
 * Probes I2C address 0x55 (BQ27220) first, then 0x64 (CW2217E).
 *
 * @param config[in] Configuration with I2C bus and optional BQ27220 parameters
 * @return fuel_gauge_handle_t Handle, or NULL if no fuel gauge detected
 */
fuel_gauge_handle_t fuel_gauge_create(const fuel_gauge_config_t *config);

/**
 * @brief Delete a fuel gauge handle
 */
esp_err_t fuel_gauge_delete(fuel_gauge_handle_t handle);

/**
 * @brief Get the detected chip type
 */
fuel_gauge_type_t fuel_gauge_get_type(fuel_gauge_handle_t handle);

/* ========== Measurement API (units identical across both chips) ========== */

uint16_t fuel_gauge_get_voltage(fuel_gauge_handle_t handle);             /* mV */
int16_t  fuel_gauge_get_current(fuel_gauge_handle_t handle);             /* mA (+ = discharge, - = charge) */
int16_t  fuel_gauge_get_avgcurrent(fuel_gauge_handle_t handle);          /* mA */
uint16_t fuel_gauge_get_temperature(fuel_gauge_handle_t handle);         /* 0.1°K */
uint16_t fuel_gauge_get_state_of_charge(fuel_gauge_handle_t handle);     /* % */
uint16_t fuel_gauge_get_state_of_health(fuel_gauge_handle_t handle);     /* % */
uint16_t fuel_gauge_get_full_charge_capacity(fuel_gauge_handle_t handle);/* mAh */
uint16_t fuel_gauge_get_design_capacity(fuel_gauge_handle_t handle);     /* mAh */
uint16_t fuel_gauge_get_remaining_capacity(fuel_gauge_handle_t handle);  /* mAh */
uint16_t fuel_gauge_get_cycle_count(fuel_gauge_handle_t handle);
uint16_t fuel_gauge_get_charge_voltage(fuel_gauge_handle_t handle);      /* mV */
uint16_t fuel_gauge_get_charge_current(fuel_gauge_handle_t handle);      /* mA */
int16_t  fuel_gauge_get_average_power(fuel_gauge_handle_t handle);       /* mW */
uint16_t fuel_gauge_get_time_to_empty(fuel_gauge_handle_t handle);       /* min */
uint16_t fuel_gauge_get_time_to_full(fuel_gauge_handle_t handle);        /* min */
int16_t  fuel_gauge_get_maxload_current(fuel_gauge_handle_t handle);     /* mA */
int16_t  fuel_gauge_get_standby_current(fuel_gauge_handle_t handle);     /* mA */
uint16_t fuel_gauge_get_raw_coulomb_count(fuel_gauge_handle_t handle);   /* BQ27220 only, 0 on CW2217E */

esp_err_t fuel_gauge_get_battery_status(fuel_gauge_handle_t handle, battery_status_t *status);
esp_err_t fuel_gauge_get_operation_status(fuel_gauge_handle_t handle, operation_status_t *status);

#ifdef __cplusplus
}
#endif
