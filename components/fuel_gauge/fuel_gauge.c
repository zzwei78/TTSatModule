/*
 * Fuel Gauge Abstraction Layer Implementation
 *
 * Runtime auto-detection: probes BQ27220 (0x55) then CW2217E (0x64),
 * wraps the detected chip and dispatches all calls via if/else.
 */
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "fuel_gauge.h"
#include "bq27220.h"
#include "cw2217e.h"

static const char *TAG = "FUEL_GAUGE";

/* I2C addresses */
#define BQ27220_I2C_ADDR    0x55
#define CW2217E_I2C_ADDR    0x64
#define PROBE_I2C_FREQ_HZ   100000

/* Internal device wrapper */
typedef struct {
    fuel_gauge_type_t type;
    void *dev;  /* bq27220_handle_t or cw2217e_handle_t */
} fuel_gauge_device_t;

/* ========== I2C Probe ========== */

/* Scan I2C bus for debugging: prints all responding addresses */
static void scan_i2c_bus(i2c_master_bus_handle_t bus)
{
    ESP_LOGW(TAG, "Scanning I2C bus for all devices...");
    for (uint8_t addr = 1; addr < 128; addr++) {
        i2c_master_dev_handle_t dev = NULL;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 200000,
        };
        if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
            continue;
        }
        uint8_t dummy = 0;
        esp_err_t ret = i2c_master_transmit_receive(dev, &dummy, 1, &dummy, 1, 100);
        i2c_master_bus_rm_device(dev);
        if (ret == ESP_OK) {
            ESP_LOGW(TAG, "  Found device at 0x%02X", addr);
        }
    }
}

/* Probe I2C address using transmit_receive (same method as boardtest reference) */
static bool probe_i2c(i2c_master_bus_handle_t bus, uint16_t addr)
{
    i2c_master_dev_handle_t dev = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 200000,
    };

    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
        return false;
    }

    uint8_t dummy = 0;
    esp_err_t ret = i2c_master_transmit_receive(dev, &dummy, 1, &dummy, 1, 100);

    i2c_master_bus_rm_device(dev);
    return (ret == ESP_OK);
}

/* ========== Lifecycle ========== */

fuel_gauge_handle_t fuel_gauge_create(const fuel_gauge_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, NULL, TAG, "Invalid config");
    ESP_RETURN_ON_FALSE(config->i2c_bus != NULL, NULL, TAG, "Invalid i2c_bus");

    fuel_gauge_device_t *fg = calloc(1, sizeof(fuel_gauge_device_t));
    if (!fg) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return NULL;
    }

    /* Try BQ27220 first (0x55) */
    if (probe_i2c(config->i2c_bus, BQ27220_I2C_ADDR)) {
        bq27220_config_t bq_cfg = {
            .i2c_bus = config->i2c_bus,
            .cfg = config->gauging_config,
            .cedv = config->cedv_params,
        };
        void *bq = bq27220_create(&bq_cfg);
        if (bq) {
            fg->type = FUEL_GAUGE_BQ27220;
            fg->dev = bq;
            ESP_LOGI(TAG, "Detected BQ27220 at 0x%02X", BQ27220_I2C_ADDR);
            return fg;
        }
        ESP_LOGW(TAG, "BQ27220 probe OK at 0x%02X but create failed", BQ27220_I2C_ADDR);
    }

    /* Try CW2217E (0x64) */
    if (probe_i2c(config->i2c_bus, CW2217E_I2C_ADDR)) {
        cw2217e_config_t cw_cfg = {
            .i2c_bus = config->i2c_bus,
            .cfg = config->gauging_config,
            .cedv = config->cedv_params,
        };
        void *cw = cw2217e_create(&cw_cfg);
        if (cw) {
            fg->type = FUEL_GAUGE_CW2217E;
            fg->dev = cw;
            ESP_LOGI(TAG, "Detected CW2217E at 0x%02X", CW2217E_I2C_ADDR);
            return fg;
        }
        ESP_LOGW(TAG, "CW2217E probe OK at 0x%02X but create failed", CW2217E_I2C_ADDR);
    }

    ESP_LOGE(TAG, "No fuel gauge detected (probed 0x%02X and 0x%02X)",
             BQ27220_I2C_ADDR, CW2217E_I2C_ADDR);
    scan_i2c_bus(config->i2c_bus);
    free(fg);
    return NULL;
}

esp_err_t fuel_gauge_delete(fuel_gauge_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;

    if (fg->type == FUEL_GAUGE_BQ27220) {
        bq27220_delete(fg->dev);
    } else if (fg->type == FUEL_GAUGE_CW2217E) {
        cw2217e_delete(fg->dev);
    }

    free(fg);
    return ESP_OK;
}

fuel_gauge_type_t fuel_gauge_get_type(fuel_gauge_handle_t handle)
{
    if (!handle) return FUEL_GAUGE_UNKNOWN;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    return fg->type;
}

/* ========== Dispatch functions ========== */

uint16_t fuel_gauge_get_voltage(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_voltage(fg->dev);

    /* CW2217E: try read, if fails wakeup and retry (chip may have slept) */
    uint16_t v = cw2217e_get_voltage(fg->dev);
    if (v == 0) {
        uint8_t mode_before = cw2217e_get_mode_config(fg->dev);
        cw2217e_wakeup(fg->dev);
        vTaskDelay(pdMS_TO_TICKS(10));
        v = cw2217e_get_voltage(fg->dev);
        uint8_t mode_after = cw2217e_get_mode_config(fg->dev);
        if (v > 0) {
            ESP_LOGW(TAG, "CW2217E recovered: MODE_CONFIG 0x%02X->0x%02X, V=%umV",
                     mode_before, mode_after, v);
        } else {
            ESP_LOGE(TAG, "CW2217E read failed: MODE_CONFIG before=0x%02X after=0x%02X",
                     mode_before, mode_after);
        }
    }
    return v;
}

int16_t fuel_gauge_get_current(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_current(fg->dev);
    /* CW2217E: negate to unify with BQ27220 convention
     * (negative = charging, positive = discharging) */
    return -cw2217e_get_current(fg->dev);
}

int16_t fuel_gauge_get_avgcurrent(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_avgcurrent(fg->dev);
    return -cw2217e_get_avgcurrent(fg->dev);
}

uint16_t fuel_gauge_get_temperature(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_temperature(fg->dev);
    return cw2217e_get_temperature(fg->dev);
}

uint16_t fuel_gauge_get_state_of_charge(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_state_of_charge(fg->dev);
    return cw2217e_get_state_of_charge(fg->dev);
}

uint16_t fuel_gauge_get_state_of_health(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_state_of_health(fg->dev);
    return cw2217e_get_state_of_health(fg->dev);
}

uint16_t fuel_gauge_get_full_charge_capacity(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_full_charge_capacity(fg->dev);
    return cw2217e_get_full_charge_capacity(fg->dev);
}

uint16_t fuel_gauge_get_design_capacity(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_design_capacity(fg->dev);
    return cw2217e_get_design_capacity(fg->dev);
}

uint16_t fuel_gauge_get_remaining_capacity(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_remaining_capacity(fg->dev);
    return cw2217e_get_remaining_capacity(fg->dev);
}

uint16_t fuel_gauge_get_cycle_count(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_cycle_count(fg->dev);
    return cw2217e_get_cycle_count(fg->dev);
}

uint16_t fuel_gauge_get_charge_voltage(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_charge_voltage(fg->dev);
    return cw2217e_get_charge_voltage(fg->dev);
}

uint16_t fuel_gauge_get_charge_current(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_charge_current(fg->dev);
    return cw2217e_get_charge_current(fg->dev);
}

int16_t fuel_gauge_get_average_power(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_average_power(fg->dev);
    return cw2217e_get_average_power(fg->dev);
}

uint16_t fuel_gauge_get_time_to_empty(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_time_to_empty(fg->dev);
    return cw2217e_get_time_to_empty(fg->dev);
}

uint16_t fuel_gauge_get_time_to_full(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_time_to_full(fg->dev);
    return cw2217e_get_time_to_full(fg->dev);
}

int16_t fuel_gauge_get_maxload_current(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_maxload_current(fg->dev);
    /* CW2217E: return standby current as the closest available proxy */
    return cw2217e_get_standby_current(fg->dev);
}

int16_t fuel_gauge_get_standby_current(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_standby_current(fg->dev);
    return cw2217e_get_standby_current(fg->dev);
}

uint16_t fuel_gauge_get_raw_coulomb_count(fuel_gauge_handle_t handle)
{
    if (!handle) return 0;
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_raw_coulomb_count(fg->dev);
    /* CW2217E has no raw coulomb count */
    return cw2217e_get_raw_coulomb_count(fg->dev);
}

esp_err_t fuel_gauge_get_battery_status(fuel_gauge_handle_t handle, battery_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(status, ESP_ERR_INVALID_ARG, TAG, "Invalid status ptr");
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_battery_status(fg->dev, status);
    return cw2217e_get_battery_status(fg->dev, status);
}

esp_err_t fuel_gauge_get_operation_status(fuel_gauge_handle_t handle, operation_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(status, ESP_ERR_INVALID_ARG, TAG, "Invalid status ptr");
    fuel_gauge_device_t *fg = (fuel_gauge_device_t *)handle;
    if (fg->type == FUEL_GAUGE_BQ27220) return bq27220_get_operation_status(fg->dev, status);
    return cw2217e_get_operation_status(fg->dev, status);
}
