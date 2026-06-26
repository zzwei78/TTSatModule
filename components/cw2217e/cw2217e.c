/*
 * CW2217E Fuel Gauge Driver
 *
 * Cellwise CW2217E/CW221X driver using native cw2217e_* function names.
 * The header cw2217e.h provides #define aliases to bq27220_* for compatibility.
 *
 * I2C address: 0x64 (7-bit, from reference: 0xC8 >> 1)
 * Expected chip ID: 0xA0
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "cw2217e.h"

static const char *TAG = "CW2217E";

/* ========== CW2217E Register Definitions ========== */
#define CW_REG_CHIP_ID         0x00
#define CW_REG_VCELL_H         0x02
#define CW_REG_VCELL_L         0x03
#define CW_REG_SOC_INT         0x04
#define CW_REG_SOC_DECIMAL     0x05
#define CW_REG_TEMP            0x06
#define CW_REG_MODE_CONFIG     0x08
#define CW_REG_GPIO_CONFIG     0x0A
#define CW_REG_SOC_ALERT       0x0B
#define CW_REG_TEMP_MAX        0x0C
#define CW_REG_TEMP_MIN        0x0D
#define CW_REG_CURRENT_H       0x0E
#define CW_REG_CURRENT_L       0x0F
#define CW_REG_T_HOST_H        0xA0
#define CW_REG_T_HOST_L        0xA1
#define CW_REG_USER_CONF       0xA2
#define CW_REG_CYCLE_H         0xA4
#define CW_REG_CYCLE_L         0xA5
#define CW_REG_SOH             0xA6
#define CW_REG_IC_STATE        0xA7
#define CW_REG_STB_CUR_H       0xA8
#define CW_REG_STB_CUR_L       0xA9
#define CW_REG_FW_VERSION      0xAB
#define CW_REG_BAT_PROFILE     0x10

/* CW2217E Configuration Constants */
#define CW_CONFIG_MODE_RESTART  0x30
#define CW_CONFIG_MODE_ACTIVE   0x00
#define CW_CONFIG_MODE_SLEEP    0xF0
#define CW_CONFIG_UPDATE_FLG    0x80
#define CW_IC_VCHIP_ID          0xA0
#define CW_IC_READY_MARK        0x0C
#define CW_IC_TEMP_READY        0x08
#define CW_IC_VOL_CUR_READY     0x04
#define CW_SIZE_OF_PROFILE      80
#define CW_SLEEP_COUNTS         50

/* CW221X FW version chip type marks */
#define CW2215_MARK             0x80
#define CW2217_MARK             0x40
#define CW2218_MARK             0x00
#define CW_GPIO_SOC_IRQ_VALUE   0x00

/* I2C address: 0xC8 write address, 7-bit = 0x64 */
#define CW2217E_I2C_ADDRESS     0x64
#define CW2217E_I2C_FREQ_HZ     400000

/* Sense resistor: 10 mOhm, scaled by 1000 */
#define USER_RSENSE              (10 * 1000)

#define CW_DESIGN_CAPACITY_MAH   4700   /* 华昊锂能 4700mAh */

#define delay_ms(x) vTaskDelay(pdMS_TO_TICKS(x))

/* ========== Battery Profile ==========
 * Battery: 华昊锂能 4700mAh
 * Profile: profile3_KWEI10X, R_sense=10mΩ
 * Source: Cellwise lab test 2026-06-24
 * File: 科纬智能_TTPBank_华昊锂能4700mAh_profile3_KWEI10X_10mohm_20260624.txt */
static const uint8_t cw_battery_profile[CW_SIZE_OF_PROFILE] = {
    0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xB6, 0xBF, 0xBD, 0xC8, 0xA5, 0xA2, 0xD3, 0xB4,
    0x9F, 0xF2, 0xD5, 0xB1, 0x97, 0x80, 0x62, 0x52,
    0x49, 0x45, 0x40, 0x84, 0x54, 0xDC, 0x84, 0xFF,
    0xFF, 0xFF, 0xCF, 0xB9, 0xC2, 0xD8, 0xD9, 0xD2,
    0xD1, 0xD9, 0xE1, 0xD0, 0xB4, 0x8D, 0x8B, 0x88,
    0x8B, 0x98, 0xAF, 0xC5, 0xCD, 0xBB, 0xF7, 0x81,
    0x20, 0x00, 0xAB, 0x10, 0x00, 0xB0, 0x82, 0x00,
    0x00, 0x00, 0x64, 0x08, 0xD3, 0x90, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
};

/* ========== Internal Data Structure ========== */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} cw2217e_data_t;

/* Chip ID monitor task handle */
static TaskHandle_t s_chip_id_task_handle = NULL;
static volatile bool s_chip_id_task_running = false;

/* ========== I2C Helper Functions ========== */

static esp_err_t cw_i2c_read_byte(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, 1, 200);
}

static esp_err_t cw_i2c_read_nbyte(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, 200);
}

static esp_err_t cw_i2c_write_byte(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    esp_err_t ret = i2c_master_transmit(dev, buf, 2, 200);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C write failed: reg=0x%02X val=0x%02X err=%s",
                 reg, data, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * Read a 16-bit word from two consecutive registers with verify-then-retry pattern.
 * Required by CW221X: read twice, if different, read a third time.
 */
static int cw_read_word(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2] = {0};
    uint16_t first, second;
    esp_err_t ret;

    ret = cw_i2c_read_nbyte(dev, reg, buf, 2);
    if (ret != ESP_OK) return -1;
    first = ((uint16_t)buf[0] << 8) | buf[1];

    delay_ms(4);  /* Must be >= 4ms between reads */

    ret = cw_i2c_read_nbyte(dev, reg, buf, 2);
    if (ret != ESP_OK) return -1;
    second = ((uint16_t)buf[0] << 8) | buf[1];

    if (first != second) {
        delay_ms(4);
        ret = cw_i2c_read_nbyte(dev, reg, buf, 2);
        if (ret != ESP_OK) return -1;
        first = ((uint16_t)buf[0] << 8) | buf[1];
    }

    *value = first;
    return 0;
}

/* ========== CW2217E Internal Functions ========== */

static int cw2217e_read_chip_id(cw2217e_data_t *data, uint8_t *chip_id)
{
    esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_CHIP_ID, chip_id);
    return (ret == ESP_OK) ? 0 : -1;
}

static int cw2217e_read_ic_state(cw2217e_data_t *data, uint8_t *state)
{
    esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_IC_STATE, state);
    return (ret == ESP_OK) ? 0 : -1;
}

static int cw2217e_enter_sleep(cw2217e_data_t *data)
{
    /* Read current mode before writing */
    uint8_t cur_mode = 0;
    cw_i2c_read_byte(data->i2c_dev, CW_REG_MODE_CONFIG, &cur_mode);
    ESP_LOGI(TAG, "enter_sleep: current MODE_CONFIG=0x%02X", cur_mode);

    /* If already in SLEEP mode, no need to re-enter */
    if (cur_mode == CW_CONFIG_MODE_SLEEP) {
        ESP_LOGI(TAG, "Already in SLEEP mode, skipping");
        return 0;
    }

    /* Per Cellwise reference: always RESTART → SLEEP for clean state transition.
     * This resets the chip's internal state machine before entering sleep,
     * ensuring a clean environment for profile writing. */
    if (cw_i2c_write_byte(data->i2c_dev, CW_REG_MODE_CONFIG, CW_CONFIG_MODE_RESTART) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write RESTART command");
        return -1;
    }
    delay_ms(20);  /* Reference: >= 20ms */

    if (cw_i2c_write_byte(data->i2c_dev, CW_REG_MODE_CONFIG, CW_CONFIG_MODE_SLEEP) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write SLEEP mode");
        return -1;
    }
    delay_ms(10);  /* Reference: >= 10ms */

    return 0;
}

static int cw2217e_enter_active(cw2217e_data_t *data)
{
    /* Read current mode */
    uint8_t cur_mode = 0;
    cw_i2c_read_byte(data->i2c_dev, CW_REG_MODE_CONFIG, &cur_mode);

    /* If already active, skip */
    if (cur_mode == CW_CONFIG_MODE_ACTIVE) {
        ESP_LOGI(TAG, "Already in ACTIVE mode");
        return 0;
    }

    ESP_LOGI(TAG, "Entering ACTIVE mode (current=0x%02X)", cur_mode);
    if (cw_i2c_write_byte(data->i2c_dev, CW_REG_MODE_CONFIG, CW_CONFIG_MODE_RESTART) != ESP_OK)
        return -1;
    delay_ms(50);  /* Increased from 20ms */

    if (cw_i2c_write_byte(data->i2c_dev, CW_REG_MODE_CONFIG, CW_CONFIG_MODE_ACTIVE) != ESP_OK)
        return -1;
    delay_ms(10);

    return 0;
}

static int cw2217e_write_profile(cw2217e_data_t *data)
{
    for (int i = 0; i < CW_SIZE_OF_PROFILE; i++) {
        if (cw_i2c_write_byte(data->i2c_dev, CW_REG_BAT_PROFILE + i, cw_battery_profile[i]) != ESP_OK)
            return -1;
        delay_ms(1);  /* CW2217E needs settling time between writes in SLEEP mode */
    }
    return 0;
}

static int cw2217e_check_state(cw2217e_data_t *data)
{
    uint8_t reg_val;
    esp_err_t ret;

    /* Check if active */
    ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_MODE_CONFIG, &reg_val);
    if (ret != ESP_OK) return -1;
    ESP_LOGI(TAG, "check_state: MODE_CONFIG=0x%02X (ACTIVE=0x%02X SLEEP=0x%02X)",
             reg_val, CW_CONFIG_MODE_ACTIVE, CW_CONFIG_MODE_SLEEP);
    if (reg_val != CW_CONFIG_MODE_ACTIVE) return 1;  /* NOT_ACTIVE */

    /* Check update flag */
    ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_SOC_ALERT, &reg_val);
    if (ret != ESP_OK) return -1;
    if (!(reg_val & CW_CONFIG_UPDATE_FLG)) return 2;  /* PROFILE_NOT_READY */

    /* Check if profile matches */
    for (int i = 0; i < CW_SIZE_OF_PROFILE; i++) {
        uint8_t profile_byte;
        ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_BAT_PROFILE + i, &profile_byte);
        if (ret != ESP_OK) return -1;
        if (cw_battery_profile[i] != profile_byte)
            return 3;  /* PROFILE_NEED_UPDATE */
    }

    return 0;  /* All good */
}

static int cw2217e_config_start_ic(cw2217e_data_t *data)
{
    /* Sleep -> write profile -> set flags -> active -> wait ready */
    if (cw2217e_enter_sleep(data) < 0) {
        ESP_LOGE(TAG, "Failed to enter sleep mode");
        return -1;
    }

    if (cw2217e_write_profile(data) < 0) {
        ESP_LOGE(TAG, "Failed to write battery profile");
        return -1;
    }
    delay_ms(50);  /* Wait for profile to settle */

    /* Set UPDATE_FLAG AND SOC INTERRUPT VALUE */
    uint8_t alert_val = CW_CONFIG_UPDATE_FLG | CW_GPIO_SOC_IRQ_VALUE;
    if (cw_i2c_write_byte(data->i2c_dev, CW_REG_SOC_ALERT, alert_val) != ESP_OK)
        return -1;

    /* Close all interrupts */
    if (cw_i2c_write_byte(data->i2c_dev, CW_REG_GPIO_CONFIG, 0x00) != ESP_OK)
        return -1;

    /* Activate */
    if (cw2217e_enter_active(data) < 0) {
        ESP_LOGE(TAG, "Failed to activate");
        return -1;
    }

    /* Wait for IC ready (reference code uses ~100ms per poll, up to 50 times) */
    int count = 0;
    while (1) {
        delay_ms(100);
        uint8_t state;
        if (cw2217e_read_ic_state(data, &state) != 0) return -1;
        if ((state & CW_IC_READY_MARK) == CW_IC_READY_MARK) break;
        count++;
        if (count >= CW_SLEEP_COUNTS) {
            cw2217e_enter_sleep(data);
            ESP_LOGE(TAG, "Timeout waiting for IC ready (state=0x%02X)", state);
            return -1;
        }
    }

    return 0;
}

/* ========== Chip ID Monitor Task ========== */

static void chip_id_monitor_task(void *pvParameters)
{
    cw2217e_data_t *data = (cw2217e_data_t *)pvParameters;

    ESP_LOGI(TAG, "Chip ID monitor task started (2s interval)");

    while (s_chip_id_task_running) {
        uint8_t chip_id = 0;
        esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_CHIP_ID, &chip_id);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Chip ID read: 0x%02X (%s)", chip_id,
                     chip_id == CW_IC_VCHIP_ID ? "OK" : "MISMATCH");
        } else {
            ESP_LOGE(TAG, "Chip ID read FAILED (I2C error)");
        }
        delay_ms(2000);
    }

    ESP_LOGI(TAG, "Chip ID monitor task stopped");
    s_chip_id_task_handle = NULL;
    vTaskDelete(NULL);
}

void cw2217e_start_chip_id_monitor(cw2217e_handle_t handle)
{
    if (s_chip_id_task_handle != NULL) {
        ESP_LOGW(TAG, "Chip ID monitor already running");
        return;
    }

    s_chip_id_task_running = true;
    BaseType_t ret = xTaskCreate(
        chip_id_monitor_task,
        "cw_chip_id",
        3072,
        (void *)handle,
        5,
        &s_chip_id_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create chip ID monitor task");
        s_chip_id_task_running = false;
    }
}

void cw2217e_start_chip_id_monitor_from_bus(i2c_master_bus_handle_t i2c_bus)
{
    if (s_chip_id_task_handle != NULL) {
        ESP_LOGW(TAG, "Chip ID monitor already running");
        return;
    }

    cw2217e_data_t *data = (cw2217e_data_t *)calloc(1, sizeof(cw2217e_data_t));
    if (!data) {
        ESP_LOGE(TAG, "Failed to allocate monitor data");
        return;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CW2217E_I2C_ADDRESS,
        .scl_speed_hz = CW2217E_I2C_FREQ_HZ,
    };

    if (i2c_master_bus_add_device(i2c_bus, &dev_config, &data->i2c_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device for chip ID monitor");
        free(data);
        return;
    }

    cw2217e_start_chip_id_monitor((cw2217e_handle_t)data);
}

void cw2217e_stop_chip_id_monitor(void)
{
    if (s_chip_id_task_handle == NULL) return;
    s_chip_id_task_running = false;

    int timeout = 20;
    while (s_chip_id_task_handle != NULL && timeout-- > 0) {
        delay_ms(100);
    }
}

/* ========== CW2217E Native API Implementation ========== */

cw2217e_handle_t cw2217e_create(const cw2217e_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, NULL, TAG, "Invalid config");
    ESP_RETURN_ON_FALSE(config->i2c_bus != NULL, NULL, TAG, "Invalid i2c_bus");

    cw2217e_data_t *handle = (cw2217e_data_t *)calloc(1, sizeof(cw2217e_data_t));
    if (!handle) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return NULL;
    }

    /* Add I2C device */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CW2217E_I2C_ADDRESS,
        .scl_speed_hz = CW2217E_I2C_FREQ_HZ,
    };

    if (i2c_master_bus_add_device(config->i2c_bus, &dev_config, &handle->i2c_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device (addr=0x%02X)", CW2217E_I2C_ADDRESS);
        free(handle);
        return NULL;
    }

    /* Verify chip ID */
    uint8_t chip_id = 0;
    esp_err_t ret = cw_i2c_read_byte(handle->i2c_dev, CW_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed when reading chip ID");
        goto err;
    }
    ESP_LOGI(TAG, "Chip ID: 0x%02X (expected 0x%02X)", chip_id, CW_IC_VCHIP_ID);
    if (chip_id != CW_IC_VCHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X != 0x%02X", chip_id, CW_IC_VCHIP_ID);
        goto err;
    }

    /* Read FW version */
    uint8_t fw_version = 0;
    cw_i2c_read_byte(handle->i2c_dev, CW_REG_FW_VERSION, &fw_version);
    ESP_LOGI(TAG, "FW version: 0x%02X", fw_version);

    /* Check IC state and configure if needed */
    int state = cw2217e_check_state(handle);
    if (state < 0) {
        ESP_LOGE(TAG, "Failed to read IC state (I2C error)");
        goto err;
    }

    if (state != 0) {
        ESP_LOGI(TAG, "IC state=%d, configuring battery profile...", state);
        /* Retry init up to 3 times */
        bool configured = false;
        for (int i = 0; i < 3; i++) {
            if (cw2217e_config_start_ic(handle) == 0) {
                ESP_LOGI(TAG, "Battery profile configured successfully");
                configured = true;
                break;
            }
            if (i < 2) delay_ms(100);
        }

        /* Fallback: if config failed, try direct ACTIVATE (profile may already be in OTP) */
        if (!configured) {
            ESP_LOGW(TAG, "Config failed, trying direct ACTIVATE (OTP profile)...");
            delay_ms(200);  /* Wait for chip to recover from failed attempts */
            if (cw2217e_enter_active(handle) == 0) {
                ESP_LOGI(TAG, "Direct ACTIVATE succeeded");
                configured = true;
            }
        }

        if (!configured) {
            ESP_LOGE(TAG, "Failed to configure and activate CW2217E");
            goto err;
        }
    } else {
        ESP_LOGI(TAG, "Battery profile already configured, skipping update");
    }

    /* Wait for IC to become ready */
    uint8_t ic_state = 0;
    for (int i = 0; i < 30; i++) {
        cw2217e_read_ic_state(handle, &ic_state);
        if ((ic_state & CW_IC_READY_MARK) == CW_IC_READY_MARK) break;
        delay_ms(100);
    }
    ESP_LOGI(TAG, "IC state: 0x%02X (ready=%s)", ic_state,
             (ic_state & CW_IC_READY_MARK) == CW_IC_READY_MARK ? "yes" : "no");

    ESP_LOGI(TAG, "CW2217E initialization complete");
    return handle;

err:
    if (handle->i2c_dev) {
        i2c_master_bus_rm_device(handle->i2c_dev);
    }
    free(handle);
    return NULL;
}

esp_err_t cw2217e_delete(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    /* Stop chip ID monitor if running */
    cw2217e_stop_chip_id_monitor();

    if (data->i2c_dev) {
        i2c_master_bus_rm_device(data->i2c_dev);
    }
    free(handle);
    return ESP_OK;
}

uint16_t cw2217e_get_fw_version(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;
    uint8_t fw = 0;
    cw_i2c_read_byte(data->i2c_dev, CW_REG_FW_VERSION, &fw);
    return fw;
}

uint16_t cw2217e_get_hw_version(cw2217e_handle_t handle)
{
    /* CW2217E has no separate HW version register */
    return 0;
}

esp_err_t cw2217e_seal(cw2217e_handle_t handle)
{
    /* CW2217E has no seal/unseal concept */
    return ESP_OK;
}

esp_err_t cw2217e_unseal(cw2217e_handle_t handle)
{
    /* CW2217E has no seal/unseal concept */
    return ESP_OK;
}

esp_err_t cw2217e_set_parameter_u16(cw2217e_handle_t handle, uint16_t address, uint16_t value)
{
    /* CW2217E uses battery profile mechanism, not individual parameter writes */
    return ESP_OK;
}

uint16_t cw2217e_get_parameter_u16(cw2217e_handle_t handle, uint16_t address)
{
    return 0;
}

uint16_t cw2217e_get_voltage(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    uint16_t raw = 0;
    if (cw_read_word(data->i2c_dev, CW_REG_VCELL_H, &raw) != 0)
        return 0;

    /* Convert ADC value to mV: raw * 5 / 16 */
    return (uint16_t)((uint32_t)raw * 5 / 16);
}

int16_t cw2217e_get_current(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    /* Read FW version for scaling factor */
    uint8_t fw_version = 0;
    cw_i2c_read_byte(data->i2c_dev, CW_REG_FW_VERSION, &fw_version);

    uint16_t raw = 0;
    if (cw_read_word(data->i2c_dev, CW_REG_CURRENT_H, &raw) != 0)
        return 0;

    /* 2's complement conversion */
    int32_t current;
    if (raw & 0x8000) {
        current = -(int32_t)((0xFFFF - raw) + 1);
    } else {
        current = (int32_t)raw;
    }

    /* Scale based on FW version (CW2215/17 use *1600/Rsense) */
    if ((fw_version & 0x80) || (fw_version & 0x40)) {
        current = current * 1600 / USER_RSENSE;
    } else if (fw_version != 0 && (fw_version & 0xC0) == 0x00) {
        current = current * 3815 / USER_RSENSE;
    } else {
        current = 0;
    }

    return (int16_t)current;
}

int16_t cw2217e_get_avgcurrent(cw2217e_handle_t handle)
{
    /* CW2217E doesn't have separate average current */
    return cw2217e_get_current(handle);
}

uint16_t cw2217e_get_cycle_count(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    uint16_t raw = 0;
    if (cw_read_word(data->i2c_dev, CW_REG_CYCLE_H, &raw) != 0)
        return 0;

    return raw / 16;
}

esp_err_t cw2217e_get_battery_status(cw2217e_handle_t handle, battery_status_t *battery_status)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(battery_status, ESP_ERR_INVALID_ARG, TAG, "Invalid status ptr");

    memset(battery_status, 0, sizeof(battery_status_t));

    int16_t current = cw2217e_get_current(handle);
    battery_status->DSG = (current > 0);     /* Discharging */
    battery_status->BATTPRES = true;          /* Battery always present */

    return ESP_OK;
}

esp_err_t cw2217e_get_operation_status(cw2217e_handle_t handle, operation_status_t *operation_status)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(operation_status, ESP_ERR_INVALID_ARG, TAG, "Invalid status ptr");

    cw2217e_data_t *data = (cw2217e_data_t *)handle;
    memset(operation_status, 0, sizeof(operation_status_t));

    uint8_t ic_state = 0;
    cw2217e_read_ic_state(data, &ic_state);
    operation_status->INITCOMP = ((ic_state & CW_IC_READY_MARK) == CW_IC_READY_MARK) ? 1 : 0;

    return ESP_OK;
}

uint16_t cw2217e_get_temperature(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    uint8_t reg_val = 0;
    esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_TEMP, &reg_val);
    if (ret != ESP_OK) return 0;

    /* CW2217E: temp in 0.1C = reg_val * 10 / 2 - 400
     * BQ27220 returns 0.1K: convert C + 273.15 -> 0.1K = 0.1C + 2731.5 */
    int temp_01c = (int)reg_val * 10 / 2 - 400;
    return (uint16_t)(temp_01c + 2732);
}

uint16_t cw2217e_get_full_charge_capacity(cw2217e_handle_t handle)
{
    return CW_DESIGN_CAPACITY_MAH;
}

uint16_t cw2217e_get_design_capacity(cw2217e_handle_t handle)
{
    return CW_DESIGN_CAPACITY_MAH;
}

uint16_t cw2217e_get_remaining_capacity(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    uint16_t soc = cw2217e_get_state_of_charge(handle);
    return (uint16_t)((uint32_t)soc * CW_DESIGN_CAPACITY_MAH / 100);
}

uint16_t cw2217e_get_state_of_charge(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    uint16_t raw = 0;
    if (cw_read_word(data->i2c_dev, CW_REG_SOC_INT, &raw) != 0)
        return 0;

    uint8_t soc_int = raw >> 8;
    uint8_t soc_decimal = raw & 0xFF;

    uint16_t ui_soc = (uint16_t)(((uint32_t)soc_int * 256 + soc_decimal) * 100 / (100 * 256));

    if (ui_soc > 100) ui_soc = 100;
    return ui_soc;
}

uint16_t cw2217e_get_state_of_health(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    uint8_t soh = 0;
    esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_SOH, &soh);
    if (ret != ESP_OK) return 0;

    /* Reference code: SOH = register_value + 5, capped at 100 */
    soh += 5;
    if (soh > 100) soh = 100;
    return soh;
}

/* ========== CW2217E Extended API (from reference code) ========== */

int cw2217e_get_chip_id(cw2217e_handle_t handle, uint8_t *chip_id)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(chip_id, ESP_ERR_INVALID_ARG, TAG, "Invalid chip_id ptr");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;
    esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_CHIP_ID, chip_id);
    return (ret == ESP_OK) ? 0 : -1;
}

esp_err_t cw2217e_sleep(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;
    return (cw2217e_enter_sleep(data) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t cw2217e_wakeup(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;
    return (cw2217e_enter_active(data) == 0) ? ESP_OK : ESP_FAIL;
}

uint8_t cw2217e_get_mode_config(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0xFF, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;
    uint8_t val = 0xFF;
    cw_i2c_read_byte(data->i2c_dev, CW_REG_MODE_CONFIG, &val);
    return val;
}

esp_err_t cw2217e_write_temperature(cw2217e_handle_t handle, int temperature)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    if (temperature < -40 || temperature > 87) {
        ESP_LOGE(TAG, "Invalid temperature %d (range: -40~87)", temperature);
        return ESP_ERR_INVALID_ARG;
    }

    /* A0 = (temperature + 40) * 2 */
    uint8_t a0_value = (uint8_t)((temperature + 40) * 2);

    /* A1 = ~read(REG_T_HOST_L) */
    uint8_t a1_value = 0;
    esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_T_HOST_L, &a1_value);
    if (ret != ESP_OK) return ret;
    a1_value = ~a1_value;

    ret = cw_i2c_write_byte(data->i2c_dev, CW_REG_T_HOST_H, a0_value);
    if (ret != ESP_OK) return ret;
    ret = cw_i2c_write_byte(data->i2c_dev, CW_REG_T_HOST_L, a1_value);
    return ret;
}

int cw2217e_get_temperature_celsius(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    uint8_t reg_val = 0;
    esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, CW_REG_TEMP, &reg_val);
    if (ret != ESP_OK) return 0;

    /* Returns temperature in 0.1°C: reg_val * 10 / 2 - 400 */
    return (int)reg_val * 10 / 2 - 400;
}

uint8_t cw2217e_get_ic_state(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;
    uint8_t state = 0;
    cw2217e_read_ic_state(data, &state);
    return state;
}

void cw2217e_dump_register(cw2217e_handle_t handle)
{
    if (!handle) {
        ESP_LOGE(TAG, "Invalid handle");
        return;
    }
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    ESP_LOGI(TAG, "=== CW2217E Register Dump ===");
    for (int i = 0; i <= 0xFF; i++) {
        uint8_t reg_val = 0;
        esp_err_t ret = cw_i2c_read_byte(data->i2c_dev, (uint8_t)i, &reg_val);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "REG[0x%02X] = 0x%02X", i, reg_val);
        } else {
            ESP_LOGE(TAG, "REG[0x%02X] = READ FAILED", i);
        }
    }
    ESP_LOGI(TAG, "=== Register Dump End ===");
}

/* ========== Stub functions for unsupported features ========== */

uint16_t cw2217e_get_raw_coulomb_count(cw2217e_handle_t handle)
{
    /* CW2217E does not expose raw coulomb count */
    (void)handle;
    return 0;
}

uint16_t cw2217e_get_charge_voltage(cw2217e_handle_t handle)  { return 0; }
uint16_t cw2217e_get_charge_current(cw2217e_handle_t handle)  { return 0; }
int16_t  cw2217e_get_average_power(cw2217e_handle_t handle)   { return 0; }
uint16_t cw2217e_get_time_to_empty(cw2217e_handle_t handle)   { return 0; }
uint16_t cw2217e_get_time_to_full(cw2217e_handle_t handle)    { return 0; }
int16_t  cw2217e_get_maxload_current(cw2217e_handle_t handle)  { return 0; }
int16_t  cw2217e_get_standby_current(cw2217e_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, 0, TAG, "Invalid handle");
    cw2217e_data_t *data = (cw2217e_data_t *)handle;

    uint16_t raw = 0;
    if (cw_read_word(data->i2c_dev, CW_REG_STB_CUR_H, &raw) != 0)
        return 0;

    /* 2's complement conversion */
    int32_t current;
    if (raw & 0x8000) {
        current = -(int32_t)((0xFFFF - raw) + 1);
    } else {
        current = (int32_t)raw;
    }
    return (int16_t)(current * 1600 / USER_RSENSE);
}
