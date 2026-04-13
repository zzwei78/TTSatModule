/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "math.h"
#include "IP5561.h"
#include "priv_include/IP5561_reg.h"

static const char *TAG = "IP5561";

// Default I2C frequency
#define IP5561_I2C_FREQ_DEFAULT    200000  // 200kHz (recommended by datasheet)

// Delay macro
#define delay_ms(x) vTaskDelay(pdMS_TO_TICKS(x))

// Error printing macro
#define PRINT_ERROR(err) \
    if (err != ESP_OK) { \
        ESP_LOGE(TAG, "(%s:%d) Error: %s", __func__, __LINE__, esp_err_to_name(err)); \
    }

typedef struct {
    i2c_master_dev_handle_t i2c_ctrl_handle;   /* I2C device for 0xE8 (Control registers) */
    i2c_master_dev_handle_t i2c_stat_handle;   /* I2C device for 0xEA (Status/ADC registers) */
    ip5561_wakeup_callback_t wakeup_callback;  /* Wakeup callback for I2C communication */
} ip5561_data_t;

/* ========== Wakeup Helper ========== */

/**
 * @brief Invoke wakeup callback if registered
 */
static inline void ip5561_wakeup_if_needed(ip5561_handle_t handle)
{
    if (handle != NULL) {
        ip5561_data_t *ip_data = (ip5561_data_t *)handle;
        if (ip_data->wakeup_callback != NULL) {
            ip_data->wakeup_callback();
        }
    }
}

/* ========== I2C Communication Functions ========== */

/**
 * @brief I2C read using control address (0xE8)
 */
static esp_err_t ip5561_i2c_read_ctrl(ip5561_handle_t handle, uint8_t reg, uint8_t *data)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid data pointer");

    ip5561_wakeup_if_needed(handle);  /* Wakeup before I2C communication */

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    return i2c_master_transmit_receive(ip_data->i2c_ctrl_handle, &reg, 1, data, 1, -1);
}

/**
 * @brief I2C write using control address (0xE8)
 */
static esp_err_t ip5561_i2c_write_ctrl(ip5561_handle_t handle, uint8_t reg, uint8_t data)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ip5561_wakeup_if_needed(handle);  /* Wakeup before I2C communication */

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(ip_data->i2c_ctrl_handle, buf, sizeof(buf), -1);
}

/**
 * @brief I2C read using status address (0xEA)
 */
static esp_err_t ip5561_i2c_read_stat(ip5561_handle_t handle, uint8_t reg, uint8_t *data)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid data pointer");

    ip5561_wakeup_if_needed(handle);  /* Wakeup before I2C communication */

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    return i2c_master_transmit_receive(ip_data->i2c_stat_handle, &reg, 1, data, 1, -1);
}

/**
 * @brief I2C write using status address (0xEA)
 */
static esp_err_t ip5561_i2c_write_stat(ip5561_handle_t handle, uint8_t reg, uint8_t data)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ip5561_wakeup_if_needed(handle);  /* Wakeup before I2C communication */

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(ip_data->i2c_stat_handle, buf, sizeof(buf), -1);
}

/* ========== I2C Communication Functions ========== */

/**
 * @brief I2C read multiple bytes using control address (0xE8)
 */
static esp_err_t ip5561_i2c_read_ctrl_bytes(ip5561_handle_t handle, uint8_t reg, uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid data pointer");
    ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid length");

    ip5561_wakeup_if_needed(handle);  /* Wakeup before I2C communication */

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    return i2c_master_transmit_receive(ip_data->i2c_ctrl_handle, &reg, 1, data, len, -1);
}

/**
 * @brief I2C write multiple bytes using control address (0xE8)
 */
static esp_err_t ip5561_i2c_write_ctrl_bytes(ip5561_handle_t handle, uint8_t reg, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid data pointer");
    ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid length");

    ip5561_wakeup_if_needed(handle);  /* Wakeup before I2C communication */

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    uint8_t buf[1 + len];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    return i2c_master_transmit(ip_data->i2c_ctrl_handle, buf, sizeof(buf), -1);
}

/**
 * @brief I2C read multiple bytes using status address (0xEA)
 */
__attribute__((unused)) static esp_err_t ip5561_i2c_read_stat_bytes(ip5561_handle_t handle, uint8_t reg, uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid data pointer");
    ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid length");

    ip5561_wakeup_if_needed(handle);  /* Wakeup before I2C communication */

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    return i2c_master_transmit_receive(ip_data->i2c_stat_handle, &reg, 1, data, len, -1);
}

/**
 * @brief I2C write multiple bytes using status address (0xEA)
 */
static esp_err_t ip5561_i2c_write_stat_bytes(ip5561_handle_t handle, uint8_t reg, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid data pointer");
    ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid length");

    ip5561_wakeup_if_needed(handle);  /* Wakeup before I2C communication */

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    uint8_t buf[1 + len];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    return i2c_master_transmit(ip_data->i2c_stat_handle, buf, sizeof(buf), -1);
}

/* ========== Public API Implementation ========== */

ip5561_handle_t ip5561_create(const ip5561_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, NULL, TAG, "Invalid config");
    ESP_RETURN_ON_FALSE(config->i2c_bus != NULL, NULL, TAG, "Invalid i2c_bus");

    esp_err_t ret = ESP_OK;
    ip5561_data_t *handle = (ip5561_data_t *)calloc(1, sizeof(ip5561_data_t));
    if (handle == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return NULL;
    }

    // Configure I2C frequency
    uint32_t freq_hz = (config->scl_freq_hz > 0) ? config->scl_freq_hz : IP5561_I2C_FREQ_DEFAULT;

    // Create first I2C device for control registers (0xE8/0xE9)
    i2c_device_config_t dev_ctrl_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IP5561_I2C_ADDR_CTRL_WRITE >> 1,  // Convert 8-bit to 7-bit address
        .scl_speed_hz = freq_hz,
    };

    ret = i2c_master_bus_add_device(config->i2c_bus, &dev_ctrl_config, &handle->i2c_ctrl_handle);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to add I2C control device (0xE8)");

    // Create second I2C device for status/ADC registers (0xEA/0xEB)
    i2c_device_config_t dev_stat_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IP5561_I2C_ADDR_STAT_WRITE >> 1,  // Convert 8-bit to 7-bit address
        .scl_speed_hz = freq_hz,
    };

    ret = i2c_master_bus_add_device(config->i2c_bus, &dev_stat_config, &handle->i2c_stat_handle);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to add I2C status device (0xEA)");

    // Verify device communication by reading SYS_CTL0 register (uses 0xE8)
    uint8_t sys_ctl0 = 0;
    ret = ip5561_i2c_read_ctrl((ip5561_handle_t)handle, IP55XX_SYS_CTL0, &sys_ctl0);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to communicate with device (0xE8)");

    ESP_LOGI(TAG, "IP5561 initialized successfully");
    ESP_LOGI(TAG, "  SYS_CTL0 (0xE8): 0x%02X", sys_ctl0);
    ESP_LOGI(TAG, "  Dual I2C addresses: 0xE8 (ctrl), 0xEA (stat)");


    return (ip5561_handle_t)handle;

err:
    if (handle != NULL) {
        if (handle->i2c_ctrl_handle != NULL) {
            i2c_master_bus_rm_device(handle->i2c_ctrl_handle);
        }
        if (handle->i2c_stat_handle != NULL) {
            i2c_master_bus_rm_device(handle->i2c_stat_handle);
        }
        free(handle);
    }
    return NULL;
}

esp_err_t ip5561_delete(ip5561_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;

    if (ip_data->i2c_ctrl_handle != NULL) {
        i2c_master_bus_rm_device(ip_data->i2c_ctrl_handle);
    }
    if (ip_data->i2c_stat_handle != NULL) {
        i2c_master_bus_rm_device(ip_data->i2c_stat_handle);
    }

    free(ip_data);
    ESP_LOGI(TAG, "IP5561 deleted");
    return ESP_OK;
}

esp_err_t ip5561_set_wakeup_callback(ip5561_handle_t handle, ip5561_wakeup_callback_t callback)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ip5561_data_t *ip_data = (ip5561_data_t *)handle;
    ip_data->wakeup_callback = callback;

    ESP_LOGI(TAG, "Wakeup callback %s", callback ? "registered" : "disabled");
    return ESP_OK;
}

/* ========== System Control Functions ========== */

esp_err_t ip5561_set_charge_enable(ip5561_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t sys_ctl0 = 0;
    esp_err_t ret = ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL0, &sys_ctl0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read SYS_CTL0");

    // Set or clear bit 0 (charger enable)
    if (enable) {
        sys_ctl0 |= IP55XX_SYS_CTL0_EN_CHARGER;
    } else {
        sys_ctl0 &= ~IP55XX_SYS_CTL0_EN_CHARGER;
    }

    ret = ip5561_i2c_write_ctrl(handle, IP55XX_SYS_CTL0, sys_ctl0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write SYS_CTL0");

    ESP_LOGD(TAG, "Charge %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

bool ip5561_get_charge_enable(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return false;
    }

    uint8_t sys_ctl0 = 0;
    if (ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL0, &sys_ctl0) != ESP_OK) {
        return false;
    }

    return (sys_ctl0 & IP55XX_SYS_CTL0_EN_CHARGER) != 0;
}

esp_err_t ip5561_set_boost_enable(ip5561_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t sys_ctl0 = 0;
    esp_err_t ret = ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL0, &sys_ctl0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read SYS_CTL0");

    // Set or clear bit 1 (boost enable)
    if (enable) {
        sys_ctl0 |= IP55XX_SYS_CTL0_EN_BOOST;
    } else {
        sys_ctl0 &= ~IP55XX_SYS_CTL0_EN_BOOST;
    }

    ret = ip5561_i2c_write_ctrl(handle, IP55XX_SYS_CTL0, sys_ctl0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write SYS_CTL0");

    ESP_LOGD(TAG, "Boost mode %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

/* ========== Wireless Power Charging Functions ========== */

esp_err_t ip5561_set_wpc_enable(ip5561_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t vwpc_ctl0 = 0;
    esp_err_t ret = ip5561_i2c_read_ctrl(handle, IP55XX_VWPC_CTL0, &vwpc_ctl0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read VWPC_CTL0");

    // Set or clear bit 0 (wireless charger enable)
    if (enable) {
        vwpc_ctl0 |= IP55XX_VWPC_CTL0_EN_WPC;
    } else {
        vwpc_ctl0 &= ~IP55XX_VWPC_CTL0_EN_WPC;
    }

    ret = ip5561_i2c_write_ctrl(handle, IP55XX_VWPC_CTL0, vwpc_ctl0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write VWPC_CTL0");

    ESP_LOGI(TAG, "Wireless charger %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

bool ip5561_get_wpc_enable(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return false;
    }

    uint8_t vwpc_ctl0 = 0;
    if (ip5561_i2c_read_ctrl(handle, IP55XX_VWPC_CTL0, &vwpc_ctl0) != ESP_OK) {
        return false;
    }

    return (vwpc_ctl0 & IP55XX_VWPC_CTL0_EN_WPC) != 0;
}

esp_err_t ip5561_get_wpc_status(ip5561_handle_t handle, ip5561_wpc_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid status pointer");

    memset(status, 0, sizeof(ip5561_wpc_status_t));

    // Read VWPC_CTL0 register (0xE8)
    uint8_t vwpc_ctl0 = 0;
    esp_err_t ret = ip5561_i2c_read_ctrl(handle, IP55XX_VWPC_CTL0, &vwpc_ctl0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read VWPC_CTL0");

    status->wpc_ctl0 = vwpc_ctl0;
    status->wpc_enabled = (vwpc_ctl0 & IP55XX_VWPC_CTL0_EN_WPC) != 0;
    status->wpc_present = (vwpc_ctl0 & IP55XX_VWPC_CTL0_WPC_DET) != 0;
    status->wpc_charging = (vwpc_ctl0 & IP55XX_VWPC_CTL0_WPC_CHG) != 0;
    status->wpc_done = (vwpc_ctl0 & IP55XX_VWPC_CTL0_WPC_DONE) != 0;

    return ESP_OK;
}

bool ip5561_is_wpc_present(ip5561_handle_t handle)
{
    ip5561_wpc_status_t status;
    if (ip5561_get_wpc_status(handle, &status) != ESP_OK) {
        return false;
    }

    return status.wpc_present;
}

bool ip5561_is_wpc_charging(ip5561_handle_t handle)
{
    ip5561_wpc_status_t status;
    if (ip5561_get_wpc_status(handle, &status) != ESP_OK) {
        return false;
    }

    return status.wpc_charging;
}

esp_err_t ip5561_get_ocp_state(ip5561_handle_t handle, uint8_t *ocp_state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(ocp_state != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid ocp_state pointer");

    // Read OCP_STATE register (0xFC at 0xEA)
    esp_err_t ret = ip5561_i2c_read_stat(handle, IP55XX_OCP_STATE, ocp_state);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read OCP_STATE");

    // Parse and log OCP status
    ESP_LOGD(TAG, "OCP_STATE (0xFC): 0x%02X", *ocp_state);
    if (*ocp_state & 0x04) {
        ESP_LOGW(TAG, "  Boost OCP triggered! (current limit exceeded)");
    }
    if (*ocp_state & 0x08) {
        ESP_LOGE(TAG, "  Boost short circuit detected!");
    }
    if (*ocp_state & 0x01) {
        ESP_LOGW(TAG, "  VBUS OCP triggered");
    }
    if (*ocp_state & 0x02) {
        ESP_LOGW(TAG, "  VOUT OCP triggered");
    }

    return ESP_OK;
}

esp_err_t ip5561_get_sys_ctl0(ip5561_handle_t handle, uint8_t *sys_ctl0)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(sys_ctl0 != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid sys_ctl0 pointer");

    return ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL0, sys_ctl0);
}

esp_err_t ip5561_set_sys_ctl0(ip5561_handle_t handle, uint8_t sys_ctl0)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    return ip5561_i2c_write_ctrl(handle, IP55XX_SYS_CTL0, sys_ctl0);
}

esp_err_t ip5561_get_sys_ctl1(ip5561_handle_t handle, uint8_t *sys_ctl1)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(sys_ctl1 != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid sys_ctl1 pointer");

    return ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL1, sys_ctl1);
}

esp_err_t ip5561_set_sys_ctl1(ip5561_handle_t handle, uint8_t sys_ctl1)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    return ip5561_i2c_write_ctrl(handle, IP55XX_SYS_CTL1, sys_ctl1);
}

esp_err_t ip5561_configure_light_load(ip5561_handle_t handle, bool enable_current_detect,
                                       bool enable_power_detect, uint8_t timeout_mode)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t sys_ctl1 = 0;
    esp_err_t ret = ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL1, &sys_ctl1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read SYS_CTL1");

    // Configure light load detection bits
    if (enable_current_detect) {
        sys_ctl1 |= IP55XX_SYS_CTL1_EN_ISYS_LOW;
    } else {
        sys_ctl1 &= ~IP55XX_SYS_CTL1_EN_ISYS_LOW;
    }

    if (enable_power_detect) {
        sys_ctl1 |= IP55XX_SYS_CTL1_EN_POW_LOW;
    } else {
        sys_ctl1 &= ~IP55XX_SYS_CTL1_EN_POW_LOW;
    }

    // Set timeout mode (bits 5:4)
    sys_ctl1 &= ~IP55XX_SYS_CTL1_EN_ILOW_TIME_MASK;
    sys_ctl1 |= ((timeout_mode & 0x3) << IP55XX_SYS_CTL1_EN_ILOW_TIME_SHIFT);

    ret = ip5561_i2c_write_ctrl(handle, IP55XX_SYS_CTL1, sys_ctl1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write SYS_CTL1");

    ESP_LOGI(TAG, "Light load detection configured: current=%s, power=%s, timeout=%d",
             enable_current_detect ? "EN" : "DIS",
             enable_power_detect ? "EN" : "DIS",
             timeout_mode);

    return ESP_OK;
}

/* ========== Fast Charge Protocol Control ========== */

esp_err_t ip5561_disable_fast_charge(ip5561_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_err_t ret;
    uint8_t reg_value;

    // Disable QC (Quick Charge) protocol detection
    // QC_CTRL0 (0x81) - Bit 0: QC enable
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_QC_CTRL0, &reg_value);
    if (ret == ESP_OK) {
        reg_value &= ~(1 << 0);  // Clear bit 0 to disable QC
        ip5561_i2c_write_ctrl(handle, IP55XX_QC_CTRL0, reg_value);
        ESP_LOGI(TAG, "QC protocol disabled (QC_CTRL0: 0x%02X)", reg_value);
    } else {
        ESP_LOGW(TAG, "Failed to read QC_CTRL0, may not be supported");
    }

    // Disable PD (Power Delivery) protocol detection
    // PD_CTRL (0xD4)
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_PD_CTRL, &reg_value);
    if (ret == ESP_OK) {
        // Clear all enable bits (typically bit 0 or bit 7)
        reg_value = 0x00;
        ip5561_i2c_write_ctrl(handle, IP55XX_PD_CTRL, reg_value);
        ESP_LOGI(TAG, "PD protocol disabled (PD_CTRL: 0x%02X)", reg_value);
    } else {
        ESP_LOGW(TAG, "Failed to read PD_CTRL, may not be supported");
    }

    ESP_LOGI(TAG, "Fast charge protocols disabled - using 5V standard charging");
    return ESP_OK;
}

esp_err_t ip5561_print_diagnostics(ip5561_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_err_t ret;
    uint8_t reg_value;

    ESP_LOGI(TAG, "========== IP5561 Diagnostic Register Dump ==========");

    // SYS_CTL0
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL0, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SYS_CTL0 (0x00): 0x%02X", reg_value);
        ESP_LOGI(TAG, "  Charger: %s | Boost: %s | Auto-Boost: %s",
                 (reg_value & 0x01) ? "EN" : "DIS",
                 (reg_value & 0x02) ? "EN" : "DIS",
                 (reg_value & 0x04) ? "EN" : "DIS");
    }

    // SYS_CTL1
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL1, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SYS_CTL1 (0x03): 0x%02X", reg_value);
        ESP_LOGI(TAG, "  ILOW Current: %s | ILOW Power: %s | ILOW Timeout: %d",
                 (reg_value & 0x04) ? "EN" : "DIS",
                 (reg_value & 0x08) ? "EN" : "DIS",
                 (reg_value >> 4) & 0x03);
    }

    // SYS_STAT0
    ret = ip5561_i2c_read_stat(handle, 0x01, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SYS_STAT0 (0x01): 0x%02X", reg_value);
        // Note: Bit definition unclear, only showing raw value
    }

    // QC_CTRL0
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_QC_CTRL0, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "QC_CTRL0 (0x81): 0x%02X - QC Protocol %s",
                 reg_value, (reg_value & 0x01) ? "ENABLED" : "disabled");
    } else {
        ESP_LOGW(TAG, "QC_CTRL0 (0x81): Not readable (may not be supported)");
    }

    // PD_CTRL
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_PD_CTRL, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PD_CTRL (0xD4): 0x%02X - PD Protocol %s",
                 reg_value, (reg_value != 0) ? "ENABLED" : "disabled");
    } else {
        ESP_LOGW(TAG, "PD_CTRL (0xD4): Not readable (may not be supported)");
    }

    // VBUS_CTL0
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_VBUS_CTL0, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "VBUS_CTL0 (0x18): 0x%02X", reg_value);
    }

    // VOUT_CTL0
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_VOUT_CTL0, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "VOUT_CTL0 (0x10): 0x%02X", reg_value);
    }

    // Charge timeout control
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_CHG_TMO_CTL1, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CHG_TMO_CTL1 (0x21): 0x%02X - Timer %s, Timeout=%d",
                 reg_value,
                 (reg_value & 0x80) ? "ENABLED" : "disabled",
                 (reg_value >> 4) & 0x07);
    }

    // VOUT_CTL1 - Boost output control
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_VOUT_CTL1, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "VOUT_CTL1 (0x13): 0x%02X - Boost control register", reg_value);
    }

    // VOUT_CTL2 - Boost protection
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_VOUT_CTL2, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "VOUT_CTL2 (0x1C): 0x%02X - Boost protection register", reg_value);
    }

    // NTC Enable - Temperature protection status
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_NTC_ENABLE, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NTC_ENABLE (0xFD): 0x%02X - Temp protection %s",
                 reg_value, (reg_value & 0x01) ? "ENABLED" : "DISABLED");
    }

    // MOS Control
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_MOS_CTRL, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MOS_CTRL (0x86): 0x%02X - MOSFET control register", reg_value);
    }

    // MOS Status
    ret = ip5561_i2c_read_stat(handle, IP55XX_MOS_STATUS, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MOS_STATUS (0x87): 0x%02X - MOSFET status (bit0=%s)",
                 reg_value, (reg_value & 0x01) ? "ON" : "OFF");
    }

    // ========== Additional Status Registers for Fault Detection ==========
    ESP_LOGI(TAG, "=== Fault Detection Registers ===");

    // BATLOW (0x03) - Battery low voltage threshold
    ret = ip5561_i2c_read_stat(handle, IP55XX_BATLOW, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BATLOW (0x03): 0x%02X - Battery low voltage threshold", reg_value);
    }

    // GPIO_20UA_EN (0x19) - GPIO 20uA current output enable
    ret = ip5561_i2c_read_stat(handle, IP55XX_GPIO_20UA_EN, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GPIO_20UA_EN (0x19): 0x%02X - GPIO 20uA enable", reg_value);
    }

    // STAT_A1 through STAT_AF - Various fault/status registers
    uint8_t stat_a1, stat_a4, stat_a5, stat_a8, stat_af;
    ret = ip5561_i2c_read_stat(handle, IP55XX_FCP_STATUS, &stat_a1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "FCP_STATUS (0xA1): 0x%02X - FCP status register", stat_a1);
    }

    ret = ip5561_i2c_read_stat(handle, IP55XX_STATUS_SRC0, &stat_a4);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "STATUS_SRC0 (0xA4): 0x%02X - SRC fast charge status 0", stat_a4);
    }

    ret = ip5561_i2c_read_stat(handle, IP55XX_STATUS_SRC1, &stat_a5);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "STATUS_SRC1 (0xA5): 0x%02X - SRC fast charge status 1", stat_a5);
    }

    ret = ip5561_i2c_read_stat(handle, IP55XX_STATUS_SRC2, &stat_a8);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "STATUS_SRC2 (0xA8): 0x%02X - SRC fast charge status 2", stat_a8);
    }

    ret = ip5561_i2c_read_stat(handle, IP55XX_AFC_STATUS, &stat_af);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AFC_STATUS (0xAF): 0x%02X - AFC status register", stat_af);
    }

    ESP_LOGI(TAG, "====================================================");

    return ESP_OK;
}

esp_err_t ip5561_disable_boost_protections(ip5561_handle_t handle, bool disable_ntc)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_err_t ret;
    uint8_t reg_value;

    ESP_LOGI(TAG, "Disabling boost output protections...");

    // 1. Disable charge timeout (CHG_TMO_CTL1)
    // Bit 7: Charge timer enable, Bit 6-4: Timer duration
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_CHG_TMO_CTL1, &reg_value);
    if (ret == ESP_OK) {
        reg_value &= ~(1 << 7);  // Disable charge timer
        ip5561_i2c_write_ctrl(handle, IP55XX_CHG_TMO_CTL1, reg_value);
        ESP_LOGI(TAG, "Charge timeout disabled (CHG_TMO_CTL1: 0x%02X)", reg_value);
    }

    // 2. Disable boost timeout
    // Some chips have separate boost timeout in VOUT_CTL1 or VOUT_CTL2
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_VOUT_CTL1, &reg_value);
    if (ret == ESP_OK) {
        // Check if there's a timeout bit to disable
        ESP_LOGI(TAG, "VOUT_CTL1 (0x13): 0x%02X - checking for timeout bits", reg_value);
        // Try to disable any timeout protection (implementation varies by chip)
        // This is chip-specific, may need adjustment based on actual datasheet
    }

    // 3. Check VBUS_CTL0 for any protection that might affect boost
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_VBUS_CTL0, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "VBUS_CTL0 (0x18): 0x%02X - checking for boost protections", reg_value);
        // VBUS undervoltage protection might affect boost operation
        // Clear protection enable bits if present
    }

    // 4. Handle NTC temperature protection
    // WARNING: Disabling NTC can be dangerous! Only for testing!
    // NTC_ENABLE register at 0xFD
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_NTC_ENABLE, &reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NTC_ENABLE (0xFD): 0x%02X - NTC protection is %s",
                 reg_value, (reg_value & 0x01) ? "ENABLED" : "DISABLED");

        if (disable_ntc) {
            ESP_LOGW(TAG, "⚠️ DANGEROUS: Disabling NTC temperature protection!");
            reg_value &= ~(1 << 0);  // Disable NTC
            ip5561_i2c_write_ctrl(handle, IP55XX_NTC_ENABLE, reg_value);
            ESP_LOGI(TAG, "NTC protection DISABLED (0xFD: 0x%02X)", reg_value);
        }
    }

    ESP_LOGW(TAG, "Boost protections configured - Monitor temperature carefully!");
    return ESP_OK;
}

esp_err_t ip5561_set_ntc_enable(ip5561_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_err_t ret;
    uint8_t reg_value;

    ret = ip5561_i2c_read_ctrl(handle, IP55XX_NTC_ENABLE, &reg_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NTC_ENABLE register");
        return ret;
    }

    if (enable) {
        reg_value |= (1 << 0);  // Set bit 0 to enable NTC
        ESP_LOGI(TAG, "Enabling NTC temperature protection");
    } else {
        reg_value &= ~(1 << 0);  // Clear bit 0 to disable NTC
        ESP_LOGW(TAG, "⚠️ DANGEROUS: Disabling NTC temperature protection!");
    }

    ret = ip5561_i2c_write_ctrl(handle, IP55XX_NTC_ENABLE, reg_value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NTC protection %s (0xFD: 0x%02X)",
                 enable ? "ENABLED" : "DISABLED", reg_value);
    }

    return ret;
}

esp_err_t ip5561_disable_ntc_temperature_protection(ip5561_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_err_t ret;
    uint8_t reg_value;

    /* Read current NTC_ENABLE register value */
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_NTC_ENABLE, &reg_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NTC_ENABLE register (0xFD)");
        return ret;
    }

    ESP_LOGW(TAG, "⚠️ DANGEROUS: Disabling NTC charge temperature protection!");
    ESP_LOGW(TAG, "⚠️ Monitor battery and chip temperature carefully!");

    /* Clear charge temperature protection bits
     * Bit 2: En_chg_ht - Charge high temp protection (45°C)
     * Bit 3: En_chg_lt - Charge low temp protection (0°C)
     * Preserve all other bits (0, 1, 4, 5, 6, 7)
     */
    reg_value &= ~(0x0C);  /* Clear bits 2 and 3 (mask = 0b00001100) */

    /* Write back to register */
    ret = ip5561_i2c_write_ctrl(handle, IP55XX_NTC_ENABLE, reg_value);
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "NTC charge temperature protection DISABLED (0xFD: 0x%02X)", reg_value);
        ESP_LOGW(TAG, "  - ChgHT (bit 2, 45°C): %s", (reg_value & 0x04) ? "ENABLED ❌" : "DISABLED ✅");
        ESP_LOGW(TAG, "  - ChgLT (bit 3, 0°C):  %s", (reg_value & 0x08) ? "ENABLED ❌" : "DISABLED ✅");
        ESP_LOGW(TAG, "  - Other NTC functions preserved");
    }

    return ret;
}

/* ========== Charging Voltage Control ========== */

esp_err_t ip5561_set_charge_voltage(ip5561_handle_t handle, uint16_t voltage_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(voltage_mv >= 4200 && voltage_mv <= 4400,
                       ESP_ERR_INVALID_ARG, TAG, "Voltage out of range (4200-4400mV)");

    // First enable register-based voltage setting (0xE8 0x33[4]=0)
    uint8_t reg_33 = 0;
    ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL5, &reg_33);  // SYS_CTL5, not CHG_CTL5
    reg_33 &= ~(1 << 4);  // Clear bit 4 to enable register control
    ip5561_i2c_write_ctrl(handle, IP55XX_SYS_CTL5, reg_33);

    // Set voltage in 0xEA 0x3A register
    uint8_t reg_3a = 0;
    ip5561_i2c_read_stat(handle, IP55XX_CHG_VOLT_SET, &reg_3a);

    // Set voltage bits [3:2]
    reg_3a &= ~IP55XX_CHG_VOLT_VSET_MASK;
    if (voltage_mv < 4250) {
        reg_3a |= IP55XX_CHG_VOLT_VSET_4V2;
    } else if (voltage_mv < 4325) {
        reg_3a |= IP55XX_CHG_VOLT_VSET_4V3;
    } else if (voltage_mv < 4375) {
        reg_3a |= IP55XX_CHG_VOLT_VSET_4V35;
    } else {
        reg_3a |= IP55XX_CHG_VOLT_VSET_4V4;
    }

    // Keep default RCV = 28mV
    reg_3a &= ~IP55XX_CHG_VOLT_RCV_MASK;
    reg_3a |= IP55XX_CHG_VOLT_RCV_28MV;

    esp_err_t ret = ip5561_i2c_write_stat(handle, IP55XX_CHG_VOLT_SET, reg_3a);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write CHG_VOLT_SET");

    ESP_LOGD(TAG, "Charge voltage set to %d mV", voltage_mv);
    return ESP_OK;
}

esp_err_t ip5561_set_9v_charge_current(ip5561_handle_t handle, uint16_t current_ma)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    // CHG_CTL5 register at 0x6F (0xE8 I2C address)
    // Controls 9V input charging current limit
    // Current formula: value = current_ma / 50 (e.g., 3500mA = 70 = 0x46)

    uint8_t current_reg_val = (uint8_t)(current_ma / 50);

    // Clamp to reasonable range (500mA to 5000mA)
    if (current_reg_val < 10) current_reg_val = 10;   // 500mA minimum
    if (current_reg_val > 100) current_reg_val = 100; // 5000mA maximum

    // Write to CHG_CTL5 (0x6F) at 0xE8 address
    esp_err_t ret = ip5561_i2c_write_ctrl(handle, IP55XX_CHG_CTL5, current_reg_val);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write CHG_CTL5");

    ESP_LOGI(TAG, "9V charge current limit set to %d mA (reg=0x%02X)", current_ma, current_reg_val);
    return ESP_OK;
}

esp_err_t ip5561_set_9v_uv_threshold(ip5561_handle_t handle, uint16_t uv_threshold_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    // CHG_CTL1 register at 0x0C (0xE8 I2C address)
    // Controls 9V input undervoltage loop threshold
    // High 4 bits [7:4] = VUV_9V threshold setting
    // Threshold mapping (example, adjust based on actual IP5561 datasheet):
    //   0x0 = 6.0V, 0x1 = 6.5V, 0x2 = 7.0V, 0x3 = 7.5V
    //   0x4 = 8.0V, 0x5 = 8.5V, 0x6 = 9.0V, etc.
    // Formula example: reg_val = ((uv_threshold_mv - 6000) / 500) & 0x0F

    // Clamp to reasonable range
    if (uv_threshold_mv < 6000) uv_threshold_mv = 6000;
    if (uv_threshold_mv > 9500) uv_threshold_mv = 9500;

    // Calculate register value for high 4 bits [7:4]
    uint8_t uv_threshold_reg = (uint8_t)((uv_threshold_mv - 6000) / 500);
    if (uv_threshold_reg > 0x0F) uv_threshold_reg = 0x0F;

    // Read current register value to preserve lower 4 bits
    uint8_t current_val = 0;
    esp_err_t ret = ip5561_i2c_read_ctrl(handle, IP55XX_CHG_CTL1, &current_val);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read CHG_CTL1");

    // Update only high 4 bits, preserve low 4 bits
    uint8_t new_val = (current_val & 0x0F) | (uv_threshold_reg << 4);

    // Write back to CHG_CTL1
    ret = ip5561_i2c_write_ctrl(handle, IP55XX_CHG_CTL1, new_val);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write CHG_CTL1");

    ESP_LOGI(TAG, "9V UV threshold set to %d mV (reg=0x%02X, VUV_9V=0x%X)",
             uv_threshold_mv, new_val, uv_threshold_reg);
    return ESP_OK;
}

esp_err_t ip5561_set_vbus_9v_current(ip5561_handle_t handle, uint16_t current_ma)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    // VBUS_9V register at 0xBB (0xE8 I2C address)
    // Controls VBUS 9V boost output current limit
    // Current formula: reg_val = current_ma / 50 (e.g., 1000mA = 20 = 0x14)

    uint8_t current_reg_val = (uint8_t)(current_ma / 50);

    // Clamp to reasonable range (100mA to 12750mA)
    if (current_reg_val < 2) {
        current_reg_val = 2;   // 100mA minimum (2 * 50 = 100)
    }

    // Write to VBUS_9V (0xBB) at 0xE8 address (control register)
    esp_err_t ret = ip5561_i2c_write_ctrl(handle, IP55XX_VBUS_9V, current_reg_val);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write VBUS_9V");

    ESP_LOGI(TAG, "VBUS 9V output current limit set to %d mA (reg=0x%02X)",
             current_ma, current_reg_val);
    return ESP_OK;
}

/* ========== Status Reading Functions ========== */

esp_err_t ip5561_get_charge_status(ip5561_handle_t handle, ip5561_chg_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid status pointer");

    memset(status, 0, sizeof(ip5561_chg_status_t));

    // Read SYS_CTL0 to get charger enable status (0xE8)
    uint8_t sys_ctl0 = 0;
    ip5561_i2c_read_ctrl(handle, IP55XX_SYS_CTL0, &sys_ctl0);
    status->chg_enabled = (sys_ctl0 & IP55XX_SYS_CTL0_EN_CHARGER) != 0;

    // Read charging state from 0xEA 0xE9 (CHG_STATE2)
    uint8_t chg_state = 0;
    ip5561_i2c_read_stat(handle, IP55XX_CHG_STATE2, &chg_state);
    status->chg_status = chg_state;

    // Parse status bits (from CHG_STATE2)
    // Note: Bit 0 is reserved, NOT vbus_valid (previous code was wrong)
    status->vbus_valid = status->chg_enabled;  // Use charger enable as proxy

    // Parse charge state from bits 6:4
    uint8_t charge_state_val = (chg_state >> 4) & 0x07;
    status->chg_done = (charge_state_val == 0x05);  // 101b = Full charge

    // Bit 3: Charge_en_state - charging enable status flag
    // 1 = charging in progress, 0 = not charging
    status->charging = (chg_state & 0x08) != 0;  // Check Bit 3 only!

    // Diagnostic logging
    ESP_LOGD(TAG, "IP5561: chg_state=0x%02X, chg_enabled=%d, charging=%d, chg_done=%d",
             chg_state, status->chg_enabled, status->charging, status->chg_done);

    return ESP_OK;
}

bool ip5561_is_charging(ip5561_handle_t handle)
{
    ip5561_chg_status_t status;
    if (ip5561_get_charge_status(handle, &status) != ESP_OK) {
        return false;
    }

    return status.charging;
}

bool ip5561_is_charge_complete(ip5561_handle_t handle)
{
    ip5561_chg_status_t status;
    if (ip5561_get_charge_status(handle, &status) != ESP_OK) {
        return false;
    }

    return status.chg_done;
}

bool ip5561_is_vbus_present(ip5561_handle_t handle)
{
    ip5561_chg_status_t status;
    if (ip5561_get_charge_status(handle, &status) != ESP_OK) {
        return false;
    }

    return status.vbus_valid;
}

/* ========== ADC Reading Functions ========== */

uint16_t ip5561_get_battery_voltage(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    // Read 16-bit ADC value from 0xEA 0x50-0x51
    uint8_t vbat_l = 0, vbat_h = 0;
    if (ip5561_i2c_read_stat(handle, IP55XX_VBAT_ADC_L, &vbat_l) != ESP_OK) {
        return 0;
    }
    if (ip5561_i2c_read_stat(handle, IP55XX_VBAT_ADC_H, &vbat_h) != ESP_OK) {
        return 0;
    }

    uint16_t vbat_adc = (vbat_h << 8) | vbat_l;
    /* Convert ADC to voltage using datasheet LSB value */
    return (uint32_t)(vbat_adc * IP55XX_VBAT_ADC_LSB);
}

int16_t ip5561_get_battery_current(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    // Read 16-bit ADC value from 0xEA 0x6E-0x6F
    uint8_t ibat_l = 0, ibat_h = 0;
    if (ip5561_i2c_read_stat(handle, IP55XX_IBAT_ADC_L, &ibat_l) != ESP_OK) {
        return 0;
    }
    if (ip5561_i2c_read_stat(handle, IP55XX_IBAT_ADC_H, &ibat_h) != ESP_OK) {
        return 0;
    }

    uint16_t ibat_adc = (ibat_h << 8) | ibat_l;
    /* Convert ADC to current using datasheet LSB value */
    /* Note: IBAT is signed (positive=charging, negative=discharging) */
    return (int16_t)(ibat_adc * IP55XX_IBAT_ADC_LSB);
}

uint8_t ip5561_get_battery_percent(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    uint8_t soc = 0;
    if (ip5561_i2c_read_stat(handle, IP55XX_SOC_PERCENT, &soc) != ESP_OK) {
        return 0;
    }

    return soc;
}

/* ========== VBUS/VOUT Reading ========== */

uint16_t ip5561_get_vbus_voltage(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    uint8_t vbus_l = 0, vbus_h = 0;
    if (ip5561_i2c_read_stat(handle, IP55XX_VBUS_ADC_L, &vbus_l) != ESP_OK) {
        return 0;
    }
    if (ip5561_i2c_read_stat(handle, IP55XX_VBUS_ADC_H, &vbus_h) != ESP_OK) {
        return 0;
    }

    uint16_t vbus_adc = (vbus_h << 8) | vbus_l;
    /* Convert ADC to voltage using datasheet LSB value */
    return (uint16_t)(vbus_adc * IP55XX_VBUS_ADC_LSB);
}

int16_t ip5561_get_ibus_current(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    // Read 16-bit ADC value from 0xEA 0x54-0x55 (IVBUS - Input Current)
    uint8_t ibus_l = 0, ibus_h = 0;
    if (ip5561_i2c_read_stat(handle, IP55XX_IBUS_IADC_L, &ibus_l) != ESP_OK) {
        return 0;
    }
    if (ip5561_i2c_read_stat(handle, IP55XX_IBUS_IADC_H, &ibus_h) != ESP_OK) {
        return 0;
    }

    /* IBUS ADC is signed 16-bit value */
    int16_t ibus_adc = (int16_t)((ibus_h << 8) | ibus_l);
    /* Convert ADC to current using datasheet LSB value */
    return (int16_t)(ibus_adc * IP55XX_IBUS_ADC_LSB);
}

uint16_t ip5561_get_vout_voltage(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    uint8_t vout_l = 0, vout_h = 0;
    if (ip5561_i2c_read_stat(handle, IP55XX_VOUT_ADC_L, &vout_l) != ESP_OK) {
        return 0;
    }
    if (ip5561_i2c_read_stat(handle, IP55XX_VOUT_ADC_H, &vout_h) != ESP_OK) {
        return 0;
    }

    uint16_t vout_adc = (vout_h << 8) | vout_l;
    /* Convert ADC to voltage using datasheet LSB value */
    return (uint16_t)(vout_adc * IP55XX_VOUT_ADC_LSB);
}

/**
 * @brief Configure NTC1 for temperature reading
 *
 * This function configures the NTC1 thermistor for manual temperature reading
 * according to IP5561 datasheet section 3.6:
 *
 * Step 1: Configure 0xE8 0xFD[0]=0 (disable internal NTC protection)
 * Step 2: Configure 0xE8 0xF6[5:4] (set current source)
 * Step 3: Configure 0xEA 0x19[1]=1 (enable NTC1 current output)
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
esp_err_t ip5561_configure_ntc1(ip5561_handle_t handle, uint8_t current_source)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(current_source <= 3, ESP_ERR_INVALID_ARG, TAG,
                       "Invalid current_source (must be 0, 2, or 3)");

    esp_err_t ret;
    uint8_t reg_value;

    /* ========== Step 1: DISABLE NTC1 to stop auto control ==========
     * We must DISABLE NTC1 first (bit0=0) to stop the internal state machine
     * from auto-controlling the NTC circuit. Only then can we manually configure
     * the current source.
     */
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_NTC_ENABLE, &reg_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read NTC_ENABLE register");

    /* Save original state for logging */
    uint8_t original_fd = reg_value;

    /* Force disable NTC1 (clear bit 0) to stop auto control */
    reg_value &= ~0x01;
    ret = ip5561_i2c_write_ctrl(handle, IP55XX_NTC_ENABLE, reg_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to disable NTC1");

    ESP_LOGD(TAG, "NTC1: Disabled (0xFD: 0x%02X→0x%02X, bit0=0) to stop auto control",
             original_fd, reg_value);

    /* Small delay to ensure disable takes effect */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ========== Step 2: Configure NTC_CTRL (0xE8 0xF6) - Set current source ==========
     * Bits [5:4] (NTC1_20uA_Sel):
     *   00: Internal state machine auto control
     *   01: Internal state machine auto control
     *   10: 80uA current source
     *   11: 20uA current source
     */
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_NTC_CTRL, &reg_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read NTC_CTRL register");

    /* Clear current source bits [5:4] */
    reg_value &= ~(0x03 << 4);
    /* Set new current source value */
    reg_value |= ((current_source & 0x03) << 4);

    ret = ip5561_i2c_write_ctrl(handle, IP55XX_NTC_CTRL, reg_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write NTC_CTRL register");

    const char *current_str;
    switch (current_source) {
        case 0: current_str = "Auto"; break;
        case 2: current_str = "80uA"; break;
        case 3: current_str = "20uA"; break;
        default: current_str = "Unknown"; break;
    }

    ESP_LOGD(TAG, "NTC1: Current source set to %s (0xF6[5:4]=%d)", current_str, current_source);

    /* ========== Step 3: Configure GPIO_20UA_EN (0xEA 0x19) - Enable NTC1 current output ==========
     * Bit 1 (NTC1_20UA_EN): 1 = enable, 0 = disable
     * This is REQUIRED for the current source to actually output current to NTC1 pin
     */
    ret = ip5561_i2c_read_stat(handle, IP55XX_GPIO_20UA_EN, &reg_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read GPIO_20UA_EN register");

    if (!(reg_value & 0x02)) {
        /* NTC1 current output is disabled, enable it */
        reg_value |= 0x02;  /* Set bit 1 */
        ret = ip5561_i2c_write_stat(handle, IP55XX_GPIO_20UA_EN, reg_value);
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write GPIO_20UA_EN register");
        ESP_LOGD(TAG, "NTC1: Current output ENABLED (0x19: 0x%02X, bit1=1)", reg_value);
    } else {
        ESP_LOGD(TAG, "NTC1: Current output already enabled (0x19: 0x%02X)", reg_value);
    }

    /* ========== Step 4: RE-ENABLE NTC1 with manual current source ==========
     * Now we can re-enable NTC1 (bit0=1) with our manually configured current source
     */
    ret = ip5561_i2c_read_ctrl(handle, IP55XX_NTC_ENABLE, &reg_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read NTC_ENABLE register");

    reg_value |= 0x01;  /* Set bit 0 to enable NTC1 */
    ret = ip5561_i2c_write_ctrl(handle, IP55XX_NTC_ENABLE, reg_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to enable NTC1");

    ESP_LOGI(TAG, "NTC1: Configuration complete - %s mode enabled (0xFD: 0x%02X, bit0=1)", current_str, reg_value);

    return ESP_OK;
}

uint16_t ip5561_get_ntc_voltage(ip5561_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    // Read 16-bit ADC value from 0xEA 0x64-0x65
    uint8_t ntc_l = 0, ntc_h = 0;
    if (ip5561_i2c_read_stat(handle, IP55XX_NTC1_ADC_L, &ntc_l) != ESP_OK) {
        return 0;
    }
    if (ip5561_i2c_read_stat(handle, IP55XX_NTC1_ADC_H, &ntc_h) != ESP_OK) {
        return 0;
    }

    uint16_t ntc_adc = (ntc_h << 8) | ntc_l;
    return ntc_adc;  // Returns raw ADC value (scale factor: 0.26855 mV/LSB)
}

esp_err_t ip5561_get_ntc_temperature(ip5561_handle_t handle, int16_t *temp_c, uint16_t *adc_value)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(temp_c, ESP_ERR_INVALID_ARG, TAG, "Invalid temp_c pointer");
    /* adc_value can be NULL */

    esp_err_t ret;
    uint8_t ntc_enable_original;
    uint16_t ntc_adc;

    /* Step 1: Read original NTC_ENABLE (0xFD) register value */
    ret = ip5561_read_reg(handle, IP55XX_NTC_ENABLE, &ntc_enable_original, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NTC_ENABLE register (0xFD)");
        return ret;
    }

    ESP_LOGD(TAG, "NTC temperature read: original 0xFD = 0x%02X", ntc_enable_original);

    /* Step 2: Configure NTC1 for temperature reading (80uA current source)
     * According to IP5561 datasheet, current source selection:
     * 0 = Internal state machine auto control
     * 2 = 80uA
     * 3 = 20uA
     */
    ret = ip5561_configure_ntc1(handle, 2);  /* 80uA current source */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure NTC1");
        return ret;
    }

    /* Small delay to let ADC settle after configuration change */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Step 3: Read NTC ADC value and convert to temperature */
    ntc_adc = ip5561_get_ntc_voltage(handle);
    if (ntc_adc == 0) {
        *temp_c = 0;
        if (adc_value) {
            *adc_value = 0;
        }
        ESP_LOGE(TAG, "Failed to read NTC ADC value");
        /* Still restore original value before returning error */
        ip5561_write_reg(handle, IP55XX_NTC_ENABLE, ntc_enable_original, false);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Store raw ADC value if caller requested it */
    if (adc_value != NULL) {
        *adc_value = ntc_adc;
    }

    /* NTC circuit parameters
     * Circuit: NTC1_PIN -- NTC thermistor (10k@25°C) -- GND
     * Current source: 80uA flows from PIN to GND through NTC
     */
    #define NTC_R25_OHMS         10000   /* 10kΩ @ 25°C */
    #define NTC_BETA_K           3435    /* Beta value (K) */
    #define NTC_T0_K             298.15f /* 25°C in Kelvin */
    #define NTC_CURRENT_SOURCE_UA 80      /* 80uA current source (configured above) */
    #define NTC_ADC_LSB_MV       0.26855 /* mV per LSB */

    /* Convert ADC to voltage */
    float v_ntc_mv = ntc_adc * NTC_ADC_LSB_MV;

    /* Calculate NTC resistance using Ohm's law: R = V / I */
    float r_ntc = v_ntc_mv / NTC_CURRENT_SOURCE_UA;  /* Result in kΩ */
    r_ntc = r_ntc * 1000.0f;  /* kΩ → Ω */

    /* Log raw values for debugging */
    ESP_LOGD(TAG, "NTC: ADC=%u, V=%.2fmV, R=%.1fΩ", ntc_adc, v_ntc_mv, r_ntc);

    /* Check for open circuit (infinite resistance) */
    if (v_ntc_mv > 3000) {
        *temp_c = 125;  /* Return max temperature */
        ESP_LOGW(TAG, "NTC: Open circuit detected (V=%.2fmV)", v_ntc_mv);
        goto restore;
    }

    /* Check for short circuit (very low resistance) */
    if (v_ntc_mv < 10) {
        *temp_c = -40;  /* Return min temperature */
        ESP_LOGW(TAG, "NTC: Short circuit detected (V=%.2fmV)", v_ntc_mv);
        goto restore;
    }

    /* Calculate temperature using Beta formula
     * 1/T = 1/T0 + (1/B) * ln(R/R0)
     * T = 1 / (1/T0 + (1/B) * ln(R/R0))
     */
    float ln_ratio = logf(r_ntc / NTC_R25_OHMS);
    float inv_t = (1.0f / NTC_T0_K) + (1.0f / NTC_BETA_K) * ln_ratio;
    float temp_k = 1.0f / inv_t;

    /* Convert Kelvin to Celsius */
    *temp_c = (int16_t)(temp_k - 273.15f);

    /* Clamp to reasonable range */
    if (*temp_c > 125) *temp_c = 125;
    if (*temp_c < -40) *temp_c = -40;

restore:
    /* Step 4: Restore original NTC_ENABLE register value */
    ret = ip5561_write_reg(handle, IP55XX_NTC_ENABLE, ntc_enable_original, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to restore NTC_ENABLE (0xFD) to 0x%02X", ntc_enable_original);
    } else {
        ESP_LOGD(TAG, "NTC temperature read: restored 0xFD = 0x%02X, temp = %d°C, ADC = %u",
                 ntc_enable_original, *temp_c, ntc_adc);
    }

    return ESP_OK;
}

/* ========== Register Level Access ========== */

esp_err_t ip5561_read_reg(ip5561_handle_t handle, uint8_t reg, uint8_t *value, bool is_stat)
{
    if (is_stat) {
        return ip5561_i2c_read_stat(handle, reg, value);
    } else {
        return ip5561_i2c_read_ctrl(handle, reg, value);
    }
}

esp_err_t ip5561_write_reg(ip5561_handle_t handle, uint8_t reg, uint8_t value, bool is_stat)
{
    if (is_stat) {
        return ip5561_i2c_write_stat(handle, reg, value);
    } else {
        return ip5561_i2c_write_ctrl(handle, reg, value);
    }
}

esp_err_t ip5561_read_regs(ip5561_handle_t handle, uint8_t reg, uint8_t *data, size_t len, bool is_stat)
{
    if (is_stat) {
        return ip5561_i2c_read_stat_bytes(handle, reg, data, len);
    } else {
        return ip5561_i2c_read_ctrl_bytes(handle, reg, data, len);
    }
}

esp_err_t ip5561_write_regs(ip5561_handle_t handle, uint8_t reg, const uint8_t *data, size_t len, bool is_stat)
{
    if (is_stat) {
        return ip5561_i2c_write_stat_bytes(handle, reg, data, len);
    } else {
        return ip5561_i2c_write_ctrl_bytes(handle, reg, data, len);
    }
}
