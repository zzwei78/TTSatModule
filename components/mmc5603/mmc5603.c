/*
 * mmc5603.c - MMC5603NJ 3-Axis Magnetometer Driver
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "mmc5603.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

static const char *TAG = "MMC5603";

/* Internal data structure */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} mmc5603_data_t;

/* I2C read register */
static esp_err_t mmc5603_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, size_t len, uint8_t *data)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, pdMS_TO_TICKS(200));
}

/* I2C write register */
static esp_err_t mmc5603_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

mmc5603_handle_t mmc5603_create(const mmc5603_config_t *config)
{
    if (config == NULL || config->i2c_bus == NULL) {
        ESP_LOGE(TAG, "Invalid config");
        return NULL;
    }

    mmc5603_data_t *data = calloc(1, sizeof(mmc5603_data_t));
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return NULL;
    }

    /* Add device to I2C bus */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MMC5603_I2C_ADDRESS,
        .scl_speed_hz = config->scl_freq_hz > 0 ? config->scl_freq_hz : MMC5603_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus, &dev_config, &data->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        free(data);
        return NULL;
    }

    /* Software reset */
    ret = mmc5603_write_reg(data->i2c_dev, MMC5603_REG_IC1, MMC5603_IC1_SW_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to software reset: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(data->i2c_dev);
        free(data);
        return NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Read and verify Product ID */
    uint8_t product_id = 0;
    ret = mmc5603_read_reg(data->i2c_dev, MMC5603_REG_PRODUCT_ID, 1, &product_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Product ID: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(data->i2c_dev);
        free(data);
        return NULL;
    }

    if (product_id != MMC5603_PRODUCT_ID) {
        ESP_LOGE(TAG, "Invalid Product ID: 0x%02X (expected 0x%02X)", product_id, MMC5603_PRODUCT_ID);
        i2c_master_bus_rm_device(data->i2c_dev);
        free(data);
        return NULL;
    }

    ESP_LOGI(TAG, "MMC5603NJ detected (ProductID=0x%02X)", product_id);

    /* Enable Auto SET/RESET (recommended to always enable) */
    mmc5603_write_reg(data->i2c_dev, MMC5603_REG_IC0, MMC5603_IC0_AUTO_SR_EN);

    return (mmc5603_handle_t)data;
}

esp_err_t mmc5603_delete(mmc5603_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mmc5603_data_t *data = (mmc5603_data_t *)handle;
    i2c_master_bus_rm_device(data->i2c_dev);
    free(data);
    return ESP_OK;
}

esp_err_t mmc5603_read_chipid(mmc5603_handle_t handle, uint8_t *chipid)
{
    if (handle == NULL || chipid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mmc5603_data_t *data = (mmc5603_data_t *)handle;
    return mmc5603_read_reg(data->i2c_dev, MMC5603_REG_PRODUCT_ID, 1, chipid);
}

esp_err_t mmc5603_read_xyz(mmc5603_handle_t handle, int32_t *x_mg, int32_t *y_mg, int32_t *z_mg)
{
    if (handle == NULL || x_mg == NULL || y_mg == NULL || z_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mmc5603_data_t *data = (mmc5603_data_t *)handle;

    /* Trigger single magnetic measurement with Auto SR */
    esp_err_t ret = mmc5603_write_reg(data->i2c_dev, MMC5603_REG_IC0,
                                       MMC5603_IC0_TAKE_MEAS_M | MMC5603_IC0_AUTO_SR_EN);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Wait for measurement complete (BW=00 → 6.6ms + margin) */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Read 6 bytes: Xout0, Xout1, Yout0, Yout1, Zout0, Zout1 (16-bit mode) */
    uint8_t buf[6];
    ret = mmc5603_read_reg(data->i2c_dev, MMC5603_REG_XOUT0, 6, buf);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 16-bit combination: Xout[19:4] */
    uint16_t raw_x = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_y = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t raw_z = ((uint16_t)buf[4] << 8) | buf[5];

    /* Convert to mG: (raw - 32768) * 1000 / 1024 */
    *x_mg = ((int32_t)raw_x - MMC5603_ZERO_FIELD_16BIT) * 1000 / MMC5603_SENSITIVITY_16BIT;
    *y_mg = ((int32_t)raw_y - MMC5603_ZERO_FIELD_16BIT) * 1000 / MMC5603_SENSITIVITY_16BIT;
    *z_mg = ((int32_t)raw_z - MMC5603_ZERO_FIELD_16BIT) * 1000 / MMC5603_SENSITIVITY_16BIT;

    return ESP_OK;
}

esp_err_t mmc5603_read_regs(mmc5603_handle_t handle, uint8_t reg, size_t len, uint8_t *data)
{
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mmc5603_data_t *d = (mmc5603_data_t *)handle;
    return mmc5603_read_reg(d->i2c_dev, reg, len, data);
}
