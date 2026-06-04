/*
 * da228ec.c - DA228EC 3-Axis Accelerometer Driver
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "da228ec.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "DA228EC";

/* Internal data structure */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} da228ec_data_t;

/* I2C read register */
static esp_err_t da228ec_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, size_t len, uint8_t *data)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, -1);
}

/* I2C write register */
static esp_err_t da228ec_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), -1);
}

da228ec_handle_t da228ec_create(const da228ec_config_t *config)
{
    if (config == NULL || config->i2c_bus == NULL) {
        ESP_LOGE(TAG, "Invalid config");
        return NULL;
    }

    da228ec_data_t *data = calloc(1, sizeof(da228ec_data_t));
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return NULL;
    }

    /* Add device to I2C bus */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DA228EC_I2C_ADDRESS,
        .scl_speed_hz = config->scl_freq_hz > 0 ? config->scl_freq_hz : DA228EC_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus, &dev_config, &data->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        free(data);
        return NULL;
    }

    /* Read and verify Chip ID */
    uint8_t chipid = 0;
    ret = da228ec_read_reg(data->i2c_dev, DA228EC_REG_CHIPID, 1, &chipid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIPID: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(data->i2c_dev);
        free(data);
        return NULL;
    }

    if (chipid != DA228EC_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid CHIPID: 0x%02X (expected 0x%02X)", chipid, DA228EC_CHIP_ID);
        i2c_master_bus_rm_device(data->i2c_dev);
        free(data);
        return NULL;
    }

    ESP_LOGI(TAG, "DA228EC detected (CHIPID=0x%02X)", chipid);

    /* Configure: ±2g, 3.9Hz, normal mode */
    da228ec_write_reg(data->i2c_dev, DA228EC_REG_RANGE, 0x00);
    da228ec_write_reg(data->i2c_dev, DA228EC_REG_ODR_AXIS, 0x02);
    da228ec_write_reg(data->i2c_dev, DA228EC_REG_MODE_BW, 0x1E);

    return (da228ec_handle_t)data;
}

esp_err_t da228ec_delete(da228ec_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    da228ec_data_t *data = (da228ec_data_t *)handle;
    i2c_master_bus_rm_device(data->i2c_dev);
    free(data);
    return ESP_OK;
}

esp_err_t da228ec_read_chipid(da228ec_handle_t handle, uint8_t *chipid)
{
    if (handle == NULL || chipid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    da228ec_data_t *data = (da228ec_data_t *)handle;
    return da228ec_read_reg(data->i2c_dev, DA228EC_REG_CHIPID, 1, chipid);
}

esp_err_t da228ec_read_xyz(da228ec_handle_t handle, int16_t *x_mg, int16_t *y_mg, int16_t *z_mg)
{
    if (handle == NULL || x_mg == NULL || y_mg == NULL || z_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    da228ec_data_t *data = (da228ec_data_t *)handle;
    uint8_t buf[6];

    /* Read 6 bytes starting from ACC_X_LSB (0x02) */
    esp_err_t ret = da228ec_read_reg(data->i2c_dev, DA228EC_REG_ACC_X_LSB, 6, buf);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Combine: 12-bit signed, left-aligned
     * raw = (MSB << 4) | (LSB >> 4), then sign-extend from 12 bits */
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]) >> 4;
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]) >> 4;
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]) >> 4;

    /* Convert to mg: ±2g sensitivity = 1024 LSB/g, so mg = raw * 1000 / 1024 */
    *x_mg = (int16_t)((int32_t)raw_x * 1000 / 1024);
    *y_mg = (int16_t)((int32_t)raw_y * 1000 / 1024);
    *z_mg = (int16_t)((int32_t)raw_z * 1000 / 1024);

    return ESP_OK;
}

esp_err_t da228ec_read_regs(da228ec_handle_t handle, uint8_t reg, size_t len, uint8_t *data)
{
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    da228ec_data_t *d = (da228ec_data_t *)handle;
    return da228ec_read_reg(d->i2c_dev, reg, len, data);
}
