/*
 * sc7a20h.c - SC7A20H 3-Axis Accelerometer Driver
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "sc7a20h.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SC7A20H";

/* Internal data structure */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} sc7a20h_data_t;

/* I2C read register with address auto-increment */
static esp_err_t sc7a20h_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, size_t len, uint8_t *data)
{
    /* Set bit[7]=1 for multi-byte address auto-increment */
    uint8_t reg_addr = (len > 1) ? (reg | 0x80) : reg;
    return i2c_master_transmit_receive(dev, &reg_addr, 1, data, len, pdMS_TO_TICKS(200));
}

/* I2C write register */
static esp_err_t sc7a20h_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

sc7a20h_handle_t sc7a20h_create(const sc7a20h_config_t *config)
{
    if (config == NULL || config->i2c_bus == NULL) {
        ESP_LOGE(TAG, "Invalid config");
        return NULL;
    }

    sc7a20h_data_t *data = calloc(1, sizeof(sc7a20h_data_t));
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return NULL;
    }

    /* Add device to I2C bus */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SC7A20H_I2C_ADDRESS,
        .scl_speed_hz = config->scl_freq_hz > 0 ? config->scl_freq_hz : SC7A20H_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus, &dev_config, &data->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        free(data);
        return NULL;
    }

    /* Read and verify WHO_AM_I */
    uint8_t chipid = 0;
    ret = sc7a20h_read_reg(data->i2c_dev, SC7A20H_REG_WHO_AM_I, 1, &chipid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(data->i2c_dev);
        free(data);
        return NULL;
    }

    if (chipid != SC7A20H_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I: 0x%02X (expected 0x%02X)", chipid, SC7A20H_WHO_AM_I_VALUE);
        i2c_master_bus_rm_device(data->i2c_dev);
        free(data);
        return NULL;
    }

    ESP_LOGI(TAG, "SC7A20H detected (WHO_AM_I=0x%02X)", chipid);

    /* Configure: CTRL_REG0 = 0x00 (no oversampling, no DLPF) */
    sc7a20h_write_reg(data->i2c_dev, SC7A20H_REG_CTRL_REG0, 0x00);

    /* Configure: CTRL_REG1 = 0x47 (ODR=25Hz, normal mode, XYZ enabled)
     * Bit[7:4] = 0100 -> ODR = 25Hz
     * Bit[3]   = 0    -> Normal mode
     * Bit[2:0] = 111  -> Z/Y/X axes enabled */
    sc7a20h_write_reg(data->i2c_dev, SC7A20H_REG_CTRL_REG1, 0x47);

    /* Configure: CTRL_REG4 = 0x80 (BDU=1, FS=±2g, little-endian)
     * Bit[7]   = 1    -> Block data update
     * Bit[6]   = 0    -> Little-endian
     * Bit[5:4] = 00   -> ±2g (1024 counts/g)
     * Bit[3]   = 0    -> Normal mode */
    sc7a20h_write_reg(data->i2c_dev, SC7A20H_REG_CTRL_REG4, 0x80);

    return (sc7a20h_handle_t)data;
}

esp_err_t sc7a20h_delete(sc7a20h_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sc7a20h_data_t *data = (sc7a20h_data_t *)handle;
    i2c_master_bus_rm_device(data->i2c_dev);
    free(data);
    return ESP_OK;
}

esp_err_t sc7a20h_read_chipid(sc7a20h_handle_t handle, uint8_t *chipid)
{
    if (handle == NULL || chipid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    sc7a20h_data_t *data = (sc7a20h_data_t *)handle;
    return sc7a20h_read_reg(data->i2c_dev, SC7A20H_REG_WHO_AM_I, 1, chipid);
}

esp_err_t sc7a20h_read_xyz(sc7a20h_handle_t handle, int16_t *x_mg, int16_t *y_mg, int16_t *z_mg)
{
    if (handle == NULL || x_mg == NULL || y_mg == NULL || z_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sc7a20h_data_t *data = (sc7a20h_data_t *)handle;
    uint8_t buf[6];

    /* Read 6 bytes starting from OUT_X_L (0x28) with auto-increment */
    esp_err_t ret = sc7a20h_read_reg(data->i2c_dev, SC7A20H_REG_OUT_X_L, 6, buf);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Combine: 12-bit signed, left-aligned (same as DA228EC)
     * Raw 16-bit value has 12-bit data in bits[15:4], need >>4 to extract */
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]) >> 4;
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]) >> 4;
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]) >> 4;

    /* Convert to mg: ±2g sensitivity = 1024 counts/g, so mg = raw * 1000 / 1024 */
    *x_mg = (int16_t)((int32_t)raw_x * 1000 / 1024);
    *y_mg = (int16_t)((int32_t)raw_y * 1000 / 1024);
    *z_mg = (int16_t)((int32_t)raw_z * 1000 / 1024);

    return ESP_OK;
}

esp_err_t sc7a20h_read_regs(sc7a20h_handle_t handle, uint8_t reg, size_t len, uint8_t *data_out)
{
    if (handle == NULL || data_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    sc7a20h_data_t *data = (sc7a20h_data_t *)handle;
    return sc7a20h_read_reg(data->i2c_dev, reg, len, data_out);
}
