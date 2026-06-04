/*
 * da228ec.h - DA228EC 3-Axis Accelerometer Driver
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef DA228EC_H
#define DA228EC_H

#include "esp_err.h"
#include "driver/i2c_master.h"

/* I2C 7-bit address */
#define DA228EC_I2C_ADDRESS         0x27

/* Chip ID */
#define DA228EC_CHIP_ID             0x13

/* Register addresses */
#define DA228EC_REG_CONFIG          0x00
#define DA228EC_REG_CHIPID          0x01
#define DA228EC_REG_ACC_X_LSB       0x02
#define DA228EC_REG_ACC_X_MSB       0x03
#define DA228EC_REG_ACC_Y_LSB       0x04
#define DA228EC_REG_ACC_Y_MSB       0x05
#define DA228EC_REG_ACC_Z_LSB       0x06
#define DA228EC_REG_ACC_Z_MSB       0x07
#define DA228EC_REG_MOTION_FLAG     0x09
#define DA228EC_REG_NEWDATA_FLAG    0x0A
#define DA228EC_REG_ACTIVE_STATUS   0x0B
#define DA228EC_REG_RANGE           0x0F
#define DA228EC_REG_ODR_AXIS        0x10
#define DA228EC_REG_MODE_BW         0x11

/* Range settings */
#define DA228EC_RANGE_2G            0x00    /* Sensitivity: 1024 LSB/g */
#define DA228EC_RANGE_4G            0x01    /* Sensitivity: 512 LSB/g */
#define DA228EC_RANGE_8G            0x02    /* Sensitivity: 256 LSB/g */

/* CONFIG register bits */
#define DA228EC_CONFIG_ACTIVE       0x01

/* Default I2C frequency */
#define DA228EC_I2C_FREQ_HZ         200000

/* Configuration structure */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    uint32_t scl_freq_hz;
} da228ec_config_t;

/* Opaque handle */
typedef void *da228ec_handle_t;

/**
 * @brief Create DA228EC device handle and initialize sensor
 *
 * @param config Configuration (I2C bus handle, frequency)
 * @return Handle on success, NULL on failure
 */
da228ec_handle_t da228ec_create(const da228ec_config_t *config);

/**
 * @brief Delete DA228EC device handle
 */
esp_err_t da228ec_delete(da228ec_handle_t handle);

/**
 * @brief Read chip ID
 *
 * @param handle Device handle
 * @param chipid Output: chip ID (expected 0x13)
 * @return ESP_OK on success
 */
esp_err_t da228ec_read_chipid(da228ec_handle_t handle, uint8_t *chipid);

/**
 * @brief Read acceleration data (unit: mg)
 *
 * @param handle Device handle
 * @param x_mg Output: X-axis acceleration in mg
 * @param y_mg Output: Y-axis acceleration in mg
 * @param z_mg Output: Z-axis acceleration in mg
 * @return ESP_OK on success
 */
esp_err_t da228ec_read_xyz(da228ec_handle_t handle, int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);

/**
 * @brief Read registers (for debugging)
 *
 * @param handle Device handle
 * @param reg Start register address
 * @param len Number of bytes to read
 * @param data Output buffer
 * @return ESP_OK on success
 */
esp_err_t da228ec_read_regs(da228ec_handle_t handle, uint8_t reg, size_t len, uint8_t *data);

#endif /* DA228EC_H */
