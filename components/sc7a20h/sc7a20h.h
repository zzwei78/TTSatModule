/*
 * sc7a20h.h - SC7A20H 3-Axis Accelerometer Driver
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef SC7A20H_H
#define SC7A20H_H

#include "esp_err.h"
#include "driver/i2c_master.h"

/* I2C 7-bit address (SA0=1) */
#define SC7A20H_I2C_ADDRESS         0x19

/* Chip ID (WHO_AM_I register value) */
#define SC7A20H_WHO_AM_I_VALUE      0x11

/* Register addresses */
#define SC7A20H_REG_WHO_AM_I        0x0F
#define SC7A20H_REG_CTRL_REG0       0x1F
#define SC7A20H_REG_CTRL_REG1       0x20
#define SC7A20H_REG_CTRL_REG2       0x21
#define SC7A20H_REG_CTRL_REG4       0x23
#define SC7A20H_REG_STATUS          0x27
#define SC7A20H_REG_OUT_X_L         0x28
#define SC7A20H_REG_OUT_X_H         0x29
#define SC7A20H_REG_OUT_Y_L         0x2A
#define SC7A20H_REG_OUT_Y_H         0x2B
#define SC7A20H_REG_OUT_Z_L         0x2C
#define SC7A20H_REG_OUT_Z_H         0x2D

/* Default I2C frequency */
#define SC7A20H_I2C_FREQ_HZ         200000

/* Configuration structure */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    uint32_t scl_freq_hz;
} sc7a20h_config_t;

/* Opaque handle */
typedef void *sc7a20h_handle_t;

/**
 * @brief Create SC7A20H device handle and initialize sensor
 *
 * @param config Configuration (I2C bus handle, frequency)
 * @return Handle on success, NULL on failure
 */
sc7a20h_handle_t sc7a20h_create(const sc7a20h_config_t *config);

/**
 * @brief Delete SC7A20H device handle
 */
esp_err_t sc7a20h_delete(sc7a20h_handle_t handle);

/**
 * @brief Read chip ID (WHO_AM_I)
 *
 * @param handle Device handle
 * @param chipid Output: chip ID (expected 0x11)
 * @return ESP_OK on success
 */
esp_err_t sc7a20h_read_chipid(sc7a20h_handle_t handle, uint8_t *chipid);

/**
 * @brief Read acceleration data (unit: mg)
 *
 * @param handle Device handle
 * @param x_mg Output: X-axis acceleration in mg
 * @param y_mg Output: Y-axis acceleration in mg
 * @param z_mg Output: Z-axis acceleration in mg
 * @return ESP_OK on success
 */
esp_err_t sc7a20h_read_xyz(sc7a20h_handle_t handle, int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);

/**
 * @brief Read registers (for debugging)
 *
 * @param handle Device handle
 * @param reg Start register address
 * @param len Number of bytes to read
 * @param data Output buffer
 * @return ESP_OK on success
 */
esp_err_t sc7a20h_read_regs(sc7a20h_handle_t handle, uint8_t reg, size_t len, uint8_t *data);

#endif /* SC7A20H_H */
