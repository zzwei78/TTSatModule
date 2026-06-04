/*
 * mmc5603.h - MMC5603NJ 3-Axis Magnetometer Driver
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef MMC5603_H
#define MMC5603_H

#include "esp_err.h"
#include "driver/i2c_master.h"

/* I2C 7-bit address */
#define MMC5603_I2C_ADDRESS         0x30

/* Product ID */
#define MMC5603_PRODUCT_ID          0x10

/* Register addresses */
#define MMC5603_REG_XOUT0           0x00
#define MMC5603_REG_XOUT1           0x01
#define MMC5603_REG_YOUT0           0x02
#define MMC5603_REG_YOUT1           0x03
#define MMC5603_REG_ZOUT0           0x04
#define MMC5603_REG_ZOUT1           0x05
#define MMC5603_REG_XOUT2           0x06
#define MMC5603_REG_YOUT2           0x07
#define MMC5603_REG_ZOUT2           0x08
#define MMC5603_REG_TOUT            0x09
#define MMC5603_REG_STATUS1         0x18
#define MMC5603_REG_ODR             0x1A
#define MMC5603_REG_IC0             0x1B    /* Internal Control 0 */
#define MMC5603_REG_IC1             0x1C    /* Internal Control 1 */
#define MMC5603_REG_IC2             0x1D    /* Internal Control 2 */
#define MMC5603_REG_PRODUCT_ID      0x39

/* Internal Control 0 bits */
#define MMC5603_IC0_TAKE_MEAS_M     (1 << 0)
#define MMC5603_IC0_TAKE_MEAS_T     (1 << 1)
#define MMC5603_IC0_DO_SET          (1 << 3)
#define MMC5603_IC0_DO_RESET        (1 << 4)
#define MMC5603_IC0_AUTO_SR_EN      (1 << 5)

/* Internal Control 1 bits */
#define MMC5603_IC1_BW_MASK         0x03
#define MMC5603_IC1_SW_RESET        (1 << 7)

/* Status1 bits */
#define MMC5603_STATUS1_MEAS_M_DONE (1 << 6)
#define MMC5603_STATUS1_MEAS_T_DONE (1 << 7)

/* 16-bit mode constants */
#define MMC5603_ZERO_FIELD_16BIT    32768
#define MMC5603_SENSITIVITY_16BIT   1024    /* counts/Gauss */

/* Default I2C frequency */
#define MMC5603_I2C_FREQ_HZ         200000

/* Configuration structure */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    uint32_t scl_freq_hz;
} mmc5603_config_t;

/* Opaque handle */
typedef void *mmc5603_handle_t;

/**
 * @brief Create MMC5603 device handle and initialize sensor
 */
mmc5603_handle_t mmc5603_create(const mmc5603_config_t *config);

/**
 * @brief Delete MMC5603 device handle
 */
esp_err_t mmc5603_delete(mmc5603_handle_t handle);

/**
 * @brief Read product ID
 */
esp_err_t mmc5603_read_chipid(mmc5603_handle_t handle, uint8_t *chipid);

/**
 * @brief Read magnetic field data (unit: mG)
 *
 * Triggers single measurement, waits for completion, returns X/Y/Z in mG
 */
esp_err_t mmc5603_read_xyz(mmc5603_handle_t handle, int32_t *x_mg, int32_t *y_mg, int32_t *z_mg);

/**
 * @brief Read registers (for debugging)
 */
esp_err_t mmc5603_read_regs(mmc5603_handle_t handle, uint8_t reg, size_t len, uint8_t *data);

#endif /* MMC5603_H */
