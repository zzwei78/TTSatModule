/*
 * version.h - Firmware Version Information
 *
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Firmware Version Information */
#define FIRMWARE_VERSION_MAJOR     1
#define FIRMWARE_VERSION_MINOR     0
#define FIRMWARE_VERSION_PATCH     0

/* Build Information */
#define FIRMWARE_BUILD_DATE        __DATE__
#define FIRMWARE_BUILD_TIME        __TIME__

/* Version String */
#define FIRMWARE_VERSION_STRING    "1.1.6"

/* Product Information */
#define PRODUCT_MANUFACTURER       "TTCat"
#define PRODUCT_MODEL_NUMBER       "TTSat-1000"
#define PRODUCT_HARDWARE_REVISION  "1.0"
#define PRODUCT_SERIAL_NUMBER      NULL  /* Set dynamically from device ID */

/* Software Information */
#define SOFTWARE_VERSION_STRING    "TTSat-" FIRMWARE_VERSION_STRING

/**
 * @brief Get firmware version string
 *
 * @return Firmware version string (e.g., "1.0.0")
 */
const char* get_firmware_version(void);

/**
 * @brief Get software version string
 *
 * @return Software version string (e.g., "TTSat-1.0.0")
 */
const char* get_software_version(void);

/**
 * @brief Get manufacturer name
 *
 * @return Manufacturer name
 */
const char* get_manufacturer_name(void);

/**
 * @brief Get model number
 *
 * @return Model number
 */
const char* get_model_number(void);

/**
 * @brief Get hardware revision
 *
 * @return Hardware revision string
 */
const char* get_hardware_revision(void);

/**
 * @brief Get build date/time string
 *
 * @return Build date and time
 */
const char* get_build_datetime(void);

#ifdef __cplusplus
}
#endif
