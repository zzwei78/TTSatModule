/*
 * ota_partition.h - OTA Partition Operations
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef OTA_PARTITION_H
#define OTA_PARTITION_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_partition.h"

/* OTA Partition Types */
typedef enum {
    OTA_PARTITION_MCU = 0,    // MCU firmware partition
    OTA_PARTITION_TT = 1       // Tiantong module firmware partition
} ota_partition_type_t;

/* OTA Partition Names */
#define OTA_PARTITION_NAME_MCU    "OTA_MCU"
#define OTA_PARTITION_NAME_TT      "OTA_TT"

/* OTA Status */
typedef enum {
    OTA_PARTITION_STATUS_IDLE = 0,
    OTA_PARTITION_STATUS_WRITING,
    OTA_PARTITION_STATUS_VERIFYING,
    OTA_PARTITION_STATUS_SUCCESS,
    OTA_PARTITION_STATUS_FAILED
} ota_status_t;

/* OTA Context */
typedef struct {
    ota_partition_type_t type;
    const esp_partition_t *partition;
    size_t total_size;
    size_t written_size;
    uint32_t crc32;
    ota_status_t status;
    bool initialized;
} ota_context_t;

/**
 * @brief Initialize OTA partition for writing
 *
 * @param type Partition type (MCU or TT)
 * @param total_size Total size of firmware to be written
 * @param expected_crc32 Expected CRC32 checksum (0 to skip verification)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ota_partition_init(ota_partition_type_t type, size_t total_size, uint32_t expected_crc32);

/**
 * @brief Write data to OTA partition
 *
 * @param data Data buffer to write
 * @param len Data length
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ota_partition_write(const uint8_t *data, size_t len);

/**
 * @brief Finalize OTA partition and verify integrity
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ota_partition_finalize(void);

/**
 * @brief Get current OTA status
 *
 * @return ota_status_t Current status
 */
ota_status_t ota_partition_get_status(void);

/**
 * @brief Get written size
 *
 * @return size_t Written size in bytes
 */
size_t ota_partition_get_written_size(void);

/**
 * @brief Get total size
 *
 * @return size_t Total size in bytes
 */
size_t ota_partition_get_total_size(void);

/**
 * @brief Abort OTA operation
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ota_partition_abort(void);

/**
 * @brief Calculate CRC32 of data
 *
 * @param data Data buffer
 * @param len Data length
 * @return uint32_t CRC32 checksum
 */
uint32_t ota_partition_calculate_crc32(const uint8_t *data, size_t len);

/**
 * @brief Get current OTA partition type
 *
 * @return ota_partition_type_t Current partition type
 */
ota_partition_type_t ota_partition_get_type(void);

/**
 * @brief Get OTA partition name
 *
 * @return const char* Partition name
 */
const char* ota_partition_get_name(void);

#endif /* OTA_PARTITION_H */
