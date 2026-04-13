/*
 * gatt_ota_server.c - GATT OTA Server Implementation
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <endian.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble/gatt_ota_server.h"
#include "system/ota_partition.h"
#include "system/syslog.h"
#include "tt/tt_module_ota.h"
#include "tt/tt_module.h"

/* Tag for logging */
static const char *TAG = "GATT_OTA";

/* MTU size settings */
#define OTA_GATT_MTU_SIZE         512   // MTU size during OTA
#define NORMAL_GATT_MTU_SIZE       512   // Normal MTU size (same as OTA - no MTU switching needed)

/* OTA timeout settings */
#define OTA_TIMEOUT_MS            8000  // OTA data reception timeout in milliseconds

/* OTA server initialization flag */
static bool g_ota_server_initialized = false;

/* Service enabled flag */
static bool g_ota_service_enabled = false;

/* Service registration flag */
static bool g_ota_service_registered = false;

/* OTA in progress flag */
static bool g_ota_in_progress = false;

/* OTA timeout timer */
static esp_timer_handle_t g_ota_timeout_timer = NULL;

/* Current connection handle for timeout callback */
static uint16_t g_ota_conn_handle = 0;

/* OTA data packet sequence tracking */
static uint16_t g_ota_expected_seq = 0;  // Expected sequence number
static bool g_ota_first_packet = true;   // First packet flag
static uint8_t g_ota_last_notified_progress = 0;  // Last notified progress (to reduce notify frequency)

/* OTA Service characteristic handles */
static uint16_t ota_control_val_handle = 0;
static uint16_t ota_data_val_handle = 0;
static uint16_t ota_status_val_handle = 0;

/* Maximum data size for OTA operations */
#define GATT_OTA_MAX_DATA_SIZE 512  // Buffer size (can be larger than MTU)

/* OTA Control Packet Structure */
typedef struct {
    uint8_t cmd;           // Command byte
    uint8_t reserved[3];   // Reserved bytes
    uint32_t total_size;   // Total firmware size (little-endian)
    uint32_t crc32;        // Expected CRC32 checksum (little-endian)
} __attribute__((packed)) ota_control_packet_t;

/* Forward declarations */
static int ota_service_handler(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static esp_err_t handle_ota_control_command(uint16_t conn_handle, const uint8_t *data, size_t len);
static esp_err_t handle_ota_data_write(uint16_t conn_handle, const uint8_t *data, size_t len);
static void ota_timeout_callback(void* arg);
static void ota_timeout_start(void);
static void ota_timeout_stop(void);
static uint16_t crc16_modbus(const uint8_t *data, size_t len);

/**
 * @brief Set MTU size for OTA
 */
static void set_ota_mtu(void)
{
    int rc = ble_att_set_preferred_mtu(OTA_GATT_MTU_SIZE);
    if (rc != 0) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to set OTA MTU to %d: rc=%d", OTA_GATT_MTU_SIZE, rc);
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "MTU set to %d for OTA", OTA_GATT_MTU_SIZE);
    }
}

/**
 * @brief Restore normal MTU size
 *
 * Note: Only sets preferred MTU, does not initiate exchange.
 * Client should initiate MTU exchange when needed.
 */
static void restore_normal_mtu(uint16_t conn_handle)
{
    // Set preferred MTU back to normal
    int rc = ble_att_set_preferred_mtu(NORMAL_GATT_MTU_SIZE);
    if (rc != 0) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to set preferred MTU to %d: rc=%d", NORMAL_GATT_MTU_SIZE, rc);
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Preferred MTU set to %d (client should initiate exchange)", NORMAL_GATT_MTU_SIZE);
    }
}

/**
 * @brief Clean up OTA state
 *
 * This function resets all OTA-related state variables and stops timers.
 * Call this when OTA completes, aborts, or encounters an error.
 */
static void ota_cleanup(void)
{
    // Save connection handle before cleanup
    uint16_t conn_handle = g_ota_conn_handle;

    // Abort OTA partition operation first
    ota_partition_abort();

    g_ota_in_progress = false;
    g_ota_conn_handle = 0;
    g_ota_first_packet = true;
    g_ota_expected_seq = 0;
    g_ota_last_notified_progress = 0;  // Reset progress tracking
    ota_timeout_stop();

    // Restore MTU with saved connection handle
    restore_normal_mtu(conn_handle);
}

/**
 * @brief Calculate CRC-16/MODBUS checksum
 *
 * CRC-16/MODBUS parameters:
 * - Polynomial: 0x8005
 * - Initial value: 0xFFFF
 * - Reverse input: Yes
 * - Reverse output: Yes
 * - Final XOR: 0x0000
 *
 * @param data Data buffer
 * @param len Data length
 * @return CRC16 checksum (little-endian)
 */
static uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;  // Initial value

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // Polynomial reversed
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief OTA timeout callback - triggered when no data received for timeout period
 */
static void ota_timeout_callback(void* arg)
{
    SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA timeout triggered, aborting OTA...");

    // Check if OTA is still in progress
    if (!g_ota_in_progress) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA not in progress, ignoring timeout");
        return;
    }

    // Save connection handle before cleanup
    uint16_t conn_handle = g_ota_conn_handle;

    // Clean up OTA state (includes partition abort)
    ota_cleanup();

    // Send failed status notification
    if (conn_handle != 0) {
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA aborted due to timeout");
}

/**
 * @brief Start OTA timeout timer
 */
static void ota_timeout_start(void)
{
    esp_err_t ret;

    if (g_ota_timeout_timer == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA timeout timer not initialized");
        return;
    }

    // Stop any existing timer
    ota_timeout_stop();

    // Start one-shot timer
    ret = esp_timer_start_once(g_ota_timeout_timer, OTA_TIMEOUT_MS * 1000);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to start OTA timeout timer: %s", esp_err_to_name(ret));
    } else {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA timeout timer started (%d ms)", OTA_TIMEOUT_MS);
    }
}

/**
 * @brief Stop OTA timeout timer
 */
static void ota_timeout_stop(void)
{
    esp_err_t ret;

    if (g_ota_timeout_timer == NULL) {
        return;
    }

    ret = esp_timer_stop(g_ota_timeout_timer);
    if (ret == ESP_OK) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA timeout timer stopped");
    } else if (ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means timer was not running, which is fine
        SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to stop OTA timeout timer: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Handle OTA control command
 */
static esp_err_t handle_ota_control_command(uint16_t conn_handle, const uint8_t *data, size_t len)
{
    if (data == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Invalid command data");
        return ESP_ERR_INVALID_ARG;
    }

    if (len < sizeof(ota_control_packet_t)) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Invalid control packet size: %d", len);
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
        return ESP_ERR_INVALID_ARG;
    }

    ota_control_packet_t *packet = (ota_control_packet_t *)data;
    uint8_t response = OTA_RESP_ERROR;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA Control: cmd=0x%02x, size=%d, crc32=0x%08x",
             packet->cmd, packet->total_size, packet->crc32);

    switch (packet->cmd) {
    case OTA_CMD_START_MCU:
    case OTA_CMD_START_TT:
        // Check if OTA is already in progress
        if (g_ota_in_progress) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA already in progress");
            response = OTA_RESP_INVALID_STATE;
            gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
            break;
        }

        // Set preferred MTU (client should initiate exchange before OTA)
        set_ota_mtu();

        ota_partition_type_t partition_type;
        if (packet->cmd == OTA_CMD_START_MCU) {
            partition_type = OTA_PARTITION_MCU;
        } else {
            partition_type = OTA_PARTITION_TT;
        }

        esp_err_t ret = ota_partition_init(partition_type, packet->total_size, packet->crc32);
        if (ret == ESP_OK) {
            g_ota_in_progress = true;
            g_ota_conn_handle = conn_handle;
            g_ota_first_packet = true;
            g_ota_expected_seq = 0;
            g_ota_last_notified_progress = 0;  // Reset progress tracking for new OTA

            if (partition_type == OTA_PARTITION_MCU) {
                ota_timeout_start();
                SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "MCU OTA started: size=%d, crc32=0x%08x",
                         packet->total_size, packet->crc32);
            } else {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT OTA started: size=%d, crc32=0x%08x",
                         packet->total_size, packet->crc32);
            }

            response = OTA_RESP_OK;
            gatt_ota_server_send_status(conn_handle, OTA_STATUS_WRITING, 0);
        } else {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to start OTA: %s", esp_err_to_name(ret));
            response = OTA_RESP_ERROR;
            gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
        }
        break;

    case OTA_CMD_ABORT:
        {
            ota_cleanup();
            response = OTA_RESP_OK;
            SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA aborted");
            gatt_ota_server_send_status(conn_handle, OTA_STATUS_IDLE, 0);
        }
        break;

    default:
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Invalid OTA command: 0x%02x", packet->cmd);
        response = OTA_RESP_INVALID_CMD;
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
        break;
    }

    // Send response back via control characteristic
    struct os_mbuf *txom = ble_hs_mbuf_from_flat(&response, 1);
    if (txom) {
        ble_gatts_notify_custom(conn_handle, ota_control_val_handle, txom);
    } else {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Handle OTA data write (with sequence number and CRC16)
 */
static esp_err_t handle_ota_data_write(uint16_t conn_handle, const uint8_t *data, size_t len)
{
    uint8_t response = OTA_RESP_OK;

    if (data == NULL || len < 6) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Invalid data packet: too short (len=%d), aborting OTA", len);
        // Send error response and terminate OTA
        uint8_t error_resp = OTA_RESP_INVALID_PACKET;
        struct os_mbuf *txom = ble_hs_mbuf_from_flat(&error_resp, 1);
        if (txom) {
            ble_gatts_notify_custom(conn_handle, ota_control_val_handle, txom);
        }
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
        ota_cleanup();
        return ESP_ERR_INVALID_ARG;
    }

    // Parse packet header: [SEQ(2)] [LENGTH(2)]
    uint16_t seq = le16toh(*(uint16_t*)&data[0]);
    uint16_t data_len = le16toh(*(uint16_t*)&data[2]);

    SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA data packet: seq=%d, data_len=%d, total_len=%d",
             seq, data_len, len);

    // Validate packet length
    size_t expected_len = 4 + data_len + 2;  // header + data + crc16
    if (len != expected_len) {
        uint16_t current_mtu = ble_att_mtu(conn_handle);
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG,
                 "Packet length mismatch: expected=%d, actual=%d, current_mtu=%d, aborting OTA",
                 expected_len, len, current_mtu);
        // Send error response and terminate OTA
        uint8_t error_resp = OTA_RESP_INVALID_PACKET;
        struct os_mbuf *txom = ble_hs_mbuf_from_flat(&error_resp, 1);
        if (txom) {
            ble_gatts_notify_custom(conn_handle, ota_control_val_handle, txom);
        }
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
        ota_cleanup();
        return ESP_ERR_INVALID_ARG;
    }

    // Extract data payload
    const uint8_t *payload = &data[4];

    // Verify CRC16
    uint16_t received_crc = le16toh(*(uint16_t*)&data[4 + data_len]);
    uint16_t calculated_crc = crc16_modbus(data, 4 + data_len);

    if (received_crc != calculated_crc) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG,
                 "CRC16 mismatch: received=0x%04x, calculated=0x%04x, aborting OTA", received_crc, calculated_crc);
        // Send error response and terminate OTA
        uint8_t error_resp = OTA_RESP_PACKET_CRC_ERROR;
        struct os_mbuf *txom = ble_hs_mbuf_from_flat(&error_resp, 1);
        if (txom) {
            ble_gatts_notify_custom(conn_handle, ota_control_val_handle, txom);
        }
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
        ota_cleanup();
        return ESP_ERR_INVALID_CRC;
    }

    // Check sequence number
    if (g_ota_first_packet) {
        if (seq != 0) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG,
                     "First packet sequence must be 0, got %d, aborting OTA", seq);
            // Send error response and terminate OTA
            uint8_t error_resp = OTA_RESP_SEQ_ERROR;
            struct os_mbuf *txom = ble_hs_mbuf_from_flat(&error_resp, 1);
            if (txom) {
                ble_gatts_notify_custom(conn_handle, ota_control_val_handle, txom);
            }
            gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
            ota_cleanup();
            return ESP_ERR_INVALID_STATE;
        }
        g_ota_first_packet = false;
        g_ota_expected_seq = 1;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "First packet received, expecting seq=1 next");
    } else {
        if (seq != g_ota_expected_seq) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG,
                     "Sequence number error: expected=%d, received=%d, aborting OTA", g_ota_expected_seq, seq);
            // Send error response and terminate OTA
            uint8_t error_resp = OTA_RESP_SEQ_ERROR;
            struct os_mbuf *txom = ble_hs_mbuf_from_flat(&error_resp, 1);
            if (txom) {
                ble_gatts_notify_custom(conn_handle, ota_control_val_handle, txom);
            }
            gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
            ota_cleanup();
            return ESP_ERR_INVALID_STATE;
        }
        g_ota_expected_seq++;
    }

    // Reset timeout timer for MCU OTA
    ota_partition_type_t part_type = ota_partition_get_type();
    if (part_type == OTA_PARTITION_MCU) {
        ota_timeout_start();  // Reset timeout timer on each data packet
    }

    // Write data to partition
    esp_err_t ret = ota_partition_write(payload, data_len);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to write OTA data: %s", esp_err_to_name(ret));
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
        ota_cleanup();
        return ret;
    }

    // Calculate progress
    size_t written = ota_partition_get_written_size();
    size_t total = ota_partition_get_total_size();
    uint8_t progress = (total > 0) ? (uint8_t)((written * 100) / total) : 0;

    SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA progress: %d/%d bytes (%d%%)", written, total, progress);

    // Only send status notify every 5% progress change to reduce BLE traffic
    // Also send on first packet (0%) and completion (100%)
    if (progress == 0 || progress >= g_ota_last_notified_progress + 5 || progress == 100) {
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_WRITING, progress);
        g_ota_last_notified_progress = progress;
        SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "Status notify sent: %d%%", progress);
    }

    // Check if all data has been written (final packet)
    bool is_final_packet = (written >= total);

    // Send ACK for this packet
    response = OTA_RESP_OK;
    struct os_mbuf *txom = ble_hs_mbuf_from_flat(&response, 1);
    if (txom) {
        ble_gatts_notify_custom(conn_handle, ota_control_val_handle, txom);
    }

    // For final packet, wait a bit to ensure ACK is sent before continuing
    // This prevents client write timeout when ACK is lost
    if (is_final_packet) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "Final packet ACK sent, waiting 50ms before finalization");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Check if all data has been written
    if (is_final_packet) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "All data written, finalizing OTA...");
        ota_timeout_stop();  // Stop timeout timer when data transfer complete
        gatt_ota_server_send_status(conn_handle, OTA_STATUS_VERIFYING, 100);

        // Finalize and verify
        ret = ota_partition_finalize();
        if (ret == ESP_OK) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA completed successfully");
            gatt_ota_server_send_status(conn_handle, OTA_STATUS_SUCCESS, 100);

            // Check partition type to determine next action
            part_type = ota_partition_get_type();

            SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA partition type: %d", part_type);

            // Handle MCU OTA - reboot to apply new firmware
            if (part_type == OTA_PARTITION_MCU) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "MCU firmware updated, rebooting to apply...");
                SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Restarting in 3 seconds...");

                // Delay to allow final status to be sent
                vTaskDelay(pdMS_TO_TICKS(3000));
                esp_restart();

                // Code will not reach here due to restart
                return ESP_OK;
            }

            // Handle TT module OTA - start XMODEM update from partition
            if (part_type == OTA_PARTITION_TT) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT module firmware downloaded, starting XMODEM update...");

                // Notify client that TT module update is starting
                gatt_ota_server_send_status(conn_handle, OTA_STATUS_WRITING, 0);

                // Delay a bit to allow status to be sent
                vTaskDelay(pdMS_TO_TICKS(500));

                // Define progress callback to notify BLE client
                void tt_ota_progress_callback(tt_ota_state_t state, int progress) {
                    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT OTA progress: state=%d, progress=%d%%", state, progress);

                    // Map TT OTA state to GATT OTA status
                    uint8_t gatt_status;
                    switch (state) {
                        case TT_OTA_STATE_UPLOADING:
                        case TT_OTA_STATE_VERIFYING:
                            gatt_status = OTA_STATUS_WRITING;
                            break;
                        case TT_OTA_STATE_COMPLETED:
                            gatt_status = OTA_STATUS_SUCCESS;
                            // Clean up OTA state when TT OTA completes
                            ota_cleanup();
                            break;
                        case TT_OTA_STATE_FAILED:
                            gatt_status = OTA_STATUS_FAILED;
                            // Clean up OTA state on failure
                            ota_cleanup();
                            break;
                        default:
                            gatt_status = OTA_STATUS_WRITING;
                            break;
                    }

                    // Send status update to BLE client
                    gatt_ota_server_send_status(conn_handle, gatt_status, progress);
                }

                // Start TT module firmware update from partition
                ret = tt_module_ota_start_update(tt_ota_progress_callback);
                if (ret != ESP_OK) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to start TT module OTA: %s", esp_err_to_name(ret));
                    gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 0);
                    ota_cleanup();
                }
            }
        } else {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA finalization failed: %s", esp_err_to_name(ret));
            gatt_ota_server_send_status(conn_handle, OTA_STATUS_FAILED, 100);
            ota_cleanup();
        }
    }

    return ESP_OK;
}

/**
 * @brief OTA Service Handler
 */
static int ota_service_handler(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Check if service is enabled
    if (!g_ota_service_enabled) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA service is disabled");
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (attr_handle == ota_status_val_handle) {
            // Return current OTA status
            uint8_t status_data[2];
            ota_status_t status = ota_partition_get_status();
            size_t written = ota_partition_get_written_size();
            size_t total = ota_partition_get_total_size();
            uint8_t progress = (total > 0) ? (uint8_t)((written * 100) / total) : 0;

            status_data[0] = (uint8_t)status;
            status_data[1] = progress;

            int rc = os_mbuf_append(ctxt->om, status_data, sizeof(status_data));
            if (rc != 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        }
        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        {
            if (attr_handle == ota_control_val_handle) {
                // Handle control command
                SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA Control command received: len=%d", ctxt->om->om_len);

                // Check size limit
                if (ctxt->om->om_len > GATT_OTA_MAX_DATA_SIZE) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Control data too large: %d", ctxt->om->om_len);
                    return BLE_HS_ENOMEM;
                }

                // Copy command data to local buffer
                uint8_t cmd_data[GATT_OTA_MAX_DATA_SIZE];
                int rc = ble_hs_mbuf_to_flat(ctxt->om, cmd_data, ctxt->om->om_len, NULL);
                if (rc != 0) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to copy command data: %d", rc);
                    return rc;
                }

                // Process command directly
                if (handle_ota_control_command(conn_handle, cmd_data, ctxt->om->om_len) != ESP_OK) {
                    return BLE_HS_ENOMEM;
                }
            } else if (attr_handle == ota_data_val_handle) {
                // Handle firmware data
                SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA data received: len=%d", ctxt->om->om_len);

                // Check size limit
                if (ctxt->om->om_len > GATT_OTA_MAX_DATA_SIZE) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Data too large: %d", ctxt->om->om_len);
                    return BLE_HS_ENOMEM;
                }

                // Check if OTA is in progress
                if (!g_ota_in_progress) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA not in progress, ignoring data");
                    return BLE_ATT_ERR_UNLIKELY;
                }

                // Copy data to local buffer
                uint8_t data_buf[GATT_OTA_MAX_DATA_SIZE];
                int rc = ble_hs_mbuf_to_flat(ctxt->om, data_buf, ctxt->om->om_len, NULL);
                if (rc != 0) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to copy data: %d", rc);
                    return rc;
                }

                // Process data directly
                esp_err_t ret = handle_ota_data_write(conn_handle, data_buf, ctxt->om->om_len);
                if (ret != ESP_OK) {
                    // Data processing failed
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to process OTA data: %s", esp_err_to_name(ret));
                    return BLE_HS_EAPP;
                }
            }
        }
        break;

    default:
        break;
    }
    return 0;
}

/**
 * @brief Send OTA status notification
 */
int gatt_ota_server_send_status(uint16_t conn_handle, uint8_t status, uint8_t progress)
{
    uint8_t status_data[2];
    status_data[0] = status;
    status_data[1] = progress;

    struct os_mbuf *txom = ble_hs_mbuf_from_flat(status_data, sizeof(status_data));
    if (!txom) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to allocate mbuf");
        return BLE_HS_ENOMEM;
    }

    int rc = ble_gatts_notify_custom(conn_handle, ota_status_val_handle, txom);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to send status notification: rc=%d", rc);
        os_mbuf_free(txom);
        return rc;
    }

    return 0;
}

/* OTA Service Definition */
static const struct ble_gatt_svc_def ota_service_defs[] = {
    {
        /*** Service: OTA Firmware Update ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_OTA_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* Control Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_OTA_CHR_CONTROL_UUID16),
                .access_cb = ota_service_handler,
                .val_handle = &ota_control_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Data Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_OTA_CHR_DATA_UUID16),
                .access_cb = ota_service_handler,
                .val_handle = &ota_data_val_handle,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                /* Status Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_OTA_CHR_STATUS_UUID16),
                .access_cb = ota_service_handler,
                .val_handle = &ota_status_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services. */
    },
};

/**
 * @brief Initialize GATT OTA Server (only count config, do NOT register)
 */
int gatt_ota_server_init(void)
{
    int rc;

    /* Prevent re-initialization */
    if (g_ota_server_initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "GATT OTA server already initialized");
        return 0;
    }

    // Create OTA timeout timer
    const esp_timer_create_args_t ota_timeout_timer_args = {
        .callback = &ota_timeout_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ota_timeout"
    };

    rc = esp_timer_create(&ota_timeout_timer_args, &g_ota_timeout_timer);
    if (rc != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to create OTA timeout timer: %d", rc);
        return rc;
    }

    // Count service configuration
    rc = ble_gatts_count_cfg(ota_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to count OTA service config: rc=%d", rc);
        return rc;
    }

    // Register service during initialization (must be done before connection)
    rc = ble_gatts_add_svcs(ota_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Failed to add OTA service: rc=%d", rc);
        return rc;
    }
    g_ota_service_registered = true;

    g_ota_server_initialized = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "GATT OTA server initialized (service registered)");
    return 0;
}

/**
 * @brief Enable OTA service
 */
void gatt_ota_server_enable(void)
{
    g_ota_service_enabled = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA service enabled");
}

/**
 * @brief Disable OTA service
 */
void gatt_ota_server_disable(void)
{
    g_ota_service_enabled = false;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "OTA service disabled");
}

/**
 * @brief Check if OTA is in progress
 */
bool gatt_ota_is_in_progress(void)
{
    return g_ota_in_progress;
}

/**
 * @brief Clean up OTA state on disconnect
 *
 * Call this when connection is lost to reset OTA state and abort partition operation.
 * This allows the client to reconnect and start a new OTA operation.
 */
void gatt_ota_server_cleanup_on_disconnect(uint16_t conn_handle)
{
    // Only clear OTA if this is the active OTA connection
    if (g_ota_conn_handle == conn_handle) {
        if (g_ota_in_progress) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG,
                "OTA connection (handle=%d) lost during OTA, cleaning up state", conn_handle);
            ota_cleanup();
        }
    } else if (g_ota_in_progress) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_OTA, TAG,
            "Disconnect conn_handle=%d does not match active OTA connection=%d, not clearing OTA",
            conn_handle, g_ota_conn_handle);
    }
}
