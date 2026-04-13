/*
 * spp_voice_server.h - SPP Voice Service for BLE GATT
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef SPP_VOICE_SERVER_H
#define SPP_VOICE_SERVER_H

#include <stdint.h>
#include "host/ble_hs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SPP Voice Service UUIDs */
#define BLE_SVC_SPP_VOICE_UUID16        0xABF0  /* Service UUID */
#define BLE_SVC_SPP_VOICE_CHR_UUID16    0xABF1  /* Characteristic UUID */

/**
 * @brief Initialize SPP Voice Service
 *
 * This function initializes the SPP Voice service for voice data transmission.
 * The service handles AT^AUDPCM commands and forwards them to voice_packet_handler.
 *
 * @return 0 on success, negative error code otherwise
 */
int spp_voice_server_init(void);

/**
 * @brief Send voice data to BLE client
 *
 * This function sends formatted AT^AUDPCM command with base64-encoded voice data
 * to the connected BLE client via GATT notification.
 *
 * @param conn_handle BLE connection handle
 * @param data AT^AUDPCM command string (e.g., "AT^AUDPCM=\"base64_data\"")
 * @param len Length of the command string
 * @return 0 on success, negative error code otherwise
 */
int spp_voice_server_send(uint16_t conn_handle, const uint8_t *data, uint16_t len);

/**
 * @brief Get SPP Voice service characteristic value handle
 *
 * @return Characteristic value handle
 */
uint16_t spp_voice_server_get_val_handle(void);

/**
 * @brief Enable Voice service
 */
void spp_voice_server_enable(void);

/**
 * @brief Disable Voice service
 */
void spp_voice_server_disable(void);

/**
 * @brief Check if Voice service is enabled
 *
 * @return true if enabled, false otherwise
 */
bool spp_voice_server_is_enabled(void);

/**
 * @brief Clean up voice server state on BLE disconnect
 *
 * Only clears state if the disconnected connection matches the active connection.
 * This is important for multi-connection support where DEBUG connections may
 * disconnect without affecting the PRIMARY connection's voice service.
 *
 * @param conn_handle BLE connection handle that disconnected
 */
void spp_voice_server_cleanup_on_disconnect(uint16_t conn_handle);

#ifdef __cplusplus
}
#endif

#endif /* SPP_VOICE_SERVER_H */
