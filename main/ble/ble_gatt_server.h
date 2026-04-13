/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef BLE_GATT_SERVER_H
#define BLE_GATT_SERVER_H

#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define SPP_GATT_MTU_SIZE   512

// Function prototypes
void ble_spp_server_host_task(void *param);
int gatt_svr_init(void);
int ble_gatt_server_init(void);

#endif /* BLE_GATT_SERVER_H */
