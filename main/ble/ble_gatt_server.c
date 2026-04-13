/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "ble/ble_gatt_server.h"
#include "ble/ble_conn_manager.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"
#include "ble/spp_at_server.h"
#include "ble/spp_voice_server.h"
#include "ble/gatt_system_server.h"
#include "ble/gatt_ota_server.h"
#include "ble/gatt_log_server.h"
#include "audio/voice_packet_handler.h"
#include "system/syslog.h"

// External variables
bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
static uint8_t own_addr_type;
static int ble_active_conn_count = 0;

static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);

/**
 * Logs information about a connection to the console.
 */
static void
ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                desc->our_ota_addr.val[0], desc->our_ota_addr.val[1],
                desc->our_ota_addr.val[2], desc->our_ota_addr.val[3],
                desc->our_ota_addr.val[4], desc->our_ota_addr.val[5]);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                desc->our_id_addr.val[0], desc->our_id_addr.val[1],
                desc->our_id_addr.val[2], desc->our_id_addr.val[3],
                desc->our_id_addr.val[4], desc->our_id_addr.val[5]);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                desc->peer_ota_addr.val[0], desc->peer_ota_addr.val[1],
                desc->peer_ota_addr.val[2], desc->peer_ota_addr.val[3],
                desc->peer_ota_addr.val[4], desc->peer_ota_addr.val[5]);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                desc->peer_id_addr.val[0], desc->peer_id_addr.val[1],
                desc->peer_id_addr.val[2], desc->peer_id_addr.val[3],
                desc->peer_id_addr.val[4], desc->peer_id_addr.val[5]);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ble_spp_server_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    /* Indicate that the service UUID list field should be included. */
    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(BLE_SVC_SPP_VOICE_UUID16)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_spp_server_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

#define BLE_VOICE_ITVL_MIN  8
#define BLE_VOICE_ITVL_MAX  10

static int
ble_spp_server_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status == 0) {
#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
            // Multi-connection mode: check max connections
            if (ble_conn_manager_is_max_connections()) {
                MODLOG_DFLT(INFO, "Max connections reached (%d), rejecting new connection\n",
                            BLE_MAX_CONNECTIONS);
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
#else
            // Single-connection mode: only allow 1 connection
            if (ble_active_conn_count >= 1) {
                MODLOG_DFLT(INFO, "Connection already exists (count=%d), rejecting new connection\n",
                            ble_active_conn_count);
                // Stop advertising to prevent further connection attempts
                ble_gap_adv_stop();
                // Terminate the new connection immediately
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
#endif

            rc = ble_att_set_preferred_mtu(SPP_GATT_MTU_SIZE);
            if (rc != 0) {
                MODLOG_DFLT(INFO, "Failed to set preferred MTU; rc = %d", rc);
            }

            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            ble_spp_server_print_conn_desc(&desc);

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
            // Add connection and assign role
            ble_conn_role_t role = ble_conn_manager_add_connection(
                event->connect.conn_handle,
                desc.peer_id_addr.val,
                desc.peer_id_addr.type
            );

            if (role == BLE_CONN_ROLE_NONE) {
                MODLOG_DFLT(ERROR, "Failed to add connection\n");
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }

            MODLOG_DFLT(INFO, "Connection role: %s\n",
                        role == BLE_CONN_ROLE_PRIMARY ? "PRIMARY" : "DEBUG");

            // Set GATT connection handle for syslog (will route to DEBUG if exists)
            syslog_set_gatt_conn_handle(event->connect.conn_handle);

            // Set active GATT connection for tt_module (for unsolicited AT notifications)
            extern void tt_module_set_active_gatt_conn(uint16_t conn_handle);
            tt_module_set_active_gatt_conn(event->connect.conn_handle);

            // Print connection manager state
            ble_conn_manager_print_info();

            // Stop advertising if max connections reached
            if (ble_conn_manager_is_max_connections()) {
                MODLOG_DFLT(INFO, "Max connections reached, stopping advertising\n");
                ble_gap_adv_stop();
            } else {
                MODLOG_DFLT(INFO, "Waiting for more connections (%d/%d)\n",
                            ble_conn_manager_get_connection_count(), BLE_MAX_CONNECTIONS);
            }
#else
            ble_active_conn_count++;
            MODLOG_DFLT(INFO, "Active connections: %d\n", ble_active_conn_count);

            // Set GATT connection handle for syslog
            syslog_set_gatt_conn_handle(event->connect.conn_handle);

            // Set active GATT connection for tt_module (for unsolicited AT notifications)
            extern void tt_module_set_active_gatt_conn(uint16_t conn_handle);
            tt_module_set_active_gatt_conn(event->connect.conn_handle);

            // Stop advertising after connection established
            ble_gap_adv_stop();
#endif
        } else {
            // Connection attempt failed
            MODLOG_DFLT(INFO, "Connection attempt failed\n");
#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
            // In multi-connection mode, only restart advertising if no connections
            if (ble_conn_manager_get_connection_count() == 0) {
                ble_spp_server_advertise();
            }
#else
            ble_active_conn_count = 0;
            ble_spp_server_advertise();
#endif
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
        // Remove connection from manager (handles role persistence)
        ble_conn_role_t role = ble_conn_manager_get_role(event->disconnect.conn.conn_handle);
        ble_conn_manager_remove_connection(event->disconnect.conn.conn_handle);

        // Clear GATT connection handle for syslog if needed
        // Note: syslog will auto-route to remaining connection
        syslog_clear_gatt_conn_handle();

        // Clear subscription state for this connection
        conn_handle_subs[event->disconnect.conn.conn_handle] = false;

        // Clean up AT server state (only if this was the active AT connection)
        spp_at_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);

        // Clean up voice server state (only if this was the active voice connection)
        spp_voice_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);

        // Clean up OTA state if OTA was in progress (only if this was the OTA connection)
        gatt_ota_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);

        // Print connection manager state
        ble_conn_manager_print_info();

        // Restart advertising only if no connections remain
        // (avoid interrupting primary work if one connection still active)
        if (ble_conn_manager_get_connection_count() == 0) {
            MODLOG_DFLT(INFO, "All connections closed, restarting advertising\n");
            ble_spp_server_advertise();
        } else {
            MODLOG_DFLT(INFO, "Still have %d active connection(s), advertising remains off\n",
                        ble_conn_manager_get_connection_count());
        }
#else
        ble_active_conn_count = 0;

        // Clear GATT connection handle for syslog
        syslog_clear_gatt_conn_handle();

        // Clear subscription state for this connection
        conn_handle_subs[event->disconnect.conn.conn_handle] = false;

        // Clean up AT server state (only if this was the active AT connection)
        spp_at_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);

        // Clean up voice server state (only if this was the active voice connection)
        spp_voice_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);

        // Clean up OTA state if OTA was in progress (only if this was the OTA connection)
        gatt_ota_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);

        MODLOG_DFLT(INFO, "All connections closed, restarting advertising\n");
        ble_spp_server_advertise();
#endif
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        // Advertising stopped, don't restart (will be restarted on disconnect or error)
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle = %d cid = %d mtu = %d",
                 event->mtu.conn_handle,
                 event->mtu.channel_id,
                 event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                    "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        conn_handle_subs[event->subscribe.conn_handle] = true;
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        MODLOG_DFLT(INFO, "conn update(%d)\n", desc.conn_itvl);
        assert(rc == 0);
        ble_spp_server_print_conn_desc(&desc);

        /* Update connection parameters based on voice service state */
        if (spp_voice_server_is_enabled()) {
            /* Voice service enabled: ensure low-latency parameters */
            if (desc.conn_itvl < BLE_VOICE_ITVL_MIN || desc.conn_itvl > BLE_VOICE_ITVL_MAX) {
                struct ble_gap_upd_params params = {
                    .itvl_min = BLE_VOICE_ITVL_MIN,         /* 17.5ms */
                    .itvl_max = BLE_VOICE_ITVL_MAX,         /* 20ms */
                    .latency = 0,
                    .supervision_timeout = 400,
                    .min_ce_len = 0,
                    .max_ce_len = 0,
                };
                rc = ble_gap_update_params(event->connect.conn_handle, &params);
                if (rc != 0) {
                    MODLOG_DFLT(ERROR, "failed to update voice connection parameters: %d\n", rc);
                    return rc;
                }
                MODLOG_DFLT(INFO, "voice mode: requested low-latency params (itvl=%d-%d)\n", BLE_VOICE_ITVL_MIN, BLE_VOICE_ITVL_MAX);
            }
        } else {
            /* Voice service disabled: ensure default/power-saving parameters */
            if (desc.conn_itvl < BLE_VOICE_ITVL_MIN || desc.conn_itvl > BLE_VOICE_ITVL_MAX) {
                struct ble_gap_upd_params params = {
                    .itvl_min = BLE_VOICE_ITVL_MIN,         /* 30ms (default) */
                    .itvl_max = BLE_VOICE_ITVL_MAX,         /* 50ms (default) */
                    .latency = 0,
                    .supervision_timeout = 400,
                    .min_ce_len = 0,
                    .max_ce_len = 0,
                };
                rc = ble_gap_update_params(event->connect.conn_handle, &params);
                if (rc != 0) {
                    MODLOG_DFLT(ERROR, "failed to update default connection parameters: %d\n", rc);
                    return rc;
                }
                MODLOG_DFLT(INFO, "normal mode: requested default params (itvl=14-16--default)\n");
            }
        }
        return 0;

    default:
        return 0;
    }
}

static void
ble_spp_server_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
ble_spp_server_on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                addr_val[0], addr_val[1], addr_val[2], addr_val[3], addr_val[4], addr_val[5]);
    /* Begin advertising. */
    ble_spp_server_advertise();
}

void ble_spp_server_host_task(void *param)
{
    MODLOG_DFLT(INFO, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int gatt_svr_init(void)
{
    int rc = 0;

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
    /* Initialize connection manager (for multi-connection support) */
    rc = ble_conn_manager_init();
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to init connection manager: %d", rc);
        return rc;
    }
    MODLOG_DFLT(INFO, "Connection manager initialized (max_connections=%d)", BLE_MAX_CONNECTIONS);
#endif

    /* Initialize GAP and GATT services (standard BLE services) */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Register AT command service (AT command passthrough) - Always running */
    rc = spp_at_server_init();
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to init SPP AT server: %d", rc);
        return rc;
    }

    /* Register SYSTEM service (System control and service management, 0xABFC) - Always running */
    rc = gatt_system_server_init();
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to init GATT SYSTEM server: %d", rc);
        return rc;
    }

    /* Register Voice service (registered but disabled by default) */
    rc = spp_voice_server_init();
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to init SPP Voice server: %d", rc);
        return rc;
    }

    /* Register OTA service (registered but disabled by default) */
    rc = gatt_ota_server_init();
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to init GATT OTA server: %d", rc);
        return rc;
    }

    /* Register Log service (registered but disabled by default) */
    rc = gatt_log_server_init();
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to init GATT Log server: %d", rc);
        return rc;
    }

    MODLOG_DFLT(INFO, "All GATT services initialized (AT, SYSTEM, Voice, OTA, LOG)");
    MODLOG_DFLT(INFO, "Voice, LOG, and OTA services will be enabled dynamically via System service");
    return 0;
}

int ble_gatt_server_init(void)
{
    esp_err_t ret;
    int rc;

    /* Initialize NimBLE port */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return ret;
    }

    /* Initialize connection_handle array */
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        conn_handle_subs[i] = false;
    }

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = ble_spp_server_on_reset;
    ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = CONFIG_EXAMPLE_IO_TYPE;
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_bonding = 1;
#endif
#ifdef CONFIG_EXAMPLE_MITM
    ble_hs_cfg.sm_mitm = 1;
#endif
#ifdef CONFIG_EXAMPLE_USE_SC
    ble_hs_cfg.sm_sc = 1;
#else
    ble_hs_cfg.sm_sc = 0;
#endif
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
#endif

    /* Register custom service */
    rc = gatt_svr_init();
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to init GATT server %d \n", rc);
        return rc;
    }

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("TTCat");
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to set device name %d \n", rc);
        return rc;
    }

    /* XXX Need to have template for store */
    ble_store_config_init();

    /* Create FreeRTOS task for the NimBLE host */
    nimble_port_freertos_init(ble_spp_server_host_task);

    return 0;
}
