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

static const char *TAG = "BLE_GATT_SERVER";

// Connection handle subscription state (with mutex protection)
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
static SemaphoreHandle_t conn_subs_mutex = NULL;
static uint8_t own_addr_type;
static int ble_active_conn_count = 0;

static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);

// Forward declaration for advertise function
static void ble_spp_server_advertise(void);

/**
 * @brief Thread-safe wrapper: Set subscription state for a connection handle
 *
 * @param conn_handle Connection handle
 * @param subscribed true if subscribed, false otherwise
 */
static void conn_set_subscribed(uint16_t conn_handle, bool subscribed)
{
    if (conn_handle > CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
        MODLOG_DFLT(ERROR, "Invalid conn_handle=%d (max %d)", conn_handle, CONFIG_BT_NIMBLE_MAX_CONNECTIONS);
        return;
    }

    if (conn_subs_mutex != NULL) {
        if (xSemaphoreTake(conn_subs_mutex, pdMS_TO_TICKS(CONN_SUBS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            conn_handle_subs[conn_handle] = subscribed;
            xSemaphoreGive(conn_subs_mutex);
        } else {
            MODLOG_DFLT(ERROR, "Failed to acquire conn_subs_mutex (timeout)");
        }
    } else {
        // Fallback during initialization
        conn_handle_subs[conn_handle] = subscribed;
    }
}

/**
 * @brief Thread-safe wrapper: Check if a connection handle is subscribed
 *
 * @param conn_handle Connection handle
 * @return true if subscribed, false otherwise
 */
static bool conn_is_subscribed(uint16_t conn_handle)
{
    if (conn_handle > CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
        return false;
    }

    bool subscribed = false;
    if (conn_subs_mutex != NULL) {
        if (xSemaphoreTake(conn_subs_mutex, pdMS_TO_TICKS(CONN_SUBS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            subscribed = conn_handle_subs[conn_handle];
            xSemaphoreGive(conn_subs_mutex);
        } else {
            MODLOG_DFLT(ERROR, "Failed to acquire conn_subs_mutex (timeout)");
        }
    } else {
        // Fallback during initialization
        subscribed = conn_handle_subs[conn_handle];
    }
    return subscribed;
}

/**
 * @brief Thread-safe wrapper: Clear all subscription states
 *
 * This function should be called during cleanup or reinitialization.
 */
static void conn_clear_all_subscriptions(void)
{
    if (conn_subs_mutex != NULL) {
        if (xSemaphoreTake(conn_subs_mutex, pdMS_TO_TICKS(CONN_SUBS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
                conn_handle_subs[i] = false;
            }
            xSemaphoreGive(conn_subs_mutex);
        } else {
            MODLOG_DFLT(ERROR, "Failed to acquire conn_subs_mutex (timeout)");
        }
    } else {
        // Fallback during initialization
        for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
            conn_handle_subs[i] = false;
        }
    }
}

/**
 * @brief Safe GATT notification send helper with automatic mbuf cleanup
 *
 * This function ensures that the mbuf is always freed, even if the
 * notification fails. This prevents memory leaks in error paths.
 *
 * @param conn_handle Connection handle
 * @param attr_handle Attribute handle
 * @param data Data to send
 * @param len Data length
 * @return 0 on success, non-zero error code on failure
 *
 * Note: This function takes ownership of the mbuf and will always free it.
 */
int ble_gatts_send_safe_notify(uint16_t conn_handle, uint16_t attr_handle,
                                const uint8_t *data, uint16_t len)
{
    // Enhanced parameter validation
    if (data == NULL && len > 0) {
        MODLOG_DFLT(ERROR, "Invalid parameters: data=NULL, len=%d", len);
        return BLE_HS_EINVAL;
    }

    if (len == 0) {
        MODLOG_DFLT(DEBUG, "Zero length data, skipping notification");
        return 0;
    }

    // Allocate mbuf
    struct os_mbuf *txom = ble_hs_mbuf_from_flat(data, len);
    if (txom == NULL) {
        MODLOG_DFLT(ERROR, "Failed to allocate mbuf (len=%d)", len);
        return BLE_HS_ENOMEM;
    }

    // Send notification (txom cannot be NULL here)
    int rc = ble_gatts_notify_custom(conn_handle, attr_handle, txom);

    // Handle result (mbuf ownership transferred to stack)
    if (rc == 0) {
        MODLOG_DFLT(DEBUG, "Notify sent successfully");
        // txom has been freed by NimBLE stack
    } else {
        MODLOG_DFLT(DEBUG, "Notify failed: rc=%d (mbuf freed by stack)", rc);
        // txom has been freed by NimBLE stack even on failure
    }

    return rc;
}

/**
 * @brief Safe fragmented GATT notification send helper
 *
 * This function sends large data payloads in multiple fragments to respect
 * the MTU size limit. Each fragment is sent as a separate notification.
 * The mbuf is always freed, even if sending fails.
 *
 * @param conn_handle Connection handle
 * @param attr_handle Attribute handle
 * @param data Data to send
 * @param len Total data length
 * @param mtu Maximum transmission unit (payload size per notification)
 * @return 0 on success, non-zero error code on failure
 *
 * Note: This function takes ownership of the mbuf and will always free it.
 */
int ble_gatts_send_safe_notify_fragmented(uint16_t conn_handle, uint16_t attr_handle,
                                          const uint8_t *data, uint16_t len, uint16_t mtu)
{
    // Enhanced parameter validation
    if (data == NULL && len > 0) {
        MODLOG_DFLT(ERROR, "Invalid parameters: data=NULL, len=%d", len);
        return BLE_HS_EINVAL;
    }

    if (mtu == 0) {
        MODLOG_DFLT(ERROR, "Invalid parameter: mtu=0");
        return BLE_HS_EINVAL;
    }

    if (len == 0) {
        MODLOG_DFLT(DEBUG, "Zero length data, skipping fragmented notification");
        return 0;
    }

    // Send data in MTU-sized chunks
    size_t offset = 0;
    int last_rc = 0;

    while (offset < len) {
        uint16_t chunk_len = (len - offset > mtu) ? mtu : (len - offset);

        // Allocate mbuf for this fragment
        struct os_mbuf *txom = ble_hs_mbuf_from_flat(data + offset, chunk_len);
        if (txom == NULL) {
            MODLOG_DFLT(ERROR, "Failed to allocate mbuf for fragment (offset=%zu, len=%d)", offset, chunk_len);
            return BLE_HS_ENOMEM;
        }

        // Send this fragment
        int rc = ble_gatts_notify_custom(conn_handle, attr_handle, txom);
        if (rc != 0) {
            MODLOG_DFLT(ERROR, "Fragment send failed: rc=%d (offset=%zu)", rc, offset);
            // txom has been freed by NimBLE stack even on failure
            return rc;
        }

        offset += chunk_len;
        last_rc = rc;
    }

    return last_rc;
}

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

/**
 * @brief Handle GAP connection established event
 */
static int handle_gap_event_connect(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                event->connect.status == 0 ? "established" : "failed",
                event->connect.status);

    if (event->connect.status != 0) {
        // Connection attempt failed
        MODLOG_DFLT(INFO, "Connection attempt failed\n");
#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
        if (ble_conn_manager_get_connection_count() == 0) {
            ble_spp_server_advertise();
        }
#else
        ble_active_conn_count = 0;
        ble_spp_server_advertise();
#endif
        return 0;
    }

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
    if (ble_conn_manager_is_max_connections()) {
        MODLOG_DFLT(INFO, "Max connections reached (%d), rejecting new connection\n",
                    BLE_MAX_CONNECTIONS);
        ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
#else
    if (ble_active_conn_count >= 1) {
        MODLOG_DFLT(INFO, "Connection already exists (count=%d), rejecting new connection\n",
                    ble_active_conn_count);
        ble_gap_adv_stop();
        ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
#endif

    rc = ble_att_set_preferred_mtu(SPP_GATT_MTU_SIZE);
    if (rc != 0) {
        MODLOG_DFLT(INFO, "Failed to set preferred MTU; rc = %d", rc);
    }

    rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to find connection: rc=%d", rc);
        return rc;
    }
    ble_spp_server_print_conn_desc(&desc);

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
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

    syslog_set_gatt_conn_handle(event->connect.conn_handle);

    extern void tt_module_set_active_gatt_conn(uint16_t conn_handle);
    tt_module_set_active_gatt_conn(event->connect.conn_handle);

    ble_conn_manager_print_info();

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

    syslog_set_gatt_conn_handle(event->connect.conn_handle);

    extern void tt_module_set_active_gatt_conn(uint16_t conn_handle);
    tt_module_set_active_gatt_conn(event->connect.conn_handle);

    ble_gap_adv_stop();
#endif

    return 0;
}

/**
 * @brief Handle GAP disconnection event
 */
static int handle_gap_event_disconnect(struct ble_gap_event *event, void *arg)
{
    MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

#ifdef CONFIG_BLE_MULTI_CONN_ENABLE
    ble_conn_role_t role = ble_conn_manager_get_role(event->disconnect.conn.conn_handle);
    ble_conn_manager_remove_connection(event->disconnect.conn.conn_handle);

    syslog_clear_gatt_conn_handle();
    conn_set_subscribed(event->disconnect.conn.conn_handle, false);
    spp_at_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);
    spp_voice_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);
    gatt_ota_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);

    ble_conn_manager_print_info();

    if (ble_conn_manager_get_connection_count() == 0) {
        MODLOG_DFLT(INFO, "All connections closed, restarting advertising\n");
        ble_spp_server_advertise();
    } else {
        MODLOG_DFLT(INFO, "Still have %d active connection(s), advertising remains off\n",
                    ble_conn_manager_get_connection_count());
    }
#else
    ble_active_conn_count = 0;

    syslog_clear_gatt_conn_handle();
    conn_set_subscribed(event->disconnect.conn.conn_handle, false);
    spp_at_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);
    spp_voice_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);
    gatt_ota_server_cleanup_on_disconnect(event->disconnect.conn.conn_handle);

    MODLOG_DFLT(INFO, "All connections closed, restarting advertising\n");
    ble_spp_server_advertise();
#endif

    return 0;
}

/**
 * @brief Handle GAP subscription event
 */
static int handle_gap_event_subscribe(struct ble_gap_event *event, void *arg)
{
    MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                event->subscribe.conn_handle,
                event->subscribe.attr_handle,
                event->subscribe.reason,
                event->subscribe.prev_notify,
                event->subscribe.cur_notify,
                event->subscribe.prev_indicate,
                event->subscribe.cur_indicate);

    conn_set_subscribed(event->subscribe.conn_handle, true);
    return 0;
}

/**
 * @brief Handle GAP connection update event
 */
static int handle_gap_event_conn_update(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to find connection during update: rc=%d", rc);
        return rc;
    }
    MODLOG_DFLT(INFO, "conn update(%d)\n", desc.conn_itvl);
    ble_spp_server_print_conn_desc(&desc);

    if (spp_voice_server_is_enabled()) {
        if (desc.conn_itvl < BLE_VOICE_CONN_ITVL_MIN || desc.conn_itvl > BLE_VOICE_CONN_ITVL_MAX) {
            struct ble_gap_upd_params params = {
                .itvl_min = BLE_VOICE_CONN_ITVL_MIN,
                .itvl_max = BLE_VOICE_CONN_ITVL_MAX,
                .latency = BLE_CONN_LATENCY_NONE,
                .supervision_timeout = BLE_SUPERVISION_TIMEOUT_400,
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "failed to update voice connection parameters: %d\n", rc);
                return rc;
            }
            MODLOG_DFLT(INFO, "voice mode: requested low-latency params (itvl=%d-%d)\n",
                        BLE_VOICE_CONN_ITVL_MIN, BLE_VOICE_CONN_ITVL_MAX);
        }
    } else {
        if (desc.conn_itvl < BLE_VOICE_CONN_ITVL_MIN || desc.conn_itvl > BLE_VOICE_CONN_ITVL_MAX) {
            struct ble_gap_upd_params params = {
                .itvl_min = BLE_DEFAULT_CONN_ITVL_MIN,
                .itvl_max = BLE_DEFAULT_CONN_ITVL_MAX,
                .latency = BLE_CONN_LATENCY_NONE,
                .supervision_timeout = BLE_SUPERVISION_TIMEOUT_400,
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "failed to update default connection parameters: %d\n", rc);
                return rc;
            }
            MODLOG_DFLT(INFO, "normal mode: requested default params (itvl=%d-%d)\n",
                        BLE_DEFAULT_CONN_ITVL_MIN, BLE_DEFAULT_CONN_ITVL_MAX);
        }
    }

    return 0;
}

/**
 * @brief Main GAP event handler - delegates to specific handlers
 */
static int
ble_spp_server_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        return handle_gap_event_connect(event, arg);

    case BLE_GAP_EVENT_DISCONNECT:
        return handle_gap_event_disconnect(event, arg);

    case BLE_GAP_EVENT_ADV_COMPLETE:
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle = %d cid = %d mtu = %d",
                 event->mtu.conn_handle,
                 event->mtu.channel_id,
                 event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        return handle_gap_event_subscribe(event, arg);

    case BLE_GAP_EVENT_CONN_UPDATE:
        return handle_gap_event_conn_update(event, arg);

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
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Failed to ensure address: rc=%d", rc);
        return;
    }

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
        SYS_LOGE_MODULE(SYS_LOG_MODULE_BLE_GATT, TAG, "Unknown GATT register operation: %d", ctxt->op);
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

    /* Create mutex for protecting conn_handle_subs[] array */
    conn_subs_mutex = xSemaphoreCreateMutex();
    if (conn_subs_mutex == NULL) {
        MODLOG_DFLT(ERROR, "Failed to create conn_subs_mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize connection_handle array (thread-safe via mutex) */
    conn_clear_all_subscriptions();

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

    /*
     * NimBLE Store Configuration:
     *
     * Initializes the persistent store for BLE bonding data.
     * This includes:
     * - Bonding information (encryption keys)
     * - Pairing records
     * - Device whitelist
     *
     * Note: ble_store_config_init() is provided by NimBLE.
     * It uses the default RAM-based store implementation.
     * For persistent storage across reboots, NVS-backed store should be configured.
     *
     * Current Implementation: RAM-based (lost on reboot)
     * Future Enhancement: Use ble_store_nvs for persistent bonding
     */
    ble_store_config_init();

    /* Create FreeRTOS task for the NimBLE host */
    nimble_port_freertos_init(ble_spp_server_host_task);

    return 0;
}
