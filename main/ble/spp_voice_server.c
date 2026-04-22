/*
 * spp_voice_server.c - SPP Voice Service Implementation
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble/spp_voice_server.h"
#include "ble/ble_gatt_server.h"
#include "audio/voice_packet_handler.h"
#include "audio/audiosvc.h"
#include "tt/gsm0710_manager.h"
#include "system/syslog.h"

/* GATT Voice Loopback Test Mode */
// #define CONFIG_VOICE_GATT_LOOPBACK  // Uncomment to enable GATT loopback test

static const char *TAG = "SPP_VOICE_SERVER";

/* Static buffer for voice data - avoids repeated malloc/free */
#define SPP_VOICE_MAX_DATA_SIZE VOICE_BUFFER_SIZE  // Use constant from ble_gatt_server.h

/* Voice server context - encapsulates all global state */
typedef struct {
    bool initialized;
    bool enabled;
    bool registered;
    uint16_t active_conn_handle;
    uint16_t val_handle;
    uint8_t data_buffer[SPP_VOICE_MAX_DATA_SIZE];
} voice_server_context_t;

static voice_server_context_t g_voice_server = {
    .initialized = false,
    .enabled = true,
    .registered = false,
    .active_conn_handle = 0,
    .val_handle = 0,
};

/* Initialization check macro */
#define VOICE_SERVER_CHECK_INIT() \
    do { \
        if (!g_voice_server.initialized) { \
            SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice server not initialized"); \
            return ESP_ERR_INVALID_STATE; \
        } \
    } while(0)

/**
 * @brief Voice data output callback (called by voice_packet_handler)
 *
 * This callback receives encoded voice data (AT^AUDPCM format) from
 * voice_packet_handler and sends it to the BLE client.
 */
static void spp_voice_output_callback(const uint8_t *data, size_t len, void *user_data)
{
    (void)user_data;

    if (data == NULL || len == 0) {
        return;
    }

    // Check if voice service is enabled
    if (!g_voice_server.enabled) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice service disabled, data dropped");
        return;
    }

    // Check if we have an active connection
    if (g_voice_server.active_conn_handle == 0) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "No active voice connection, data dropped");
        return;
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Sending voice data to BLE: %d bytes", len);

    // Send to BLE GATT
    spp_voice_server_send(g_voice_server.active_conn_handle, data, len);
}

/**
 * @brief Voice downlink callback (sends voice data from BLE to MUX channel 9)
 *
 * This callback receives decoded voice data (PCM) from BLE client
 * and forwards it to gsm0710_manager for transmission via MUX channel 9.
 *
 * Supports multi-frame PCM: len may be 320 bytes (1 frame) or 960 bytes (3 frames).
 *
 * In loopback mode (CONFIG_VOICE_GATT_LOOPBACK), the PCM data is split into
 * individual 320-byte frames and re-enqueued to voice_data_queue for encoding
 * and sent back to the BLE client. The encode task will accumulate frames
 * based on g_frames_per_cmd and combine them into one AT command.
 */
static void spp_voice_downlink_callback(const uint8_t *data, size_t len, void *user_data)
{
    (void)user_data;

    if (data == NULL || len == 0) {
        return;
    }

    /* Split combined PCM into individual frames (320 bytes each) */
    if (len % AUDIO_FRAME_SIZE != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
            "Invalid PCM size %d (not aligned to %d)", len, AUDIO_FRAME_SIZE);
        return;
    }

    size_t frame_count = len / AUDIO_FRAME_SIZE;

#ifdef CONFIG_VOICE_GATT_LOOPBACK
    /* Loopback: enqueue each frame to voice_data_queue for re-encoding */
    SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
        "GATT LOOPBACK: %d frames (%d bytes) to split", frame_count, len);

    extern QueueHandle_t voice_data_queue;
    if (voice_data_queue == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "GATT LOOPBACK: voice_data_queue not initialized");
        return;
    }

    for (size_t i = 0; i < frame_count; i++) {
        int16_t frame_buf[AUDIO_FRAME_SIZE];
        memcpy(frame_buf, data + i * AUDIO_FRAME_SIZE, AUDIO_FRAME_SIZE);

        if (xQueueSend(voice_data_queue, frame_buf, pdMS_TO_TICKS(10)) != pdPASS) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                "GATT LOOPBACK: Failed to enqueue frame %d/%d", (int)(i + 1), (int)frame_count);
        }
    }
#else
    /* Normal: send each frame individually to MUX channel 9 */
    for (size_t i = 0; i < frame_count; i++) {
        esp_err_t ret = gsm0710_manager_send(GSM0710_CHANNEL_VOICE_DATA,
                                              data + i * AUDIO_FRAME_SIZE,
                                              AUDIO_FRAME_SIZE);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                "Failed to send frame %d/%d to MUX CH9: %d", (int)(i + 1), (int)frame_count, ret);
        }
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
        "Sent %d frames (%d bytes) to MUX CH9", (int)frame_count, (int)len);
#endif
}

/**
 * @brief MUX channel 9 data callback (receives voice data from TT module via MUX)
 *
 * This callback receives PCM voice data from MUX channel 9 and forwards it
 * to voice_data_queue for uplink processing (AMR encode -> Base64 encode -> BLE).
 *
 * Note: Only complete 320-byte frames are processed. Incomplete frames are discarded
 * because AMRNB encoder requires exactly 160 samples (320 bytes) per frame.
 */
static void spp_voice_mux9_callback(const uint8_t *data, size_t len, void *user_data)
{
    (void)user_data;

    if (data == NULL || len == 0) {
        return;
    }

    // Check if voice service is enabled (important: drop data if service disabled)
    if (!g_voice_server.enabled) {
        // Voice service disabled, drop data without logging (too verbose during call teardown)
        return;
    }

    // Check if voice_data_queue is available
    extern QueueHandle_t voice_data_queue;
    if (voice_data_queue == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice data queue not initialized");
        return;
    }

    // Audio frame size: 20ms @ 8kHz, 16-bit = 320 bytes
    // AMRNB encoder requires exactly 160 samples (320 bytes) per frame
    const size_t audio_frame_size = 320;

    // Calculate number of complete frames (discard incomplete frames)
    size_t complete_frames = len / audio_frame_size;
    size_t remainder = len % audio_frame_size;

    /* Debug: always log MUX9 received size to diagnose 3-frame mode */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
        "MUX9: Received %d bytes (%d complete frames, %d remainder)",
        len, complete_frames, remainder);

    if (complete_frames == 0) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
            "MUX9: Received %d bytes, less than one frame (%d bytes), discarding",
            len, audio_frame_size);
        return;
    }

    // Warn if there are incomplete bytes (will be discarded)
    if (remainder > 0) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
            "MUX9: Received %d bytes, discarding incomplete frame (%d bytes)",
            len, remainder);
    }

    // Process only complete frames
    size_t offset = 0;
    int success_count = 0;

    for (size_t i = 0; i < complete_frames; i++) {
        // Send each complete frame to voice_data_queue
        // voice_packet_send_task will: AMR encode -> Base64 encode -> AT^AUDPCM format -> send to BLE
        if (xQueueSend(voice_data_queue, data + offset, 0) != pdPASS) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                "Failed to send PCM frame %d to voice queue (queue full?)", i + 1);
            break;  // Stop sending if queue is full
        } else {
            success_count++;
        }
        offset += audio_frame_size;
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
        "MUX9: %d bytes → %d complete frames, queued=%d",
        len, complete_frames, success_count);
}

/**
 * @brief SPP Voice Service access handler
 *
 * Handles read and write operations on the SPP Voice characteristic.
 */
static int spp_voice_service_handler(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Check if service is enabled
    if (!g_voice_server.enabled) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice service is disabled");
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "SPP Voice Service - Read request");
        /* Read operation: return empty or current status */
        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        {
            uint16_t data_len = ctxt->om->om_len;

            SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "SPP Voice Service - Data received: len=%d", data_len);

            /* Check size limit */
            if (data_len > SPP_VOICE_MAX_DATA_SIZE) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Data too large: %d", data_len);
                return BLE_HS_ENOMEM;
            }

            /* Save active connection handle for responses */
            g_voice_server.active_conn_handle = conn_handle;

            /* Copy data to static buffer */
            int rc = ble_hs_mbuf_to_flat(ctxt->om, g_voice_server.data_buffer,
                                        SPP_VOICE_MAX_DATA_SIZE, NULL);
            if (rc != 0) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to copy voice data: %d", rc);
                return rc;
            }

            /* Enqueue for voice downlink processing (decode task on CPU1)
             * Decode task will:
             * 1. Parse AT^AUDPCM command
             * 2. Base64 decode
             * 3. AMRNB decode
             * 4. Send PCM to MUX channel 9
             */
            int ret = voice_downlink_enqueue(g_voice_server.data_buffer, data_len);
            if (ret != 0) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to enqueue voice packet: %d", ret);
            }
        }
        break;

    default:
        SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "SPP Voice Service - Default callback");
        break;
    }

    return 0;
}

/* SPP Voice Service definition */
static const struct ble_gatt_svc_def spp_voice_service_defs[] = {
    {
        /* Service: SPP Voice */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_VOICE_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* Voice Data Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_VOICE_CHR_UUID16),
                .access_cb = spp_voice_service_handler,
                .val_handle = &g_voice_server.val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services. */
    },
};

/* Initialize SPP Voice Service (only count config, do NOT register) */
int spp_voice_server_init(void)
{
    int rc;

    /* Start voice processing tasks (encode on CPU0, decode on CPU1) */
    TaskHandle_t voice_task = voice_tasks_start();
    if (voice_task == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to start voice tasks");
        return ESP_FAIL;
    }
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice tasks started (encode:CPU0, decode:CPU1)");

    /* Register output callback with voice_packet_handler (for uplink: TT -> BLE) */
    rc = voice_packet_register_output_callback(spp_voice_output_callback, NULL);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to register voice output callback: %d", rc);
        return rc;
    }

    /* Register input callback with gsm0710_manager (for downlink: BLE -> TT via MUX ch9) */
    rc = voice_packet_register_downlink_callback(spp_voice_downlink_callback, NULL);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to register voice downlink callback: %d", rc);
        // Continue anyway - voice output may still work
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Registered voice downlink callback");
    }

    /* Register with gsm0710_manager to receive voice data from MUX channel 9 */
    rc = gsm0710_manager_register_voice_callback((gsm0710_data_received_cb_t)spp_voice_mux9_callback, NULL);
    if (rc != ESP_OK) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to register MUX channel 9 callback: %d", rc);
        // Continue anyway - may work without MUX
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Registered MUX channel 9 callback for voice data");
    }

    /* Count service configuration (required before add_svcs) */
    rc = ble_gatts_count_cfg(spp_voice_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to count SPP Voice service config: rc=%d", rc);
        return rc;
    }

    /* Register service during initialization (must be done before connection) */
    rc = ble_gatts_add_svcs(spp_voice_service_defs);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to add SPP Voice service: rc=%d", rc);
        return rc;
    }
    g_voice_server.registered = true;
    g_voice_server.initialized = true;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "SPP Voice server initialized (service registered)");
    return 0;
}

/* Send voice data to BLE client */
int spp_voice_server_send(uint16_t conn_handle, const uint8_t *data, uint16_t len)
{
    VOICE_SERVER_CHECK_INIT();

    if (!data || len == 0) {
        return BLE_HS_EINVAL;
    }

    // Use safe wrapper that ensures mbuf is always freed
    int rc = ble_gatts_send_safe_notify(conn_handle, g_voice_server.val_handle, data, len);
    if (rc != 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to send notification: rc=%d", rc);
        return rc;
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice data sent successfully: len=%d", len);
    return 0;
}

/* Get characteristic value handle */
uint16_t spp_voice_server_get_val_handle(void)
{
    VOICE_SERVER_CHECK_INIT();
    return g_voice_server.val_handle;
}

/**
 * @brief Enable Voice service
 */
void spp_voice_server_enable(void)
{
    int rc;

    g_voice_server.enabled = true;

    /* Request low-latency connection parameters for voice */
    if (g_voice_server.active_conn_handle != 0) {
        struct ble_gap_upd_params params = {
            .itvl_min = 14,
            .itvl_max = 16,
            .latency = 0,
            .supervision_timeout = 400,
            .min_ce_len = 0,
            .max_ce_len = 0,
        };
        rc = ble_gap_update_params(g_voice_server.active_conn_handle, &params);
        if (rc == 0) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice enabled: requested low-latency parameters (itvl=14-16)");
        } else {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice enabled: failed to update params: %d", rc);
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice service enabled");
}

/**
 * @brief Disable Voice service
 */
void spp_voice_server_disable(void)
{
    int rc;

    g_voice_server.enabled = false;

    /* Restore default connection parameters */
    if (g_voice_server.active_conn_handle != 0) {
        struct ble_gap_upd_params params = {
            .itvl_min = 24,          /* 30ms (default) */
            .itvl_max = 40,          /* 50ms (default) */
            .latency = 0,
            .supervision_timeout = 400,
            .min_ce_len = 0,
            .max_ce_len = 0,
        };
        rc = ble_gap_update_params(g_voice_server.active_conn_handle, &params);
        if (rc == 0) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice disabled: restored default parameters (itvl=24-40)");
        } else {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice disabled: failed to restore params: %d", rc);
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice service disabled");
}

/**
 * @brief Check if Voice service is enabled
 */
bool spp_voice_server_is_enabled(void)
{
    return g_voice_server.enabled;
}

/**
 * @brief Clean up voice server state on BLE disconnect
 *
 * This function clears the active connection handle when a BLE device disconnects.
 * This prevents voice data from being sent to a closed connection.
 */
void spp_voice_server_cleanup_on_disconnect(uint16_t conn_handle)
{
    // Only clear if this is the active connection
    if (g_voice_server.active_conn_handle == conn_handle) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
            "Cleaning up voice server state (conn_handle=%d)", conn_handle);
        g_voice_server.active_conn_handle = 0;
    } else if (g_voice_server.active_conn_handle != 0) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
            "Disconnect conn_handle=%d does not match active voice connection=%d, not clearing",
            conn_handle, g_voice_server.active_conn_handle);
    }
}
