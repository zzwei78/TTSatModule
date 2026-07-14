/*
 * tt_module_ota.c - Tiantong Module OTA Update (Streaming Implementation)
 *
 * Real-time streaming: BLE → 128B buffer → XMODEM → TT module.
 * Based on HWA_OTA demo code (tiantong ota doc.docx).
 *
 * Flow:
 *   begin() → prep task (stop MUX, AT+UPDATE, reset, setbaud, loadx, wait 'C')
 *   feed()  → accumulate 128B → XMODEM packet → send via UART
 *   finish()→ flush → EOT → wait "done" → recover TT
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "tt/tt_module_ota.h"
#include "tt/tt_module.h"
#include "system/syslog.h"
#include "tt/tt_hardware.h"
#include "system/sleep_manager.h"

static const char *TAG = "TT_MODULE_OTA";

/* TT UART port (same as normal TT communication) */
#define TT_OTA_UART_PORT      UART_NUM_1

/* XMODEM Protocol Constants */
#define XMODEM_SOH          0x01
#define XMODEM_EOT          0x04
#define XMODEM_ACK          0x06
#define XMODEM_NAK          0x15
#define XMODEM_CAN          0x18
#define XMODEM_EOF          0x1A   /* pad byte for short packets */
#define XMODEM_CRC          0x43   /* 'C' — CRC mode request from receiver */

#define XMODEM_PACKET_SIZE  128
#define XMODEM_PACKET_TOTAL 133    /* SOH + seq + ~seq + 128 data + CRC16(2) */
#define XMODEM_MAX_RETRIES  10

/* Baudrate */
#define TT_UART_BAUD_NORMAL  115200
#define TT_UART_BAUD_HIGH    921600

/* OTA timeouts (ms) */
#define OTA_TIMEOUT_READY      30000   /* Wait for "ready" after reset */
#define OTA_TIMEOUT_BAUD_OK    5000    /* Wait for "baud ok" */
#define OTA_TIMEOUT_CRC        30000   /* Wait for 'C' after loadx */
#define OTA_TIMEOUT_ACK        3000    /* Per-packet ACK timeout */
#define OTA_TIMEOUT_DONE       120000  /* Wait for "done" after EOT */
#define OTA_TIMEOUT_DATA_GAP   60000   /* Max gap between BLE data packets (60s) */

/* ========== OTA Context ========== */

typedef struct {
    /* State */
    volatile tt_ota_state_t state;
    volatile tt_ota_result_t result;
    volatile bool in_progress;
    volatile bool ready_for_data;   /* prep complete, XMODEM 'C' received */
    volatile bool feed_failed;      /* set by feed() on XMODEM error */
    volatile int64_t last_feed_time; /* Timestamp of last feed() call */

    /* Management */
    SemaphoreHandle_t mutex;
    tt_ota_progress_cb_t progress_cb;
    TaskHandle_t prep_task_handle;
    bool initialized;

    /* Data tracking */
    volatile size_t total_size;
    volatile size_t received_size;

    /* XMODEM buffer (only accessed from feed/finish — single context) */
    uint8_t xmodem_buf[XMODEM_PACKET_SIZE];
    uint8_t buf_offset;
    uint8_t packet_num;
} tt_ota_context_t;

static tt_ota_context_t g_tt_ota = {
    .state = TT_OTA_STATE_IDLE,
    .result = TT_OTA_RESULT_OK,
    .in_progress = false,
    .ready_for_data = false,
    .feed_failed = false,
    .last_feed_time = 0,
    .mutex = NULL,
    .progress_cb = NULL,
    .prep_task_handle = NULL,
    .initialized = false,
    .total_size = 0,
    .received_size = 0,
    .buf_offset = 0,
    .packet_num = 1,
};

/* ========== Raw UART Helpers ========== */

static void ota_uart_flush(void)
{
    uart_flush_input(TT_OTA_UART_PORT);
}

static esp_err_t ota_uart_send(const uint8_t *data, size_t len)
{
    int written = uart_write_bytes(TT_OTA_UART_PORT, data, len);
    if (written != (int)len) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "UART write failed: %d/%d", written, (int)len);
        return ESP_FAIL;
    }
    uart_wait_tx_done(TT_OTA_UART_PORT, pdMS_TO_TICKS(1000));
    return ESP_OK;
}

static esp_err_t ota_uart_send_str(const char *str)
{
    return ota_uart_send((const uint8_t *)str, strlen(str));
}

static int ota_uart_read_byte(uint32_t timeout_ms)
{
    uint8_t byte;
    int len = uart_read_bytes(TT_OTA_UART_PORT, &byte, 1, pdMS_TO_TICKS(timeout_ms));
    return (len == 1) ? byte : -1;
}

static esp_err_t ota_uart_wait_string(const char *expected, uint32_t timeout_ms)
{
    size_t exp_len = strlen(expected);
    size_t matched = 0;
    char buf[128];
    int buf_pos = 0;
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < (int64_t)timeout_ms * 1000) {
        int byte = ota_uart_read_byte(100);
        if (byte < 0) continue;

        if (buf_pos < (int)sizeof(buf) - 1) {
            buf[buf_pos++] = (char)byte;
            buf[buf_pos] = '\0';
        } else {
            memmove(buf, buf + 1, buf_pos - 1);
            buf_pos--;
            buf[buf_pos++] = (char)byte;
            buf[buf_pos] = '\0';
        }

        if (buf_pos >= (int)exp_len && strstr(buf, expected)) {
            return ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;
}

/* ========== XMODEM Implementation ========== */

static uint16_t xmodem_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

static esp_err_t xmodem_send_packet_and_wait(const uint8_t data[128], uint8_t pkt_num)
{
    uint8_t pkt[XMODEM_PACKET_TOTAL];
    pkt[0] = XMODEM_SOH;
    pkt[1] = pkt_num;
    pkt[2] = ~pkt_num;
    memcpy(&pkt[3], data, 128);
    uint16_t crc = xmodem_crc16(&pkt[3], 128);
    pkt[131] = (crc >> 8) & 0xFF;
    pkt[132] = crc & 0xFF;

    for (int retry = 0; retry < XMODEM_MAX_RETRIES; retry++) {
        if (ota_uart_send(pkt, XMODEM_PACKET_TOTAL) != ESP_OK) {
            continue;
        }
        int resp = ota_uart_read_byte(OTA_TIMEOUT_ACK);
        if (resp == XMODEM_ACK) return ESP_OK;
        if (resp == XMODEM_CAN) {
            g_tt_ota.result = TT_OTA_RESULT_ERROR_CAN;
            return ESP_FAIL;
        }
        /* NAK or timeout → retry */
        SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "XMODEM pkt %d: resp 0x%02X, retry %d", pkt_num, resp, retry);
    }
    g_tt_ota.result = TT_OTA_RESULT_ERROR_XMODEM;
    return ESP_FAIL;
}

static esp_err_t xmodem_send_eot(void)
{
    uint8_t eot = XMODEM_EOT;
    for (int i = 0; i < 3; i++) {
        ota_uart_send(&eot, 1);
        int resp = ota_uart_read_byte(OTA_TIMEOUT_ACK);
        if (resp == XMODEM_ACK) {
            /* Per XMODEM spec: send second EOT */
            ota_uart_send(&eot, 1);
            ota_uart_read_byte(OTA_TIMEOUT_ACK);
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

/* ========== Recovery ========== */

static void ota_recover_tt(void)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Recovering TT module...");
    uart_set_baudrate(TT_OTA_UART_PORT, TT_UART_BAUD_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(100));

    tt_hw_power_off();
    vTaskDelay(pdMS_TO_TICKS(1000));
    tt_hw_power_on();
    vTaskDelay(pdMS_TO_TICKS(3000));

    tt_module_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    tt_module_init(10);
    tt_module_start();

    sleep_manager_set_inhibit(false);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT module recovered");
}

/* ========== Finish Task (auto-triggered when all data received) ========== */

static void ota_finish_task(void *pv)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Finish task: all data received, finalizing...");
    tt_module_ota_finish();
    vTaskDelete(NULL);
}

/* ========== Prep Task (background) ========== */

static void ota_prep_task(void *pv)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "=== TT OTA Prep Started ===");

    /* Check if TT is already in upgrade mode (interrupted OTA recovery) */
    bool already_in_upgrade_mode = (tt_module_get_state() == TT_STATE_UPGRADE_MODE);

    /* Step 1: Always stop TT communication.
     * This stops the UART AT RX task that would otherwise compete for UART reads. */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Prep 1: Stop TT communication (stops UART RX task)");
    tt_module_stop();
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_set_baudrate(TT_OTA_UART_PORT, TT_UART_BAUD_NORMAL);
    ota_uart_flush();

    if (!already_in_upgrade_mode) {
        /* Step 2: AT+UPDATE (only for normal mode, skip for UPGRADE_MODE) */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Prep 2: Send AT+UPDATE");
        ota_uart_send_str("AT+UPDATE\r\n");
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Waiting 3s for TT to process AT+UPDATE...");
        vTaskDelay(pdMS_TO_TICKS(3000));
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Prep 2: Skip AT+UPDATE (already in UPGRADE_MODE)");
    }

    /* Step 3: Hardware reset (both paths — TT re-enters upgrade mode if needed) */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Prep 3: Hardware reset TT");
    tt_hw_power_off();
    vTaskDelay(pdMS_TO_TICKS(1000));
    tt_hw_power_on();

    /* Wait for "ready" (both paths) */
    g_tt_ota.state = TT_OTA_STATE_WAITING_READY;
    if (g_tt_ota.progress_cb) g_tt_ota.progress_cb(g_tt_ota.state, 5);

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Waiting for 'ready'...");
    if (ota_uart_wait_string("ready", OTA_TIMEOUT_READY) != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Timeout: 'ready'");
        g_tt_ota.result = TT_OTA_RESULT_ERROR_READY_TIMEOUT;
        goto prep_failed;
    }
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Got 'ready'");

    /* Step 4: Switch baudrate (common path) */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Prep 4: setbaud %d", TT_UART_BAUD_HIGH);
    ota_uart_send_str("setbaud 921600\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_set_baudrate(TT_OTA_UART_PORT, TT_UART_BAUD_HIGH);
    ota_uart_flush();

    if (ota_uart_wait_string("baud ok", OTA_TIMEOUT_BAUD_OK) != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Timeout: 'baud ok'");
        g_tt_ota.result = TT_OTA_RESULT_ERROR_BAUD_TIMEOUT;
        goto prep_failed;
    }
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Baudrate OK at %d", TT_UART_BAUD_HIGH);

    /* Step 5: loadx + wait for 'C' */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Prep 5: loadx, wait for 'C'");
    ota_uart_send_str("loadx\r\n");

    bool got_c = false;
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (int64_t)OTA_TIMEOUT_CRC * 1000) {
        int byte = ota_uart_read_byte(1000);
        if (byte == XMODEM_CRC) { got_c = true; break; }
    }
    if (!got_c) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Timeout: 'C'");
        g_tt_ota.result = TT_OTA_RESULT_ERROR_CRC_TIMEOUT;
        goto prep_failed;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Got 'C' — XMODEM ready");
    g_tt_ota.state = TT_OTA_STATE_UPLOADING;
    g_tt_ota.ready_for_data = true;
    g_tt_ota.last_feed_time = esp_timer_get_time();
    if (g_tt_ota.progress_cb) g_tt_ota.progress_cb(g_tt_ota.state, 5);

    /* Start data gap watchdog — aborts if BLE data stops for 60s */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Data gap watchdog armed (%ds)", OTA_TIMEOUT_DATA_GAP / 1000);
    while (g_tt_ota.in_progress && g_tt_ota.ready_for_data) {
        int64_t gap = esp_timer_get_time() - g_tt_ota.last_feed_time;
        if (gap > (int64_t)OTA_TIMEOUT_DATA_GAP * 1000) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Data gap timeout: %lldms since last feed", (int64_t)(gap / 1000));
            g_tt_ota.result = TT_OTA_RESULT_ERROR_DONE_TIMEOUT;
            g_tt_ota.state = TT_OTA_STATE_FAILED;
            g_tt_ota.ready_for_data = false;
            if (g_tt_ota.progress_cb) g_tt_ota.progress_cb(g_tt_ota.state, 0);
            ota_recover_tt();
            g_tt_ota.in_progress = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Prep task done — exit. feed()/finish() handle the rest. */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "=== Prep Complete, ready for data ===");
    g_tt_ota.prep_task_handle = NULL;
    vTaskDelete(NULL);
    return;

prep_failed:
    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "=== Prep FAILED ===");
    g_tt_ota.state = TT_OTA_STATE_FAILED;
    g_tt_ota.ready_for_data = false;
    if (g_tt_ota.progress_cb) g_tt_ota.progress_cb(g_tt_ota.state, 0);

    ota_recover_tt();

    g_tt_ota.in_progress = false;
    g_tt_ota.prep_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ========== Public API ========== */

esp_err_t tt_module_ota_init(void)
{
    if (g_tt_ota.initialized) return ESP_OK;
    g_tt_ota.mutex = xSemaphoreCreateMutex();
    if (!g_tt_ota.mutex) return ESP_ERR_NO_MEM;
    g_tt_ota.initialized = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT OTA initialized");
    return ESP_OK;
}

esp_err_t tt_module_ota_deinit(void)
{
    if (!g_tt_ota.initialized) return ESP_OK;
    if (g_tt_ota.in_progress) tt_module_ota_cancel();
    if (g_tt_ota.mutex) { vSemaphoreDelete(g_tt_ota.mutex); g_tt_ota.mutex = NULL; }
    g_tt_ota.initialized = false;
    return ESP_OK;
}

tt_ota_state_t tt_module_ota_get_state(void) { return g_tt_ota.state; }
tt_ota_result_t tt_module_ota_get_result(void) { return g_tt_ota.result; }
bool tt_module_ota_is_in_progress(void) { return g_tt_ota.in_progress; }
bool tt_module_ota_is_ready(void) { return g_tt_ota.ready_for_data; }

esp_err_t tt_module_ota_begin(size_t total_size, tt_ota_progress_cb_t progress_cb)
{
    if (!g_tt_ota.initialized) return ESP_ERR_INVALID_STATE;
    if (g_tt_ota.in_progress) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(g_tt_ota.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Reset context */
    g_tt_ota.state = TT_OTA_STATE_PREPARING;
    g_tt_ota.result = TT_OTA_RESULT_OK;
    g_tt_ota.in_progress = true;
    g_tt_ota.ready_for_data = false;
    g_tt_ota.feed_failed = false;
    g_tt_ota.total_size = total_size;
    g_tt_ota.received_size = 0;
    g_tt_ota.buf_offset = 0;
    g_tt_ota.packet_num = 1;
    g_tt_ota.progress_cb = progress_cb;

    /* Inhibit sleep */
    sleep_manager_set_inhibit(true);

    /* Start prep task */
    BaseType_t ret = xTaskCreate(ota_prep_task, "tt_ota_prep", 8192, NULL, 5,
                                  &g_tt_ota.prep_task_handle);
    if (ret != pdPASS) {
        g_tt_ota.in_progress = false;
        sleep_manager_set_inhibit(false);
        xSemaphoreGive(g_tt_ota.mutex);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(g_tt_ota.mutex);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT OTA begin: total_size=%d", (int)total_size);
    return ESP_OK;
}

esp_err_t tt_module_ota_feed(const uint8_t *data, size_t len)
{
    if (!g_tt_ota.in_progress) return ESP_ERR_INVALID_STATE;
    if (!g_tt_ota.ready_for_data) return ESP_ERR_INVALID_STATE;
    if (g_tt_ota.feed_failed) return ESP_FAIL;

    size_t offset = 0;
    while (offset < len) {
        /* Fill XMODEM buffer */
        size_t space = XMODEM_PACKET_SIZE - g_tt_ota.buf_offset;
        size_t to_copy = (len - offset < space) ? (len - offset) : space;
        memcpy(&g_tt_ota.xmodem_buf[g_tt_ota.buf_offset], &data[offset], to_copy);
        g_tt_ota.buf_offset += to_copy;
        offset += to_copy;

        /* When buffer is full, send XMODEM packet */
        if (g_tt_ota.buf_offset >= XMODEM_PACKET_SIZE) {
            if (xmodem_send_packet_and_wait(g_tt_ota.xmodem_buf, g_tt_ota.packet_num) != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "XMODEM failed at packet %d", g_tt_ota.packet_num);
                g_tt_ota.feed_failed = true;
                return ESP_FAIL;
            }
            g_tt_ota.packet_num++;
            g_tt_ota.buf_offset = 0;
        }
    }

    g_tt_ota.received_size += len;
    g_tt_ota.last_feed_time = esp_timer_get_time();  /* Refresh data gap watchdog */

    /* Progress: 5% - 85% mapped to data transfer */
    if (g_tt_ota.total_size > 0 && g_tt_ota.progress_cb) {
        int pct = 5 + (int)(g_tt_ota.received_size * 80 / g_tt_ota.total_size);
        if (pct > 85) pct = 85;
        g_tt_ota.progress_cb(TT_OTA_STATE_UPLOADING, pct);
    }

    /* Auto-detect completion: all data received → trigger finish in background */
    if (g_tt_ota.total_size > 0 && g_tt_ota.received_size >= g_tt_ota.total_size) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "All data received (%d/%d bytes), auto-finishing",
                 (int)g_tt_ota.received_size, (int)g_tt_ota.total_size);
        g_tt_ota.ready_for_data = false;  /* Stop accepting more data */
        xTaskCreate(ota_finish_task, "tt_ota_fin", 8192, NULL, 5, NULL);
    }

    return ESP_OK;
}

esp_err_t tt_module_ota_finish(void)
{
    if (!g_tt_ota.in_progress) return ESP_ERR_INVALID_STATE;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Finishing TT OTA: received %d bytes, %d packets",
             (int)g_tt_ota.received_size, g_tt_ota.packet_num - 1);

    /* Flush remaining buffer (pad with EOF) */
    if (g_tt_ota.buf_offset > 0) {
        for (int i = g_tt_ota.buf_offset; i < XMODEM_PACKET_SIZE; i++) {
            g_tt_ota.xmodem_buf[i] = XMODEM_EOF;
        }
        SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Flushing final partial packet (%d bytes data)", g_tt_ota.buf_offset);
        if (xmodem_send_packet_and_wait(g_tt_ota.xmodem_buf, g_tt_ota.packet_num) != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Final packet XMODEM failed");
            goto finish_failed;
        }
        g_tt_ota.buf_offset = 0;
    }

    /* Send EOT */
    if (g_tt_ota.progress_cb) g_tt_ota.progress_cb(TT_OTA_STATE_VERIFYING, 85);
    g_tt_ota.state = TT_OTA_STATE_VERIFYING;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "Sending EOT...");
    if (xmodem_send_eot() != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "EOT failed");
        goto finish_failed;
    }

    /* Wait for "done" or "md5 failed" */
    char resp_buf[128] = {0};
    int buf_pos = 0;
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < (int64_t)OTA_TIMEOUT_DONE * 1000) {
        int byte = ota_uart_read_byte(1000);
        if (byte < 0) {
            /* Update progress during wait */
            if (g_tt_ota.progress_cb) {
                int elapsed_pct = (int)((esp_timer_get_time() - start) * 10 / ((int64_t)OTA_TIMEOUT_DONE * 1000));
                g_tt_ota.progress_cb(TT_OTA_STATE_VERIFYING, 85 + elapsed_pct);
            }
            continue;
        }
        if (buf_pos < (int)sizeof(resp_buf) - 1) {
            resp_buf[buf_pos++] = (char)byte;
            resp_buf[buf_pos] = '\0';
        }

        if (strstr(resp_buf, "done")) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT reports: done");
            goto finish_success;
        }
        if (strstr(resp_buf, "md5 failed")) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "TT reports: md5 failed");
            g_tt_ota.result = TT_OTA_RESULT_ERROR_MD5;
            goto finish_failed;
        }
    }

    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "Timeout waiting for 'done'");
    g_tt_ota.result = TT_OTA_RESULT_ERROR_DONE_TIMEOUT;
    goto finish_failed;

finish_success:
    SYS_LOGI_MODULE(SYS_LOG_MODULE_OTA, TAG, "=== TT OTA SUCCESS ===");
    g_tt_ota.state = TT_OTA_STATE_COMPLETED;
    g_tt_ota.ready_for_data = false;

    if (g_tt_ota.progress_cb) g_tt_ota.progress_cb(g_tt_ota.state, 95);

    ota_recover_tt();

    if (g_tt_ota.progress_cb) g_tt_ota.progress_cb(g_tt_ota.state, 100);
    g_tt_ota.in_progress = false;
    return ESP_OK;

finish_failed:
    SYS_LOGE_MODULE(SYS_LOG_MODULE_OTA, TAG, "=== TT OTA FAILED (result=%d) ===", g_tt_ota.result);
    g_tt_ota.state = TT_OTA_STATE_FAILED;
    g_tt_ota.ready_for_data = false;

    if (g_tt_ota.progress_cb) g_tt_ota.progress_cb(g_tt_ota.state, 0);

    ota_recover_tt();

    g_tt_ota.in_progress = false;
    return ESP_FAIL;
}

esp_err_t tt_module_ota_cancel(void)
{
    if (!g_tt_ota.in_progress) return ESP_ERR_INVALID_STATE;

    SYS_LOGW_MODULE(SYS_LOG_MODULE_OTA, TAG, "Cancelling TT OTA...");
    g_tt_ota.in_progress = false;
    g_tt_ota.ready_for_data = false;
    g_tt_ota.state = TT_OTA_STATE_FAILED;
    g_tt_ota.result = TT_OTA_RESULT_ERROR_CAN;

    if (g_tt_ota.prep_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (g_tt_ota.prep_task_handle) {
            vTaskDelete(g_tt_ota.prep_task_handle);
            g_tt_ota.prep_task_handle = NULL;
        }
    }

    ota_recover_tt();
    return ESP_OK;
}
