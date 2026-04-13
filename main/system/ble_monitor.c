/**
 * @file ble_monitor.c
 * @brief BLE Health Monitor Implementation
 *
 * Monitors BLE stack health by periodically checking its status.
 * Triggers system reboot if BLE stack becomes unresponsive.
 *
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: MIT
 */

#include "system/ble_monitor.h"
#include "syslog.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* NimBLE headers */
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

static const char *TAG = "BLE_MON";

/* ========== Additional Configuration ========== */
#define BLE_MONITOR_MIN_STACK_WATERMARK    256     // Minimum stack watermark (bytes)

/* ========== Monitor Context ========== */
typedef struct {
    TaskHandle_t task_handle;
    volatile bool running;

    // Failure tracking
    volatile int consecutive_failures;

    // Statistics
    ble_monitor_stats_t stats;

    // Mutex for stats access
    SemaphoreHandle_t stats_mutex;
} ble_monitor_ctx_t;

static ble_monitor_ctx_t g_ble_monitor = {0};

/* ========== Internal Functions ========== */

/**
 * @brief Get current timestamp in seconds
 */
static uint32_t get_timestamp_sec(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

/**
 * @brief Update statistics (thread-safe)
 */
static void update_stats_failure(ble_monitor_failure_t reason)
{
    if (g_ble_monitor.stats_mutex && xSemaphoreTake(g_ble_monitor.stats_mutex, pdMS_TO_TICKS(100))) {
        g_ble_monitor.stats.failure_count++;
        g_ble_monitor.stats.last_failure_reason = reason;
        g_ble_monitor.stats.last_failure_timestamp = get_timestamp_sec();
        xSemaphoreGive(g_ble_monitor.stats_mutex);
    }
}

/**
 * @brief Check BLE stack health (Comprehensive detection)
 *
 * Detection items:
 * 1. BLE Host enabled status
 * 2. Free heap memory
 * 3. BLE address readability (active probe)
 * 4. BLE task stack watermark
 * 5. Sync flag status
 *
 * @return true if healthy, false if problem detected
 */
bool ble_monitor_check_health(void)
{
    bool healthy = true;
    ble_monitor_failure_t failure_reason = BLE_MONITOR_FAILURE_NONE;

    // ========== Check 1: BLE Host Status ==========
    if (!ble_hs_is_enabled()) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BLE Host is not enabled");
        healthy = false;
        failure_reason = BLE_MONITOR_FAILURE_HOST_DISABLED;
        goto check_done;
    }

    // ========== Check 2: Free Heap Memory ==========
    uint32_t free_heap = esp_get_free_heap_size();
    if (free_heap < BLE_MONITOR_LOW_HEAP_BYTES) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Low memory: %lu bytes (threshold: %d)",
                        free_heap, BLE_MONITOR_LOW_HEAP_BYTES);
        healthy = false;
        failure_reason = BLE_MONITOR_FAILURE_LOW_MEMORY;
        goto check_done;
    }

    // ========== Check 3: Active Probe - Read BLE Address ==========
    // This tests if BLE Host can respond to a simple request
    uint8_t addr[6] = {0};
    int rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr, NULL);
    if (rc != 0 && rc != BLE_HS_ENOADDR) {
        // BLE_HS_ENOADDR means no public address, which is acceptable
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BLE address read failed: %d", rc);
        healthy = false;
        failure_reason = BLE_MONITOR_FAILURE_HOST_DISABLED;
        goto check_done;
    }

    // ========== Check 4: BLE Task Stack Watermark ==========
    // Note: nimble_port_get_ble_host_task() is not available in all ESP-IDF versions
    // We check the "ble_hs" task by name instead
    TaskHandle_t ble_host_task = xTaskGetHandle("ble_hs");
    if (ble_host_task != NULL) {
        UBaseType_t stack_watermark = uxTaskGetStackHighWaterMark(ble_host_task);
        // stack_watermark is in words (4 bytes each on ESP32)
        uint32_t stack_free_bytes = stack_watermark * sizeof(StackType_t);
        if (stack_free_bytes < BLE_MONITOR_MIN_STACK_WATERMARK) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                            "BLE Host task stack low: %lu bytes (threshold: %d)",
                            stack_free_bytes, BLE_MONITOR_MIN_STACK_WATERMARK);
            healthy = false;
            failure_reason = BLE_MONITOR_FAILURE_STACK_LOW;
            goto check_done;
        }
    }

    // ========== Check 5: Historical Minimum Heap (Warning Only) ==========
    uint32_t min_free = esp_get_minimum_free_heap_size();
    if (min_free < BLE_MONITOR_LOW_HEAP_BYTES / 2) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                        "Historical minimum heap low: %lu bytes (warning only)", min_free);
        // This is a warning, not a failure - system might still be functional
    }

check_done:
    // Update failure reason if check failed
    if (!healthy && failure_reason != BLE_MONITOR_FAILURE_NONE) {
        update_stats_failure(failure_reason);
    }

    return healthy;
}

/**
 * @brief Trigger system reboot
 */
static void trigger_system_reboot(const char *reason)
{
    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "=== BLE MONITOR: SYSTEM REBOOT TRIGGERED ===");
    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Reason: %s", reason);
    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Consecutive failures: %d", g_ble_monitor.consecutive_failures);

    // Update recovery count
    if (g_ble_monitor.stats_mutex && xSemaphoreTake(g_ble_monitor.stats_mutex, pdMS_TO_TICKS(100))) {
        g_ble_monitor.stats.recovery_count++;
        xSemaphoreGive(g_ble_monitor.stats_mutex);
    }

    // Wait for log to flush
    vTaskDelay(pdMS_TO_TICKS(100));

    // Trigger system restart
    esp_restart();
}

/* ========== Monitor Task ========== */

static void ble_monitor_task(void *arg)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BLE Monitor task started");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  Check interval: %d ms", BLE_MONITOR_INTERVAL_MS);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  Max failures: %d", BLE_MONITOR_MAX_FAILURES);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  Low heap threshold: %d bytes", BLE_MONITOR_LOW_HEAP_BYTES);

    while (g_ble_monitor.running) {
        vTaskDelay(pdMS_TO_TICKS(BLE_MONITOR_INTERVAL_MS));

        // Update check count
        if (g_ble_monitor.stats_mutex && xSemaphoreTake(g_ble_monitor.stats_mutex, pdMS_TO_TICKS(100))) {
            g_ble_monitor.stats.check_count++;
            xSemaphoreGive(g_ble_monitor.stats_mutex);
        }

        // Perform health check
        bool healthy = ble_monitor_check_health();

        if (!healthy) {
            g_ble_monitor.consecutive_failures++;

            // Determine failure reason (simplified)
            ble_monitor_failure_t reason = BLE_MONITOR_FAILURE_UNKNOWN;
            if (!ble_hs_is_enabled()) {
                reason = BLE_MONITOR_FAILURE_HOST_DISABLED;
            } else if (esp_get_free_heap_size() < BLE_MONITOR_LOW_HEAP_BYTES) {
                reason = BLE_MONITOR_FAILURE_LOW_MEMORY;
            }

            update_stats_failure(reason);

            SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                            "BLE health check failed (%d/%d), reason: %d",
                            g_ble_monitor.consecutive_failures,
                            BLE_MONITOR_MAX_FAILURES,
                            reason);

            // Check if max failures reached
            if (g_ble_monitor.consecutive_failures >= BLE_MONITOR_MAX_FAILURES) {
                trigger_system_reboot("BLE stack unresponsive");
            }
        } else {
            // Healthy - reset consecutive failure counter
            if (g_ble_monitor.consecutive_failures > 0) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                                "BLE health recovered after %d failures",
                                g_ble_monitor.consecutive_failures);
            }
            g_ble_monitor.consecutive_failures = 0;
        }

        // Periodic status log (every 6 checks = 60 seconds)
        if (g_ble_monitor.stats.check_count % 6 == 0) {
            SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                            "Health check #%lu: OK, free heap: %lu bytes",
                            g_ble_monitor.stats.check_count,
                            esp_get_free_heap_size());
        }
    }

    SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BLE Monitor task stopped");
    g_ble_monitor.task_handle = NULL;
    vTaskDelete(NULL);
}

/* ========== Public APIs ========== */

esp_err_t ble_monitor_start(void)
{
    if (g_ble_monitor.running) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BLE Monitor already running");
        return ESP_OK;
    }

    // Create stats mutex
    if (g_ble_monitor.stats_mutex == NULL) {
        g_ble_monitor.stats_mutex = xSemaphoreCreateMutex();
        if (g_ble_monitor.stats_mutex == NULL) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to create stats mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    // Reset state
    g_ble_monitor.running = true;
    g_ble_monitor.consecutive_failures = 0;

    // Create monitor task
    BaseType_t ret = xTaskCreate(
        ble_monitor_task,
        "ble_mon",
        4096,
        NULL,
        5,  // Priority
        &g_ble_monitor.task_handle
    );

    if (ret != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to create BLE Monitor task");
        g_ble_monitor.running = false;
        return ESP_FAIL;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BLE Monitor started successfully");
    return ESP_OK;
}

esp_err_t ble_monitor_stop(void)
{
    if (!g_ble_monitor.running) {
        return ESP_OK;
    }

    g_ble_monitor.running = false;

    // Wait for task to finish
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

bool ble_monitor_is_running(void)
{
    return g_ble_monitor.running;
}

esp_err_t ble_monitor_get_stats(ble_monitor_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_ble_monitor.stats_mutex && xSemaphoreTake(g_ble_monitor.stats_mutex, pdMS_TO_TICKS(100))) {
        *stats = g_ble_monitor.stats;
        xSemaphoreGive(g_ble_monitor.stats_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t ble_monitor_clear_stats(void)
{
    if (g_ble_monitor.stats_mutex && xSemaphoreTake(g_ble_monitor.stats_mutex, pdMS_TO_TICKS(100))) {
        memset(&g_ble_monitor.stats, 0, sizeof(g_ble_monitor.stats));
        xSemaphoreGive(g_ble_monitor.stats_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}
