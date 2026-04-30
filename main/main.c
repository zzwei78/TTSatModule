/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "esp_partition.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble/ble_gatt_server.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio/audiosvc.h"
#include "audio/voice_packet_handler.h"
#include "system/syslog.h"
#include "tt/tt_module.h"
#include "system/power_manage.h"
#include "system/ble_monitor.h"
#include "system/sleep_manager.h"
#include "config/user_params.h"
#include "bq27220.h"

static const char *TAG = "MAIN";

/**
 * @brief Memory monitor task - prints system memory info every 60 seconds
 */
static void memory_monitor_task(void *pvParameters);

/**
 * @brief Get factory partition (user partition)
 *
 * @return Pointer to partition or NULL if not found
 */
const esp_partition_t *get_factory_partition(void)
{
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        "user"
    );
    if (!partition) {
        ESP_LOGE(TAG, "Factory partition not found!");
        return NULL;
    }
    return partition;
}

/**
 * @brief Write to factory partition
 *
 * @param data Pointer to buffer containing data to write
 * @param len Data length (bytes)
 * @param offset Offset within partition
 * @return esp_err_t
 */
esp_err_t factory_flash_write(const uint8_t *data, size_t len, size_t offset)
{
    const esp_partition_t *partition = get_factory_partition();
    if (!partition) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Erase flash sectors (4KB aligned) */
    size_t sector_size = 4096;
    size_t start_addr = partition->address + offset;
    size_t erase_addr = start_addr & ~(sector_size - 1);
    size_t erase_size = ((len + (start_addr - erase_addr) + sector_size - 1) / sector_size) * sector_size;

    ESP_LOGI(TAG, "Erase flash: addr=0x%08x, size=0x%08x", erase_addr, erase_size);
    esp_err_t err = esp_partition_erase_range(partition, erase_addr - partition->address, erase_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erase failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Write flash: addr=0x%08x, len=%d", start_addr, len);
    err = esp_partition_write(partition, offset, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write failed: %d", err);
    }
    return err;
}

/**
 * @brief Read from factory partition
 *
 * @param data Pointer to receiving buffer
 * @param len Data length to read (bytes)
 * @param offset Offset within partition
 * @return esp_err_t
 */
esp_err_t factory_flash_read(uint8_t *data, size_t len, size_t offset)
{
    const esp_partition_t *partition = get_factory_partition();
    if (!partition) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if read length exceeds remaining partition space */
    if (offset + len > partition->size) {
        ESP_LOGE(TAG, "Read would exceed partition size! offset=0x%08x, len=0x%08x, partition_size=0x%08x",
                 offset, len, partition->size);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Read flash: offset=0x%08x, len=%d", offset, len);
    esp_err_t err = esp_partition_read(partition, offset, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %d", err);
    }
    return err;
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting application...");

    /* Initialize NVS — used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize sleep manager (detects deep sleep wakeup cause) */
    sleep_manager_init();

    /* Initialize user parameters (NVS storage for user preferences) */
    ret = user_params_init();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to initialize user params: %s", esp_err_to_name(ret));
        /* Continue anyway, user preferences will use defaults */
    } else {
        const user_params_t *params = user_params_get();
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "User params loaded (flags: 0x%02X, low_batt_thresh: %umV)",
                        params ? params->flags : 0,
                        params ? params->low_battery_threshold_mv : USER_PARAMS_DEFAULT_LOW_BATT_MV);
    }

    /* Initialize system log module */
    ret = syslog_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system log: %d", ret);
        return;
    }
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "System log initialized");

    /* Print wakeup source (detected earlier, now syslog is ready) */
    sleep_manager_print_wakeup_info();

    /* ========== TEMP_AWAKE fast path (timer wakeup) ========== */
    if (sleep_manager_is_temp_awake()) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "=== TEMP_AWAKE: minimal init (BLE advertise only) ===");

        /* Minimal init: just power management + BLE */
        ret = power_manage_init();
        if (ret != ESP_OK) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power management init failed: %s", esp_err_to_name(ret));
        }

        ret = ble_gatt_server_init();
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to initialize BLE GATT server: %d", ret);
            return;
        }
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BLE GATT server initialized (TEMP_AWAKE)");

        /* Start temp_awake timer — will re-enter deep sleep if no BLE connection */
        sleep_manager_start_temp_awake_timer();

        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "=== TEMP_AWAKE ready, advertising for %ds ===",
                        SLEEP_TEMP_AWAKE_SEC);
        return;
    }

    /* ========== Full initialization path (normal boot or GPIO21 wakeup) ========== */

    /* Initialize audio codec service (AMRNB encode/decode) */
    ret = audio_svc_init();
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to initialize audio service: %d", ret);
        /* Continue anyway, audio may not be critical for basic functionality */
    } else {
        ESP_LOGI(TAG, "Audio service initialized");
    }

    /*
     * Initialization order:
     * 1. Power management (I2C, IP5561, BQ27220) - check battery voltage
     * 2. TT module (UART, GPIO) - satellite communication hardware
     * 3. Power monitor task - automatic TT module power control
     * 4. BLE GATT server - communication interface (depends on TT module for callbacks)
     */

    /* Step 1: Initialize Power Management (I2C bus, IP5561, BQ27220) */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[1/4] Initializing power management...");
    ret = power_manage_init();
    if (ret != ESP_OK) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power management init failed: %s, continuing without power monitoring",
                 esp_err_to_name(ret));
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power management initialized");
    }

    /* Step 2: Initialize TT Module (phone call has highest priority, no voltage check) */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[2/4] Initializing TT module...");
    bq27220_handle_t bq_handle = power_manage_get_bq27220_handle();

    /* Log battery voltage if available */
    if (bq_handle != NULL) {
        uint16_t battery_voltage = bq27220_get_voltage(bq_handle);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Battery voltage: %u mV", battery_voltage);
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BQ27220 not available");
    }

    /* Check if user manually turned off TT module */
    if (user_params_is_tt_manual_off()) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "TT module was manually closed by user, skipping auto-start");
    } else {
        /* Start TT module directly (phone call priority is highest) */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Starting TT module...");

        ret = tt_module_init(10);  // 10 event queue size
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to initialize TT module: %d", ret);
        } else {
            ret = tt_module_start();
            if (ret != ESP_OK) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to start TT module: %d", ret);
                tt_module_deinit();
            } else {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "TT module initialized and started");
            }
        }
    }

    /* Step 3: Start Power Monitor Task (handles automatic TT module power control) */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[3/4] Starting power monitor task...");
    if (bq_handle != NULL) {
        ret = power_manage_task_start();
        if (ret != ESP_OK) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power monitor task start failed: %s", esp_err_to_name(ret));
        } else {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power monitor task started");
        }
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Skipping power monitor task (no BQ27220)");
    }

    /* Step 4: Initialize BLE GATT Server (depends on TT module for AT callbacks) */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[4/4] Initializing BLE GATT server...");
    ret = ble_gatt_server_init();
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to initialize BLE GATT server: %d", ret);
        return;
    }
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BLE GATT server initialized");

    /* Start sleep manager decision task */
    sleep_manager_task_start();

    /* Step 5: Start BLE Health Monitor */
#if BLE_MONITOR_ENABLED
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[5/5] Starting BLE Health Monitor...");
    ret = ble_monitor_start();
    if (ret != ESP_OK) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to start BLE Monitor (non-fatal)");
    }
#else
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[5/5] BLE Health Monitor disabled");
#endif

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "=== System initialization complete ===");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Ready to accept BLE connections and commands");

    // Create memory monitor task (increased stack size to prevent overflow)
    //xTaskCreate(memory_monitor_task, "mem_monitor", 4096, NULL, 1, NULL);
}

/**
 * @brief Memory monitor task - prints system memory info every 60 seconds
 */
static void memory_monitor_task(void *pvParameters)
{
    while (1) {
        // Get heap information
        size_t free_heap = esp_get_free_heap_size();
        size_t min_free_heap = esp_get_minimum_free_heap_size();

        // Print memory information
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "=== Memory Info ===");
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  Free Heap: %u bytes (%.1f KB)", free_heap, free_heap / 1024.0);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  Min Free Heap: %u bytes (%.1f KB)", min_free_heap, min_free_heap / 1024.0);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  Largest Free Block: %u bytes", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "==================");

        // Wait 60 seconds
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
