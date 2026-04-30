/*
 * sleep_manager.c - Hybrid Sleep Manager for ESP32-S3
 *
 * Implements four-state power management:
 *   ACTIVE      - Normal operation (~60mA)
 *   LIGHT_SLEEP - BLE connected + idle 10s (~240uA)
 *   DEEP_SLEEP  - BLE disconnected + TT off + idle 60s (~7uA)
 *   TEMP_AWAKE  - Deep sleep timer wakeup, BLE advertise 5s
 */

#include <string.h>
#include "system/sleep_manager.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "syslog.h"
#include "host/ble_gap.h"
#include "tt/tt_module.h"
#include "system/power_manage.h"
#include "tt/tt_hardware.h"
#include "bq27220.h"
#include "audio/audiosvc.h"
#include "config/user_params.h"
#include "system/ble_monitor.h"

static const char *TAG = "SLEEP_MGR";

/* ========== RTC Data (survives deep sleep) ========== */

/**
 * @brief RTC Memory Data Structure with CRC Validation
 *
 * All RTC data is grouped into a single struct with CRC-16-CCITT validation.
 * This ensures data integrity after deep sleep wake cycles.
 *
 * CRC-16-CCITT parameters:
 *   - Polynomial: 0x1021
 *   - Initial value: 0xFFFF
 *   - Input/Output reflection: No
 *   - Final XOR: 0x0000
 */
typedef struct __attribute__((packed)) {
    uint32_t boot_count;           /* Number of normal boots */
    uint32_t deep_sleep_count;     /* Number of deep sleep entries */
    uint16_t last_battery_mv;      /* Battery voltage before last deep sleep */
    int64_t  sleep_timestamp;      /* Time when entering deep sleep (us) */
    uint16_t crc16;                /* CRC-16-CCITT of above fields */
    uint32_t magic;                /* Magic number for validation */
} rtc_data_t;

#define RTC_DATA_MAGIC    0x52544344  /* "RTCD" in ASCII - RTC Data */
#define RTC_DATA_VERSION  1

RTC_DATA_ATTR static volatile rtc_data_t rtc_data = {
    .boot_count = 0,
    .deep_sleep_count = 0,
    .last_battery_mv = 0,
    .sleep_timestamp = 0,
    .crc16 = 0,
    .magic = RTC_DATA_MAGIC
};

/* ========== Module State ========== */

static sleep_mode_t g_current_mode = SLEEP_MODE_ACTIVE;
static sleep_wakeup_cause_t g_wakeup_cause = SLEEP_WAKEUP_NONE;
static bool g_initialized = false;
static bool g_inhibit_sleep = false;

/* Idle tracking */
static int64_t g_last_activity_time = 0;   /* esp_timer_get_time() microseconds */

/* State tracking */
static volatile int g_ble_conn_count = 0;
static bool g_tt_powered = false;

/* BLE advertising state for TEMP_AWAKE */
static volatile bool g_ble_advertising = false;

/* Task */
static TaskHandle_t g_sleep_task_handle = NULL;
static volatile bool g_sleep_task_running = false;

/* TEMP_AWAKE timer */
static TimerHandle_t g_temp_awake_timer = NULL;

/* Deep sleep idle counter (seconds, reset when BLE connected or TT on) */
static int32_t g_deep_sleep_idle_sec = 0;

/* Track if we just woke from light sleep (for quick re-entry) */
static bool g_just_woke_from_light_sleep = false;

/* Deferred init flag to prevent double-init */
static volatile bool g_deferred_init_done = false;

/* ========== Internal Functions ========== */

static void enter_light_sleep(void);
static void enter_deep_sleep_internal(void);
static void configure_gpio_for_deep_sleep(void);
static void temp_awake_timer_callback(TimerHandle_t xTimer);
static void deferred_full_init_task(void *pvParameters);

/**
 * @brief Calculate CRC-16-CCITT checksum
 *
 * Standard CRC-16-CCITT implementation with:
 *   - Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
 *   - Initial value: 0xFFFF
 *   - No input/output reflection
 *   - No final XOR
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return CRC-16 checksum
 */
static uint16_t rtc_data_calc_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    size_t i;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Validate RTC data integrity using CRC and magic number
 *
 * @return true if RTC data is valid, false otherwise
 */
static bool rtc_data_validate(void)
{
    const uint8_t *data_ptr = (const uint8_t *)&rtc_data;
    uint16_t calculated_crc;

    /* Check magic number first (quick reject) */
    if (rtc_data.magic != RTC_DATA_MAGIC) {
        SYS_LOGW(TAG, "RTC data magic invalid: 0x%08X (expected 0x%08X)",
                 (unsigned)rtc_data.magic, RTC_DATA_MAGIC);
        return false;
    }

    /* Calculate CRC over all fields EXCEPT crc16 and magic itself */
    calculated_crc = rtc_data_calc_crc16(data_ptr,
                                         offsetof(rtc_data_t, crc16));

    if (calculated_crc != rtc_data.crc16) {
        SYS_LOGW(TAG, "RTC data CRC mismatch: calculated=0x%04X, stored=0x%04X",
                 calculated_crc, rtc_data.crc16);
        return false;
    }

    return true;
}

/**
 * @brief Update RTC data CRC after modifying fields
 *
 * Call this function after modifying any RTC data field to ensure
 * the CRC stays synchronized.
 */
static void rtc_data_update_crc(void)
{
    uint8_t *data_ptr = (uint8_t *)&rtc_data;

    /* Calculate CRC over all fields EXCEPT crc16 and magic */
    rtc_data.crc16 = rtc_data_calc_crc16(data_ptr, offsetof(rtc_data_t, crc16));
    rtc_data.magic = RTC_DATA_MAGIC;
}

/**
 * @brief Reset RTC data to default values
 *
 * Called when RTC data validation fails or on first boot.
 */
static void rtc_data_reset(void)
{
    SYS_LOGW(TAG, "Resetting RTC data to defaults");

    rtc_data.boot_count = 0;
    rtc_data.deep_sleep_count = 0;
    rtc_data.last_battery_mv = 0;
    rtc_data.sleep_timestamp = 0;
    rtc_data.crc16 = 0;
    rtc_data.magic = RTC_DATA_MAGIC;

    /* Calculate initial CRC */
    rtc_data_update_crc();
}

/**
 * @brief Safely check if BLE has any active connections
 *
 * This function performs a dual-check:
 * 1. Checks the cached connection count (fast, but may be stale)
 * 2. Queries NimBLE stack directly (accurate, but slower)
 *
 * @return true if at least one BLE connection exists, false otherwise
 */
static bool is_ble_connected(void)
{
    /* Fast path: check cached count */
    if (g_ble_conn_count > 0) {
        return true;
    }

    /* Slow path: query NimBLE stack directly */
    for (uint16_t h = 0; h < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; h++) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(h, &desc) == 0) {
            /* Connection exists but cache not updated - sync it */
            g_ble_conn_count++;
            SYS_LOGW(TAG, "Sync: Found BLE conn (handle=%d), updated count to %d",
                     h, g_ble_conn_count);
            return true;
        }
    }

    return false;
}

/* ========== Initialization ========== */

esp_err_t sleep_manager_init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    /* ========== RTC Data Validation ========== */
    /* Validate RTC data integrity on every wakeup */
    if (!rtc_data_validate()) {
        SYS_LOGE(TAG, "RTC data validation failed, resetting to defaults");
        rtc_data_reset();
        /* After reset, data should be valid */
        if (!rtc_data_validate()) {
            SYS_LOGE(TAG, "RTC data reset failed - this should not happen!");
            /* Continue anyway with default values */
        }
    } else {
        SYS_LOGI(TAG, "RTC data validated: boot=%u, deep_sleep=%u, batt=%umV",
                 (unsigned)rtc_data.boot_count,
                 (unsigned)rtc_data.deep_sleep_count,
                 rtc_data.last_battery_mv);
    }

    /* ========== Detect Wakeup Cause ========== */
    /* Detect deep sleep wakeup cause (saved for later printing after syslog init) */
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        g_wakeup_cause = SLEEP_WAKEUP_TIMER;
        g_current_mode = SLEEP_MODE_TEMP_AWAKE;
        rtc_data.deep_sleep_count++;
        rtc_data_update_crc();  /* Update CRC after modification */
    } else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        g_wakeup_cause = SLEEP_WAKEUP_GPIO21;
        g_current_mode = SLEEP_MODE_ACTIVE;
        rtc_data.deep_sleep_count++;
        rtc_data_update_crc();  /* Update CRC after modification */
    } else if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        g_wakeup_cause = SLEEP_WAKEUP_NONE;
        g_current_mode = SLEEP_MODE_ACTIVE;
        rtc_data.boot_count++;
        rtc_data_update_crc();  /* Update CRC after modification */
    } else {
        g_wakeup_cause = SLEEP_WAKEUP_OTHER;
        g_current_mode = SLEEP_MODE_ACTIVE;
    }

    /* Initialize activity timestamp to "now" */
    g_last_activity_time = esp_timer_get_time();

    /* Read initial state */
    g_tt_powered = tt_module_is_powered();
    g_ble_conn_count = 0;  /* Will be tracked by notify functions */

    g_initialized = true;
    return ESP_OK;
}

void sleep_manager_print_wakeup_info(void)
{
    if (g_wakeup_cause == SLEEP_WAKEUP_TIMER) {
        SYS_LOGI(TAG, "Deep sleep TIMER wakeup (count=%u, slept=%lld us)",
                 (unsigned)rtc_data.deep_sleep_count,
                 rtc_data.sleep_timestamp > 0 ? (esp_timer_get_time() - rtc_data.sleep_timestamp) : 0);
    } else if (g_wakeup_cause == SLEEP_WAKEUP_GPIO21) {
        SYS_LOGI(TAG, "Deep sleep EXT0 (GPIO21) wakeup (count=%u)",
                 (unsigned)rtc_data.deep_sleep_count);
    } else if (g_wakeup_cause == SLEEP_WAKEUP_NONE) {
        SYS_LOGI(TAG, "Normal boot (boot_count=%u)", (unsigned)rtc_data.boot_count);
    } else {
        SYS_LOGW(TAG, "Unknown wakeup cause: %d", g_wakeup_cause);
    }
}

/* ========== Task Management ========== */

static void sleep_decision_task(void *pvParameters);

esp_err_t sleep_manager_task_start(void)
{
    if (g_sleep_task_handle != NULL) {
        SYS_LOGW(TAG, "Sleep task already running");
        return ESP_OK;
    }

    g_sleep_task_running = true;

    BaseType_t ret = xTaskCreate(
        sleep_decision_task,
        "sleep_decision",
        4096,
        NULL,
        3,      /* Lower priority than BLE/TT tasks */
        &g_sleep_task_handle
    );

    if (ret != pdPASS) {
        SYS_LOGE(TAG, "Failed to create sleep decision task");
        g_sleep_task_running = false;
        return ESP_FAIL;
    }

    SYS_LOGI(TAG, "Sleep decision task started");
    return ESP_OK;
}

esp_err_t sleep_manager_task_stop(void)
{
    if (g_sleep_task_handle == NULL) {
        return ESP_OK;
    }

    g_sleep_task_running = false;

    int timeout = 20;
    while (g_sleep_task_handle != NULL && timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (g_sleep_task_handle != NULL) {
        SYS_LOGW(TAG, "Sleep task stop timeout");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

/* ========== Notification Functions ========== */

void sleep_manager_notify_activity(const char *source)
{
    int64_t old_time = g_last_activity_time;
    g_last_activity_time = esp_timer_get_time();
    g_just_woke_from_light_sleep = false;  /* Real activity, use full idle timeout */
    int32_t elapsed = (int32_t)((g_last_activity_time - old_time) / 1000000);
    SYS_LOGI(TAG, "Activity [%s] (prev=%ds ago)", source ? source : "?", elapsed);
}

void sleep_manager_notify_ble_connected(void)
{
    g_ble_conn_count++;
    g_last_activity_time = esp_timer_get_time();
    g_deep_sleep_idle_sec = 0;  /* Reset deep sleep counter */
    SYS_LOGI(TAG, "BLE connected (count=%d)", g_ble_conn_count);

    /* If in TEMP_AWAKE, cancel timer and transition to ACTIVE */
    if (g_current_mode == SLEEP_MODE_TEMP_AWAKE && g_temp_awake_timer != NULL) {
        /* Attempt to stop the timer with timeout */
        BaseType_t timer_stop_result = xTimerStop(g_temp_awake_timer, pdMS_TO_TICKS(100));

        if (timer_stop_result == pdPASS) {
            SYS_LOGI(TAG, "TEMP_AWAKE timer stopped successfully");
        } else {
            /* Timer might already be executing the callback - race condition detected
             * The callback will see the connection and abort deep sleep */
            SYS_LOGW(TAG, "TEMP_AWAKE timer stop failed (rc=%d), callback in progress?",
                     timer_stop_result);
        }

        g_current_mode = SLEEP_MODE_ACTIVE;
        SYS_LOGI(TAG, "TEMP_AWAKE -> ACTIVE (BLE connected)");

        /* Spawn deferred full initialization task */
        if (!g_deferred_init_done) {
            BaseType_t ret = xTaskCreate(
                deferred_full_init_task,
                "deferred_init",
                8192,
                NULL,
                3,
                NULL
            );
            if (ret != pdPASS) {
                SYS_LOGE(TAG, "Failed to create deferred init task");
            } else {
                SYS_LOGI(TAG, "Deferred init task spawned");
            }
        }
    }
}

void sleep_manager_notify_ble_disconnected(void)
{
    if (g_ble_conn_count > 0) {
        g_ble_conn_count--;
    }
    g_last_activity_time = esp_timer_get_time();
    SYS_LOGI(TAG, "BLE disconnected (count=%d)", g_ble_conn_count);
}

void sleep_manager_notify_tt_powered_on(void)
{
    g_tt_powered = true;
    g_last_activity_time = esp_timer_get_time();
    g_deep_sleep_idle_sec = 0;  /* Reset deep sleep counter */
    SYS_LOGI(TAG, "TT module powered ON");
}

void sleep_manager_notify_tt_powered_off(void)
{
    g_tt_powered = false;
    g_last_activity_time = esp_timer_get_time();
    SYS_LOGI(TAG, "TT module powered OFF");
}

void sleep_manager_set_inhibit(bool inhibit)
{
    g_inhibit_sleep = inhibit;
    if (inhibit) {
        g_last_activity_time = esp_timer_get_time();
    }
    SYS_LOGI(TAG, "Sleep inhibit: %s", inhibit ? "TRUE" : "FALSE");
}

/* ========== Query Functions ========== */

sleep_mode_t sleep_manager_get_mode(void)
{
    return g_current_mode;
}

bool sleep_manager_is_deep_sleep_wakeup(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    return (cause == ESP_SLEEP_WAKEUP_TIMER || cause == ESP_SLEEP_WAKEUP_EXT0);
}

sleep_wakeup_cause_t sleep_manager_get_wakeup_cause(void)
{
    return g_wakeup_cause;
}

bool sleep_manager_is_temp_awake(void)
{
    return (g_current_mode == SLEEP_MODE_TEMP_AWAKE);
}

/* ========== Deferred Full Initialization (TEMP_AWAKE → ACTIVE) ========== */

static void deferred_full_init_task(void *pvParameters)
{
    SYS_LOGI(TAG, "Deferred full init starting (BLE connected during TEMP_AWAKE)");

    /* Step 1: Initialize audio codec service */
    if (audio_svc_init() != 0) {
        SYS_LOGW(TAG, "Deferred: audio_svc_init failed");
    } else {
        SYS_LOGI(TAG, "Deferred: audio service initialized");
    }

    /* Step 2: Initialize TT Module */
    if (user_params_is_tt_manual_off()) {
        SYS_LOGI(TAG, "Deferred: TT module manually off, skipping");
    } else {
        esp_err_t ret = tt_module_init(10);
        if (ret != ESP_OK) {
            SYS_LOGE(TAG, "Deferred: tt_module_init failed: %s", esp_err_to_name(ret));
        } else {
            ret = tt_module_start();
            if (ret != ESP_OK) {
                SYS_LOGE(TAG, "Deferred: tt_module_start failed: %s", esp_err_to_name(ret));
                tt_module_deinit();
            } else {
                SYS_LOGI(TAG, "Deferred: TT module initialized and started");
            }
        }
    }

    /* Step 3: Start Power Monitor Task */
    bq27220_handle_t bq = power_manage_get_bq27220_handle();
    if (bq != NULL) {
        esp_err_t ret = power_manage_task_start();
        if (ret != ESP_OK) {
            SYS_LOGW(TAG, "Deferred: power_manage_task_start failed: %s", esp_err_to_name(ret));
        } else {
            SYS_LOGI(TAG, "Deferred: power monitor task started");
        }
    }

    /* Step 4: Start sleep decision task */
    sleep_manager_task_start();

    /* Step 5: Start BLE Health Monitor */
#if BLE_MONITOR_ENABLED
    esp_err_t ret = ble_monitor_start();
    if (ret != ESP_OK) {
        SYS_LOGW(TAG, "Deferred: ble_monitor_start failed (non-fatal)");
    }
#endif

    g_deferred_init_done = true;
    SYS_LOGI(TAG, "Deferred full init complete - system fully operational");

    vTaskDelete(NULL);
}

/* ========== Sleep Decision Task ========== */

static void sleep_decision_task(void *pvParameters)
{
    SYS_LOGI(TAG, "Decision task running (ble=%d, tt=%d, mode=%d)",
             g_ble_conn_count, g_tt_powered, g_current_mode);

    while (g_sleep_task_running) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (g_inhibit_sleep || g_current_mode == SLEEP_MODE_TEMP_AWAKE) {
            continue;
        }

        /* Check BLE connection directly via NimBLE */
        int ble_count = 0;
        for (uint16_t h = 0; h < 2; h++) {
            struct ble_gap_conn_desc d;
            if (ble_gap_conn_find(h, &d) == 0) {
                ble_count++;
            }
        }

        /* === BLE connected or TT on → reset deep sleep counter, check light sleep === */
        if (ble_count > 0 || g_tt_powered) {
            g_deep_sleep_idle_sec = 0;

            /* Skip light sleep if TT is initializing */
            if (g_tt_powered && tt_module_get_state() == TT_STATE_INITIALIZING) {
                continue;
            }

            int64_t now = esp_timer_get_time();
            int32_t idle_sec = (int32_t)((now - g_last_activity_time) / 1000000);
            int32_t light_threshold = g_just_woke_from_light_sleep
                                     ? SLEEP_LIGHT_REENTER_SEC : SLEEP_LIGHT_IDLE_SEC;

            if (idle_sec >= light_threshold) {
                SYS_LOGI(TAG, "Entering LIGHT_SLEEP (ble=%d, tt=%d, idle=%ds)",
                         ble_count, g_tt_powered, idle_sec);

                enter_light_sleep();

                g_current_mode = SLEEP_MODE_ACTIVE;
                g_just_woke_from_light_sleep = true;
                g_last_activity_time = esp_timer_get_time();

                esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
                if (cause == ESP_SLEEP_WAKEUP_BT) {
                    SYS_LOGI(TAG, "Woke from LIGHT_SLEEP [BLE]");
                } else if (cause == ESP_SLEEP_WAKEUP_GPIO) {
                    SYS_LOGI(TAG, "Woke from LIGHT_SLEEP [GPIO21]");
                } else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
                    SYS_LOGI(TAG, "Woke from LIGHT_SLEEP [TIMER]");
                } else {
                    SYS_LOGI(TAG, "Woke from LIGHT_SLEEP [cause=%d]", cause);
                }
            }
            continue;
        }

        /* === BLE disconnected AND TT off → count up for deep sleep === */
        g_deep_sleep_idle_sec++;

        /* Debug: print every 10s to see why ble_count=0 */
        if (g_deep_sleep_idle_sec % 10 == 0) {
            SYS_LOGI(TAG, "Deep sleep counting: ble_nimble=%d, ble_cache=%d, tt=%d, idle=%ds",
                     ble_count, g_ble_conn_count, g_tt_powered, g_deep_sleep_idle_sec);
        }

        if (g_deep_sleep_idle_sec >= SLEEP_DEEP_IDLE_SEC) {
            SYS_LOGI(TAG, "Entering DEEP_SLEEP / shutdown (ble=%d, tt=%d, idle=%ds)",
                     ble_count, g_tt_powered, g_deep_sleep_idle_sec);

            enter_deep_sleep_internal();
            /* Does not return */
        }
    }

    SYS_LOGI(TAG, "Decision task stopped");
    g_sleep_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ========== Light Sleep ========== */

static void enter_light_sleep(void)
{
    g_current_mode = SLEEP_MODE_LIGHT_SLEEP;

    /* Tell BB "AP is sleeping" — BB will pull GPIO21 LOW if it needs us */
    gpio_set_level(AP_WAKEUP_BB_PIN, 1);

    /* Configure wakeup sources */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_bt_wakeup();
    esp_sleep_enable_gpio_wakeup();
    gpio_wakeup_enable(BB_WAKEUP_AP_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_LIGHT_TIMER_SEC * 1000000ULL);

    /* Enter light sleep - CPU pauses, RAM preserved, BLE stays connected */
    esp_light_sleep_start();

    /* === Code continues here after wakeup === */

    /* Tell BB "AP is awake" */
    gpio_set_level(AP_WAKEUP_BB_PIN, 0);
}

/* ========== Deep Sleep ========== */

static void configure_gpio_for_deep_sleep(void)
{
    /*
     * GPIO1 (TTPWR_EN) and GPIO35 (AP_WAKEUP_BB) are NOT RTC GPIOs on ESP32-S3.
     * Use gpio_set_level + gpio_hold_en instead of rtc_gpio_set_level.
     * GPIO21 (BB_WAKEUP_AP) IS an RTC GPIO — use rtc_gpio functions for wakeup.
     */

    /* TT module power: ensure OFF and hold state during deep sleep */
    gpio_set_level(GPIO_TTPWR_EN, 0);
    gpio_hold_en(GPIO_TTPWR_EN);

    /* AP_WAKEUP_BB_PIN: set HIGH to tell BB "AP is sleeping"
     * BB will pull GPIO21 LOW to wake AP when it has messages */
    gpio_set_level(AP_WAKEUP_BB_PIN, 1);
    gpio_hold_en(AP_WAKEUP_BB_PIN);

    /* BB_WAKEUP_AP_PIN: pull-up (HIGH = BB sleeping, we wake on LOW) */
    rtc_gpio_pullup_en(BB_WAKEUP_AP_PIN);
    rtc_gpio_pulldown_dis(BB_WAKEUP_AP_PIN);

    /* Hold GPIO states during deep sleep */
    gpio_deep_sleep_hold_en();
}

static void enter_deep_sleep_internal(void)
{
    g_current_mode = SLEEP_MODE_DEEP_SLEEP;

    /* ========== Save state to RTC memory with CRC update ========== */
    rtc_data.sleep_timestamp = esp_timer_get_time();

    /* Read battery voltage BEFORE stopping power tasks (need I2C alive) */
    bq27220_handle_t bq = power_manage_get_bq27220_handle();
    if (bq != NULL) {
        rtc_data.last_battery_mv = bq27220_get_voltage(bq);
    } else {
        rtc_data.last_battery_mv = 0;  /* No battery data available */
    }

    /* Update CRC after modifying RTC data */
    rtc_data_update_crc();

    SYS_LOGI(TAG, "RTC data saved: boot=%u, ds=%u, batt=%umV, crc=0x%04X",
             (unsigned)rtc_data.boot_count,
             (unsigned)rtc_data.deep_sleep_count,
             rtc_data.last_battery_mv,
             rtc_data.crc16);

    /* Stop power monitor tasks */
    power_manage_task_stop();

    /* NOTE: Do NOT call sleep_manager_task_stop() here — we ARE the sleep task.
     * Calling it would deadlock (waiting for ourselves to exit).
     * Deep sleep resets the chip, so graceful task cleanup is unnecessary.
     */

    /* Ensure TT module is fully off */
    if (tt_module_is_powered()) {
        tt_module_user_power_off();
    }

    /* Configure GPIO for minimum leakage */
    configure_gpio_for_deep_sleep();

    /* Configure wakeup sources */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    /* GPIO21 wakeup only when TT module is powered (BB alive and can signal us)
     * When TT is off, BB is off too — no point listening on GPIO21 */
    if (g_tt_powered) {
        esp_sleep_enable_ext0_wakeup(BB_WAKEUP_AP_PIN, 0);
    }

    /* Periodic timer wakeup for BLE advertising */
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_DEEP_TIMER_SEC * 1000000ULL);

    SYS_LOGI(TAG, "Entering DEEP_SLEEP (battery=%umV)", rtc_data.last_battery_mv);

    /* Enter deep sleep - does not return, chip resets on wakeup */
    esp_deep_sleep_start();
}

void sleep_manager_enter_deep_sleep(void)
{
    enter_deep_sleep_internal();
}

/* ========== TEMP_AWAKE Timer ========== */

/**
 * @brief TEMP_AWAKE timer callback with race condition protection
 *
 * This callback is called when the TEMP_AWAKE timer expires. Before entering
 * deep sleep, we must check:
 * 1. BLE connection status - if connected or connecting, abort deep sleep
 * 2. Deferred init task status - if running, wait for it to complete
 *
 * Race conditions prevented:
 * - Timer expires while BLE connection is being established
 * - Timer expires while deferred init task is initializing services
 * - Connection event arrives just before timer check
 */
static void temp_awake_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    SYS_LOGI(TAG, "TEMP_AWAKE timer expired, checking conditions before deep sleep...");

    /* ========== Safety Check 1: Check if deferred init is running ========== */
    if (g_deferred_init_done) {
        SYS_LOGW(TAG, "TEMP_AWAKE: Deferred init already completed, should not be in TEMP_AWAKE mode");
        /* This is unexpected - if deferred init completed, we should have transitioned
         * out of TEMP_AWAKE mode. Log warning but continue with checks. */
    }

    /* ========== Safety Check 2: Check BLE connection via helper function ========== */
    if (is_ble_connected()) {
        /* BLE is connected - abort deep sleep, normal initialization will proceed */
        SYS_LOGI(TAG, "TEMP_AWAKE: BLE connected, aborting deep sleep");
        /* Mode transition will be handled by sleep_manager_notify_ble_connected() */
        return;
    }

    /* ========== Safety Check 3: Check sleep inhibit flag ========== */
    if (g_inhibit_sleep) {
        SYS_LOGW(TAG, "TEMP_AWAKE: Sleep inhibited, aborting deep sleep");
        return;
    }

    /* ========== Safety Check 4: Check current mode ========== */
    /* If we're no longer in TEMP_AWAKE mode, something else changed state */
    if (g_current_mode != SLEEP_MODE_TEMP_AWAKE) {
        SYS_LOGW(TAG, "TEMP_AWAKE: Mode changed to %d, aborting deep sleep", g_current_mode);
        return;
    }

    /* ========== All checks passed - safe to enter deep sleep ========== */
    SYS_LOGI(TAG, "TEMP_AWAKE: All checks passed (ble=%d, inhibit=%d, mode=%d), entering deep sleep",
             g_ble_conn_count, g_inhibit_sleep, g_current_mode);
    sleep_manager_enter_deep_sleep();
}

void sleep_manager_temp_awake_timeout(void)
{
    sleep_manager_enter_deep_sleep();
}

/**
 * @brief Create and start the TEMP_AWAKE timer
 *
 * Called from app_main() after minimal init during timer wakeup.
 */
esp_err_t sleep_manager_start_temp_awake_timer(void)
{
    if (g_temp_awake_timer != NULL) {
        xTimerDelete(g_temp_awake_timer, 0);
    }

    g_temp_awake_timer = xTimerCreate(
        "temp_awake",
        pdMS_TO_TICKS(SLEEP_TEMP_AWAKE_SEC * 1000),
        pdFALSE,   /* One-shot */
        NULL,
        temp_awake_timer_callback
    );

    if (g_temp_awake_timer == NULL) {
        SYS_LOGE(TAG, "Failed to create TEMP_AWAKE timer");
        return ESP_FAIL;
    }

    xTimerStart(g_temp_awake_timer, 0);
    SYS_LOGI(TAG, "TEMP_AWAKE timer started (%ds)", SLEEP_TEMP_AWAKE_SEC);
    return ESP_OK;
}
