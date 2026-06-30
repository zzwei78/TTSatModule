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
#include <stdio.h>
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
#include "fuel_gauge.h"
#include "audio/audiosvc.h"
#include "config/user_params.h"
#include "system/ble_monitor.h"
#include "ble/ble_gatt_server.h"
#include "ble/spp_voice_server.h"

static const char *TAG = "SLEEP_MGR";

/* GPIO5 = IP5561 INT pin. Driven HIGH by IP5561 when USB/VBUS activates it.
 * Used as ext0 deep sleep wakeup source for USB insertion detection. */
#define USB_WAKEUP_GPIO    GPIO_NUM_5

/* ========== Auto收网 (No Network → Power Off TT) ========== */
#define TT_NO_NET_CHECK_SEC        60    /* Query CREG every 60s */
#define TT_NO_NET_TIMEOUT_SEC      600   /* 10 min → auto power off TT */

/* ========== Pwrkey Monitor (V2 only) ========== */
#define PWRKEY_POLL_MS             20    /* Poll interval */
#define PWRKEY_DEBOUNCE_MS         50    /* Min press to register (debounce) */
#define PWRKEY_SHORT_THRESHOLD_MS  2000  /* < 2s = short press */
#define PWRKEY_LONG_THRESHOLD_MS   5000  /* ≥ 5s while held = force deep sleep */

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
static TaskHandle_t g_pwrkey_task_handle = NULL;
static volatile bool g_pwrkey_task_running = false;

/* TEMP_AWAKE timer */
static TimerHandle_t g_temp_awake_timer = NULL;

/* Deep sleep idle counter (seconds, reset when BLE connected or TT on) */
static int32_t g_deep_sleep_idle_sec = 0;

/* Track if we just woke from light sleep (for quick re-entry) */
static bool g_just_woke_from_light_sleep = false;

/* Auto收网: TT no-network duration tracking */
static uint32_t g_tt_no_net_sec = 0;      /* Seconds since last network registration */
static int g_tt_net_check_cnt = 0;        /* Counter for periodic CREG check */
static bool g_tt_was_registered = true;   /* Last known registration state (for change logging) */

/* Deferred init flag to prevent double-init */
static volatile bool g_deferred_init_done = false;

/* ========== Internal Functions ========== */

static void enter_light_sleep(void);
static void enter_deep_sleep_internal(void);
static void configure_gpio_for_deep_sleep(void);
static void temp_awake_timer_callback(TimerHandle_t xTimer);
static void deferred_full_init_task(void *pvParameters);
static void pwrkey_init_and_start(void);

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
    /* Primary: callback-maintained cache (reliable — driven by GAP events) */
    if (g_ble_conn_count > 0) {
        return true;
    }

    /* Fallback: query NimBLE in case a connect callback was missed.
     * Only syncs UP (never down) because ble_gap_conn_find can false-negative
     * on valid connections with handles outside [0, MAX) after reconnects.
     * Stale entries (cache > actual) are handled by ble_conn_manager self-healing. */
    int found = 0;
    for (uint16_t h = 0; h < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; h++) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(h, &desc) == 0) {
            found++;
        }
    }
    if (found > 0) {
        g_ble_conn_count = found;
        SYS_LOGW(TAG, "BLE conn sync: cache 0 → %d (nimble fallback)", found);
        return true;
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
        /* ext0: IP5561 INT (GPIO5) went HIGH → USB inserted */
        g_wakeup_cause = SLEEP_WAKEUP_USB;
        g_current_mode = SLEEP_MODE_ACTIVE;
        rtc_data.deep_sleep_count++;
        rtc_data_update_crc();
    } else if (cause == ESP_SLEEP_WAKEUP_EXT1) {
        /* ext1: check which pin triggered wakeup */
        uint64_t ext1_status = esp_sleep_get_ext1_wakeup_status();
        rtc_data.deep_sleep_count++;
        rtc_data_update_crc();
#ifdef SUPPORT_HARDWARE_V2
        if (ext1_status & (1ULL << GPIO_PWRKEY)) {
            g_wakeup_cause = SLEEP_WAKEUP_PWRKEY;
            g_current_mode = SLEEP_MODE_ACTIVE;
        } else
#endif
        if (ext1_status & (1ULL << BB_WAKEUP_AP_PIN)) {
            g_wakeup_cause = SLEEP_WAKEUP_GPIO21;
            g_current_mode = SLEEP_MODE_ACTIVE;
        } else {
            g_wakeup_cause = SLEEP_WAKEUP_OTHER;
            g_current_mode = SLEEP_MODE_ACTIVE;
        }
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

    /* Start pwrkey monitor (V2 only, runs from boot) */
    pwrkey_init_and_start();

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
        SYS_LOGI(TAG, "Deep sleep GPIO21 wakeup (count=%u)",
                 (unsigned)rtc_data.deep_sleep_count);
    } else if (g_wakeup_cause == SLEEP_WAKEUP_PWRKEY) {
        SYS_LOGI(TAG, "Deep sleep PWRKEY (GPIO9) wakeup (count=%u)",
                 (unsigned)rtc_data.deep_sleep_count);
    } else if (g_wakeup_cause == SLEEP_WAKEUP_USB) {
        SYS_LOGI(TAG, "Deep sleep USB wakeup via IP5561 INT GPIO%d (count=%u)",
                 USB_WAKEUP_GPIO, (unsigned)rtc_data.deep_sleep_count);
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

void sleep_manager_refresh_idle(void)
{
    g_last_activity_time = esp_timer_get_time();
    g_just_woke_from_light_sleep = false;
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
    g_tt_no_net_sec = 0;        /* Reset auto收网 counter */
    g_tt_net_check_cnt = 0;
    g_tt_was_registered = true;
    SYS_LOGI(TAG, "TT module powered ON");
}

void sleep_manager_notify_tt_powered_off(void)
{
    g_tt_powered = false;
    g_last_activity_time = esp_timer_get_time();
    g_tt_no_net_sec = 0;        /* Reset auto收网 counter */
    g_tt_net_check_cnt = 0;
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
    fuel_gauge_handle_t bq = power_manage_get_fuel_gauge_handle();
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

        /* Check charging status — skip all sleep modes while charging */
        {
            uint8_t chg_flags = 0;
            bool is_charging = false;
            if (power_manage_get_charging_status(&chg_flags) == ESP_OK) {
                is_charging = (chg_flags & 0x04) != 0;   /* bit2: charging */
            }
            bool is_wpc = power_manage_is_wpc_charging();
            if (is_charging || is_wpc) {
                g_deep_sleep_idle_sec = 0;  /* reset counter */
                continue;  /* skip sleep while charging */
            }
        }

        /* Check BLE connection directly via NimBLE (for diagnostics only) */
        int ble_count = 0;
        for (uint16_t h = 0; h < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; h++) {
            struct ble_gap_conn_desc d;
            if (ble_gap_conn_find(h, &d) == 0) {
                ble_count++;
            }
        }

        /* Correct cache if NimBLE consistently shows fewer connections.
         * Only correct DOWN when nimble > 0 (avoid false-negative clearing to 0).
         * This fixes stale entries from missed disconnect callbacks. */
        if (ble_count != g_ble_conn_count) {
            if (ble_count > 0 && ble_count < g_ble_conn_count) {
                /* Safe correction: nimble sees real connections, just fewer than cache */
                static int s_correct_log_cnt = 0;
                if (s_correct_log_cnt++ % 30 == 0) {  /* Log once per 30s */
                    SYS_LOGW(TAG, "BLE cache corrected: %d → %d (nimble ground truth)",
                             g_ble_conn_count, ble_count);
                }
                g_ble_conn_count = ble_count;
            } else if (ble_count == 0 && g_ble_conn_count > 0) {
                /* NimBLE says 0 but cache says >0 — don't correct (could be false negative).
                 * Log sparingly. */
                static int s_mismatch_log_cnt = 0;
                if (s_mismatch_log_cnt++ % 60 == 0) {  /* Log once per 60s */
                    SYS_LOGW(TAG, "BLE mismatch: nimble=0 cache=%d (not correcting)", g_ble_conn_count);
                }
            }
        }

        /* === BLE connected or TT on → reset deep sleep counter, check light sleep === */
        if (is_ble_connected() || g_tt_powered) {
            g_deep_sleep_idle_sec = 0;

            /* --- Auto收网: periodically check TT network registration --- */
            if (g_tt_powered && tt_module_get_state() == TT_STATE_WORKING) {
                if (++g_tt_net_check_cnt >= TT_NO_NET_CHECK_SEC) {
                    g_tt_net_check_cnt = 0;

                    char creg_resp[128] = {0};
                    tt_at_result_t at_ret = tt_module_send_at_cmd_wait(
                        "AT+CREG?", creg_resp, sizeof(creg_resp), 3000);

                    bool registered = false;
                    if (at_ret == TT_AT_RESULT_OK) {
                        /* Parse +CREG: <n>,<stat> — stat 1=home, 5=roaming = registered */
                        char *p = strstr(creg_resp, "+CREG:");
                        if (p) {
                            int n_val = 0, stat = 0;
                            if (sscanf(p, "+CREG: %d,%d", &n_val, &stat) == 2) {
                                registered = (stat == 1 || stat == 5);
                            }
                        }
                    }

                    if (registered) {
                        if (!g_tt_was_registered) {
                            SYS_LOGI(TAG, "[Auto收网] Network registered, counter reset");
                        }
                        g_tt_no_net_sec = 0;
                        g_tt_was_registered = true;
                    } else {
                        g_tt_no_net_sec += TT_NO_NET_CHECK_SEC;
                        g_tt_was_registered = false;
                        /* Progress log every 5 min */
                        if (g_tt_no_net_sec % 300 == 0) {
                            SYS_LOGW(TAG, "[Auto收网] No network for %u/%u sec",
                                     g_tt_no_net_sec, TT_NO_NET_TIMEOUT_SEC);
                        }
                    }

                    /* Timeout: auto power off TT */
                    if (g_tt_no_net_sec >= TT_NO_NET_TIMEOUT_SEC) {
                        /* Safety: don't auto-off if user force-on or call active */
                        extern bool tt_module_is_force_on(void);
                        if (tt_module_is_force_on()) {
                            SYS_LOGW(TAG, "[Auto收网] Force-on active, skipping");
                            g_tt_no_net_sec = 0;
                            continue;
                        }
                        if (spp_voice_server_is_call_active()) {
                            SYS_LOGW(TAG, "[Auto收网] Call active, skipping");
                            g_tt_no_net_sec = 0;
                            continue;
                        }

                        SYS_LOGW(TAG, "[Auto收网] No network %us, powering off TT",
                                 g_tt_no_net_sec);
                        /* Network timeout shutdown: same hardware power-off as
                         * user_power_off but does NOT set NVS manual_off flag,
                         * allowing TT to auto-restart on next TEMP_AWAKE. */
                        tt_module_network_timeout_off();
                        /* notify_tt_powered_off already called by tt_module */
                        g_tt_no_net_sec = 0;
                        continue;  /* Re-evaluate state next iteration */
                    }
                }
            }

            /* Skip light sleep if TT is initializing */
            if (g_tt_powered && tt_module_get_state() == TT_STATE_INITIALIZING) {
                continue;
            }

            /* Never enter light sleep during an active voice call */
            if (spp_voice_server_is_call_active()) {
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
            /* Final safety check: never enter deep sleep with BLE connected.
             * is_ble_connected() checks the callback-maintained cache first. */
            if (is_ble_connected()) {
                SYS_LOGW(TAG, "Deep sleep aborted: BLE still connected (cache=%d)",
                         g_ble_conn_count);
                g_deep_sleep_idle_sec = 0;
                continue;
            }

            SYS_LOGI(TAG, "Entering DEEP_SLEEP / shutdown (ble_nimble=%d, ble_cache=%d, tt=%d, idle=%ds)",
                     ble_count, g_ble_conn_count, g_tt_powered, g_deep_sleep_idle_sec);

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
#ifdef SUPPORT_HARDWARE_V2
    gpio_wakeup_enable(GPIO_PWRKEY, GPIO_INTR_LOW_LEVEL);  /* Pwrkey wakes from light sleep */
#endif
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_LIGHT_TIMER_SEC * 1000000ULL);

    /* Enter light sleep - CPU pauses between BLE connection events, RAM preserved */
    esp_light_sleep_start();

    /* === Code continues here after wakeup === */
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

    /* TT module power + boost state based on TT state AND battery voltage.
     * Note: GPIO1 = GPIO_TTPWR_EN = BOOST_PWR_EN_GPIO (same pin, active LOW) */
#ifdef SUPPORT_HARDWARE_V2
    bool bat_low = (rtc_data.last_battery_mv > 0 && rtc_data.last_battery_mv < 3400);

    if (g_tt_powered) {
        /* TT ON: boost ON + normal mode + LDO ON */
        SYS_LOGI(TAG, "Deep sleep: TT ON (bat=%umV) — boost+LDO held ON",
                 rtc_data.last_battery_mv);
        gpio_set_level(BOOST_PWR_MODE_GPIO, 1);   /* Normal mode */
        gpio_set_level(GPIO_TT_LDO_EN, 1);        /* LDO ON */
        gpio_set_level(GPIO_TTPWR_EN, 0);         /* GPIO1 LOW = boost ON + TT power ON */
    } else if (bat_low) {
        /* TT OFF but battery LOW: boost ON normal mode to keep MCU alive,
         * LDO OFF to cut TT module */
        SYS_LOGI(TAG, "Deep sleep: TT OFF + low bat (%umV) — boost ON normal, LDO OFF",
                 rtc_data.last_battery_mv);
        gpio_set_level(BOOST_PWR_MODE_GPIO, 1);   /* Normal mode */
        gpio_set_level(GPIO_TT_LDO_EN, 0);        /* LDO OFF */
        gpio_set_level(GPIO_TTPWR_EN, 0);         /* GPIO1 LOW = boost ON */
    } else {
        /* TT OFF + battery OK: everything OFF */
        SYS_LOGI(TAG, "Deep sleep: TT OFF (bat=%umV) — all power OFF",
                 rtc_data.last_battery_mv);
        gpio_set_level(GPIO_TT_LDO_EN, 0);        /* LDO OFF */
        gpio_set_level(BOOST_PWR_MODE_GPIO, 0);   /* Low-power mode */
        gpio_set_level(GPIO_TTPWR_EN, 1);         /* GPIO1 HIGH = boost OFF */
    }
    gpio_hold_en(BOOST_PWR_EN_GPIO);
    gpio_hold_en(BOOST_PWR_MODE_GPIO);
    gpio_hold_en(GPIO_TTPWR_EN);
    gpio_hold_en(GPIO_TT_LDO_EN);
    /* IP5561 I2C pins high-Z + KEY held LOW */
    power_manage_ip5561_deep_sleep_prepare();
#else
    gpio_set_level(GPIO_TTPWR_EN, TT_PWR_OFF_LEVEL);
    gpio_hold_en(GPIO_TTPWR_EN);
#endif

    /* AP_WAKEUP_BB_PIN: set HIGH to tell BB "AP is sleeping"
     * BB will pull GPIO21 LOW to wake AP when it has messages */
    gpio_set_level(AP_WAKEUP_BB_PIN, 1);
    gpio_hold_en(AP_WAKEUP_BB_PIN);

    /* BB_WAKEUP_AP_PIN: pull-up (HIGH = BB sleeping, we wake on LOW) */
    rtc_gpio_pullup_en(BB_WAKEUP_AP_PIN);
    rtc_gpio_pulldown_dis(BB_WAKEUP_AP_PIN);

#ifdef SUPPORT_HARDWARE_V2
    /* GPIO9 pwrkey: pull-up (resting HIGH, user press → LOW triggers wakeup) */
    rtc_gpio_pullup_en(GPIO_PWRKEY);
    rtc_gpio_pulldown_dis(GPIO_PWRKEY);
#endif

    /* GPIO5 (IP5561 INT): pull-down ensures defined LOW during deep sleep when
     * IP5561 is not driving INT. When USB is inserted, IP5561 drives INT HIGH
     * (strong push-pull), overriding the weak RTC pull-down → ext0 wakes MCU. */
    rtc_gpio_pullup_dis(USB_WAKEUP_GPIO);
    rtc_gpio_pulldown_en(USB_WAKEUP_GPIO);

    /* Hold GPIO states during deep sleep */
    gpio_deep_sleep_hold_en();
}

/* ========== Pwrkey Monitor Task (V2 only) ========== */
#ifdef SUPPORT_HARDWARE_V2

/* ISR: notify task on falling edge (button press). Minimal — just wake the task. */
static void IRAM_ATTR pwrkey_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t hpw = pdFALSE;
    if (g_pwrkey_task_handle != NULL) {
        vTaskNotifyGiveFromISR(g_pwrkey_task_handle, &hpw);
    }
    if (hpw) portYIELD_FROM_ISR();
}

static void pwrkey_monitor_task(void *pvParameters)
{
    SYS_LOGI(TAG, "Pwrkey monitor started (GPIO%d, interrupt-driven)", GPIO_PWRKEY);

    while (g_pwrkey_task_running) {
        /* Block until ISR signals a press (zero CPU when idle) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!g_pwrkey_task_running) break;

        int64_t press_start_us = esp_timer_get_time();
        bool very_long_triggered = false;

        /* Poll only during press — check for release or 5s threshold */
        while (g_pwrkey_task_running) {
            vTaskDelay(pdMS_TO_TICKS(PWRKEY_POLL_MS));

            bool pressed = !gpio_get_level(GPIO_PWRKEY);  /* Active LOW */
            int64_t held_ms = (esp_timer_get_time() - press_start_us) / 1000;

            if (!pressed) {
                /* === Released === */
                if (held_ms < PWRKEY_DEBOUNCE_MS) {
                    /* Debounce — ignore */
                } else if (held_ms >= PWRKEY_SHORT_THRESHOLD_MS) {
                    /* Long press 2-5s: toggle TT power.
                     * But NOT during an active call — accidental long press
                     * would kill the call. Just refresh idle instead. */
                    if (spp_voice_server_is_call_active()) {
                        SYS_LOGW(TAG, "Pwrkey long (%lldms) ignored — call active", held_ms);
                        sleep_manager_notify_activity("pwrkey");
                    } else if (tt_module_is_powered()) {
                        SYS_LOGI(TAG, "Pwrkey long (%lldms) -> TT off", held_ms);
                        tt_module_user_power_off();
                    } else {
                        SYS_LOGI(TAG, "Pwrkey long (%lldms) -> TT on", held_ms);
                        tt_module_user_power_on();
                    }
                } else {
                    /* Short press < 2s: activity refresh */
                    SYS_LOGI(TAG, "Pwrkey short (%lldms) -> activity", held_ms);
                    sleep_manager_notify_activity("pwrkey");
                }
                break;
            }

            /* Still holding — check very long (5s) */
            if (!very_long_triggered && held_ms >= PWRKEY_LONG_THRESHOLD_MS) {
                very_long_triggered = true;
                SYS_LOGW(TAG, "Pwrkey held %lldms -> force deep sleep", held_ms);

                if (tt_module_is_powered()) {
                    SYS_LOGI(TAG, "Force sleep: powering off TT first");
                    tt_module_user_power_off();
                }
                sleep_manager_enter_deep_sleep();
                /* Does not return */
            }
        }
    }

    SYS_LOGI(TAG, "Pwrkey monitor stopped");
    g_pwrkey_task_handle = NULL;
    vTaskDelete(NULL);
}

#endif /* SUPPORT_HARDWARE_V2 */

static void pwrkey_init_and_start(void)
{
#ifdef SUPPORT_HARDWARE_V2
    /* Configure GPIO9 (pwrkey): input + pull-up + falling edge interrupt */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_PWRKEY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        SYS_LOGE(TAG, "Failed to configure pwrkey GPIO");
        return;
    }

    /* Install GPIO ISR service (ignore error if already installed by another driver) */
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        SYS_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(isr_ret));
        return;
    }

    /* Attach ISR handler */
    if (gpio_isr_handler_add(GPIO_PWRKEY, pwrkey_isr_handler, NULL) != ESP_OK) {
        SYS_LOGE(TAG, "Failed to add pwrkey ISR handler");
        return;
    }

    /* Create monitor task */
    g_pwrkey_task_running = true;
    BaseType_t ret = xTaskCreate(
        pwrkey_monitor_task, "pwrkey_mon", 4096, NULL, 4, &g_pwrkey_task_handle);

    if (ret != pdPASS) {
        SYS_LOGE(TAG, "Failed to create pwrkey monitor task");
        g_pwrkey_task_running = false;
    }
#endif
}

static void enter_deep_sleep_internal(void)
{
    g_current_mode = SLEEP_MODE_DEEP_SLEEP;

    /* ========== Save state to RTC memory with CRC update ========== */
    rtc_data.sleep_timestamp = esp_timer_get_time();

    /* Read battery voltage BEFORE stopping power tasks (need I2C alive) */
    fuel_gauge_handle_t bq = power_manage_get_fuel_gauge_handle();
    if (bq != NULL) {
        rtc_data.last_battery_mv = fuel_gauge_get_voltage(bq);
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

    /* TT module: power off only if it was already off intent.
     * If TT is ON, keep it powered — configure_gpio_for_deep_sleep() will
     * hold boost+LDO+TTPWR in ON state for the duration of deep sleep. */
    if (!g_tt_powered && tt_module_is_powered()) {
        SYS_LOGI(TAG, "Deep sleep: TT was not intended ON, powering off");
        tt_module_user_power_off();
    }

    /* Configure GPIO for minimum leakage */
    configure_gpio_for_deep_sleep();

    /* Configure wakeup sources */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    /*
     * ext1 wakeup: monitor multiple RTC GPIOs, wake on ANY going LOW.
     * - GPIO9 (pwrkey): always monitored, user press → full ACTIVE boot
     * - GPIO21 (BB_WAKEUP_AP): only when TT was powered (BB can signal)
     */
    {
        uint64_t ext1_pin_mask = 0;
#ifdef SUPPORT_HARDWARE_V2
        ext1_pin_mask |= (1ULL << GPIO_PWRKEY);  /* Always: pwrkey (active low) */
#endif
        if (g_tt_powered) {
            ext1_pin_mask |= (1ULL << BB_WAKEUP_AP_PIN);  /* BB wakeup (active low) */
        }
        if (ext1_pin_mask) {
            esp_sleep_enable_ext1_wakeup(ext1_pin_mask, ESP_EXT1_WAKEUP_ANY_LOW);
        }
    }

    /*
     * ext0 wakeup: USB insertion detection via IP5561 INT (GPIO5).
     * IP5561 drives GPIO5 HIGH when VBUS/USB activates it.
     * Only enable when GPIO5 is currently LOW (no USB); if already HIGH,
     * USB is already connected and enabling ext0 would fire immediately.
     */
    {
        int int_level = gpio_get_level(USB_WAKEUP_GPIO);
        if (int_level == 0) {
            esp_sleep_enable_ext0_wakeup(USB_WAKEUP_GPIO, 1);  /* Wake on HIGH */
            SYS_LOGI(TAG, "ext0 wakeup armed: GPIO%d (IP5561 INT) → USB insert", USB_WAKEUP_GPIO);
        } else {
            SYS_LOGI(TAG, "GPIO%d already HIGH (USB present), ext0 wakeup skipped", USB_WAKEUP_GPIO);
        }
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
 * @brief TEMP_AWAKE timeout handler task (runs with its own stack)
 *
 * Spawned by temp_awake_timer_callback to avoid stack overflow in the
 * Timer Service task. Performs safety checks before entering deep sleep.
 */
static void temp_awake_timeout_task(void *pvParameters)
{
    SYS_LOGI(TAG, "TEMP_AWAKE timeout, checking conditions before deep sleep...");

    /* Safety Check 1: Deferred init already completed */
    if (g_deferred_init_done) {
        SYS_LOGW(TAG, "TEMP_AWAKE: Deferred init completed, should not be here");
    }

    /* Safety Check 2: BLE connection */
    if (is_ble_connected()) {
        SYS_LOGI(TAG, "TEMP_AWAKE: BLE connected, aborting deep sleep");
        vTaskDelete(NULL);
        return;
    }

    /* Safety Check 3: Sleep inhibit */
    if (g_inhibit_sleep) {
        SYS_LOGW(TAG, "TEMP_AWAKE: Sleep inhibited, aborting deep sleep");
        vTaskDelete(NULL);
        return;
    }

    /* Safety Check 4: Current mode */
    if (g_current_mode != SLEEP_MODE_TEMP_AWAKE) {
        SYS_LOGW(TAG, "TEMP_AWAKE: Mode changed to %d, aborting deep sleep", g_current_mode);
        vTaskDelete(NULL);
        return;
    }

    /* Safety Check 5: Charging — skip deep sleep if charging */
    {
        uint8_t chg_flags = 0;
        bool is_charging = false;
        if (power_manage_get_charging_status(&chg_flags) == ESP_OK) {
            is_charging = (chg_flags & 0x04) != 0;   /* bit2: charging */
        }
        bool is_wpc = power_manage_is_wpc_charging();
        if (is_charging || is_wpc) {
            SYS_LOGI(TAG, "TEMP_AWAKE: Charging (wired=%d, wpc=%d), aborting deep sleep",
                     is_charging, is_wpc);
            /* Switch to ACTIVE mode and start normal sleep decision task */
            g_current_mode = SLEEP_MODE_ACTIVE;
            g_deep_sleep_idle_sec = 0;
            sleep_manager_task_start();
            vTaskDelete(NULL);
            return;
        }
    }

    SYS_LOGI(TAG, "TEMP_AWAKE: All checks passed, entering deep sleep");
    sleep_manager_enter_deep_sleep();
    /* Does not return */
}

/**
 * @brief TEMP_AWAKE timer callback (runs in Timer Service context)
 *
 * IMPORTANT: Keep this function minimal! The Timer Service task has a small
 * stack (~2KB). We only spawn a dedicated task here to do the heavy work
 * (logging, BLE scanning, I2C reads, deep sleep entry).
 */
static void temp_awake_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    xTaskCreate(temp_awake_timeout_task, "temp_awake_to", 4096, NULL, 3, NULL);
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
