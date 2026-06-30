/*
 * sleep_manager.h - Hybrid Sleep Manager for ESP32-S3
 *
 * Manages Light Sleep (BLE connected) and Deep Sleep (BLE disconnected)
 * to reduce idle power from ~60mA to <1mA.
 */

#ifndef SLEEP_MANAGER_H
#define SLEEP_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config/hardware_version.h"

/* ========== Sleep State Machine ========== */

typedef enum {
    SLEEP_MODE_ACTIVE = 0,      /* Normal operation (~60mA) */
    SLEEP_MODE_LIGHT_SLEEP,     /* BLE connected + idle (~240uA) */
    SLEEP_MODE_DEEP_SLEEP,      /* BLE disconnected + TT off (~7uA) */
    SLEEP_MODE_TEMP_AWAKE       /* Deep sleep timer wakeup, advertise briefly */
} sleep_mode_t;

/* ========== Deep Sleep Wakeup Cause ========== */

typedef enum {
    SLEEP_WAKEUP_NONE = 0,
    SLEEP_WAKEUP_TIMER,          /* Periodic timer wakeup -> TEMP_AWAKE */
    SLEEP_WAKEUP_GPIO21,         /* BB_WAKEUP_AP_PIN -> full init */
    SLEEP_WAKEUP_PWRKEY,         /* GPIO9 pwrkey press -> full init */
    SLEEP_WAKEUP_USB,            /* IP5561 INT (GPIO5) HIGH -> USB inserted -> full init */
    SLEEP_WAKEUP_OTHER
} sleep_wakeup_cause_t;

/* ========== Configuration Constants ========== */

#define SLEEP_LIGHT_IDLE_SEC        3600000  /* Disabled (1M seconds) — light sleep causes BLE supervision timeout with test clients */
#define SLEEP_LIGHT_REENTER_SEC     2       /* Re-entry: quick re-sleep after wake */
#ifdef SUPPORT_HARDWARE_V2
#define SLEEP_DEEP_IDLE_SEC         120     /* V2.0: BLE disconnected, idle before deep sleep (2 min) */
#define SLEEP_DEEP_TIMER_SEC        1800    /* V2.0: Deep sleep interval between timer wakeups (30 min) */
#else
#define SLEEP_DEEP_IDLE_SEC         300     /* V1.0: BLE disconnected, idle before deep sleep (5 min) */
#define SLEEP_DEEP_TIMER_SEC        60      /* V1.0: Deep sleep interval between timer wakeups (1 min) */
#endif
#define SLEEP_TEMP_AWAKE_SEC        25      /* Timer wakeup: BLE advertise duration */
#define SLEEP_LIGHT_TIMER_SEC       30      /* Light sleep timer interval (battery check) */

/* ========== Public API ========== */

/**
 * @brief Initialize the sleep manager
 *
 * Must be called after NVS init. Detects deep sleep wakeup cause.
 */
esp_err_t sleep_manager_init(void);

/**
 * @brief Start the sleep decision background task
 */
esp_err_t sleep_manager_task_start(void);

/**
 * @brief Stop the sleep decision task
 */
esp_err_t sleep_manager_task_stop(void);

/**
 * @brief Get current sleep mode
 */
sleep_mode_t sleep_manager_get_mode(void);

/**
 * @brief Notify sleep manager of any activity (resets idle timer)
 *
 * Call from: BLE data received, AT commands, voice call events
 */
void sleep_manager_notify_activity(const char *source);

/**
 * @brief Lightweight idle refresh (no logging, for high-frequency calls)
 *
 * Call from voice data path to keep device awake during calls
 * without flooding logs. Just updates the timestamp.
 */
void sleep_manager_refresh_idle(void);

/**
 * @brief Notify that a BLE client has connected
 */
void sleep_manager_notify_ble_connected(void);

/**
 * @brief Notify that a BLE client has disconnected
 */
void sleep_manager_notify_ble_disconnected(void);

/**
 * @brief Notify that the TT module has been powered on
 */
void sleep_manager_notify_tt_powered_on(void);

/**
 * @brief Notify that the TT module has been powered off
 */
void sleep_manager_notify_tt_powered_off(void);

/**
 * @brief Check if this boot is a deep sleep wakeup
 */
bool sleep_manager_is_deep_sleep_wakeup(void);

/**
 * @brief Check if currently in TEMP_AWAKE mode (timer wakeup, minimal init)
 */
bool sleep_manager_is_temp_awake(void);

/**
 * @brief Get the deep sleep wakeup cause
 */
sleep_wakeup_cause_t sleep_manager_get_wakeup_cause(void);

/**
 * @brief Print wakeup info (call after syslog_init)
 */
void sleep_manager_print_wakeup_info(void);

/**
 * @brief Forcibly enter deep sleep now (for low-battery shutdown)
 */
void sleep_manager_enter_deep_sleep(void);

/**
 * @brief Prevent or allow sleep (for OTA, flash operations)
 */
void sleep_manager_set_inhibit(bool inhibit);

/**
 * @brief TEMP_AWAKE timeout callback - re-enters deep sleep
 */
void sleep_manager_temp_awake_timeout(void);

/**
 * @brief Start the TEMP_AWAKE one-shot timer (called from app_main)
 */
esp_err_t sleep_manager_start_temp_awake_timer(void);

#endif /* SLEEP_MANAGER_H */
