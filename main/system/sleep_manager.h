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

/* ========== Feature Switches ========== */
/* Set to 1 to enable, 0 to disable */
#ifndef SLEEP_LIGHT_SLEEP_ENABLE
#define SLEEP_LIGHT_SLEEP_ENABLE       0   /* Light sleep (CPU pause between BLE events) */
#endif
#ifndef SLEEP_TEMP_AWAKE_ENABLE
#define SLEEP_TEMP_AWAKE_ENABLE       1   /* TEMP_AWAKE (periodic timer wake + advertise) */
#endif

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

/* Deep sleep entry thresholds */
#define SLEEP_DEEP_IDLE_DISCONNECTED_SEC   300   /* BLE disconnected + TT off: 5 min idle */
#define SLEEP_DEEP_IDLE_CONNECTED_SEC      600   /* BLE connected or TT on: 10 min no command */
#define SLEEP_PRE_NOTIFY_SEC               540   /* 9 min: push "sleep imminent" notification */
#define SLEEP_DEEP_TIMER_SEC               600   /* Deep sleep timer interval: 10 min */

/* Light sleep (if enabled) */
#define SLEEP_LIGHT_IDLE_SEC               600   /* Light sleep idle threshold */
#define SLEEP_LIGHT_REENTER_SEC            2     /* Quick re-sleep after wake */
#define SLEEP_LIGHT_TIMER_SEC             30    /* Light sleep timer interval */

/* TEMP_AWAKE (if enabled) */
#define SLEEP_TEMP_AWAKE_SEC              25    /* Timer wakeup: BLE advertise duration */

/* Pwrkey thresholds (ms) */
#define PWRKEY_POLL_MS                     20    /* Poll interval during press */
#define PWRKEY_DEBOUNCE_MS                 50    /* Min press to register */
#define PWRKEY_SHORT_THRESHOLD_MS          2000  /* < 2s: short press (open TT / refresh idle) */
#define PWRKEY_SLEEP_THRESHOLD_MS          5000  /* 5-10s release: force TT off + deep sleep */
#define PWRKEY_RESET_THRESHOLD_MS          10000 /* >= 10s release: hardware reboot TT */

/* Pwrkey 5-10s long-press action (TEST mode for magnetometer hard-iron calibration):
 *   0 = force TT off + deep sleep  (production default)
 *   1 = enter mag hard-iron calibration routine (test, both blue LEDs blink)
 * Override on compile cmd: -DPWRKEY_5S_ACTION_MAG_CALIBRATION=1 */
#ifndef PWRKEY_5S_ACTION_MAG_CALIBRATION
#define PWRKEY_5S_ACTION_MAG_CALIBRATION   0
#endif

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
 * @brief Push "sleep imminent" notification to connected APP
 *
 * Called by sleep manager 1 minute before entering deep sleep.
 * Format: [0x09][seconds_until_sleep (uint16 LE)]
 *
 * @param seconds_until_sleep Seconds remaining before deep sleep
 * @return 0 on success, negative on error
 */
int gatt_system_server_send_sleep_warning(uint16_t seconds_until_sleep);

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
