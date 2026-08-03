#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "config/hardware_version.h"

/* LED pin definitions */
#ifndef GPIO_LED1
#define GPIO_LED1           10
#endif
#ifndef GPIO_LED2
#define GPIO_LED2           11
#endif
#ifndef GPIO_LED3
#define GPIO_LED3           12
#endif
#ifndef GPIO_LED4
#define GPIO_LED4           13
#endif
#ifndef GPIO_LED_G
#define GPIO_LED_G          41
#endif
#ifndef GPIO_LED_R
#define GPIO_LED_R          42
#endif

//#define LED1_R_GPIO         GPIO_LED2
//#define LED1_G_GPIO        GPIO_LED4 
//#define LED1_B_GPIO         GPIO_LED1

#define LED1_R_GPIO         GPIO_LED4 
#define LED1_G_GPIO        GPIO_LED2
#define LED1_B_GPIO         GPIO_LED1

#define LED2_R_GPIO         GPIO_LED_R
#define LED2_G_GPIO         GPIO_LED_G
#define LED2_B_GPIO         GPIO_LED3

/* ========== RGB LED Colors ========== */
typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_YELLOW,    /* R+G simultaneously */
    LED_COLOR_BLUE,
} led_color_t;

/* ========== LED Display Modes ========== */
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_SOLID,      /* Steady on */
    LED_MODE_BLINK,      /* 1s interval blink (500ms on / 500ms off) */
} led_mode_t;

/* ========== Logical LED Roles ========== */
typedef enum {
    LED_ROLE_BATTERY = 0,    /* Battery status indicator */
    LED_ROLE_SIGNAL,         /* Satellite signal indicator */
    LED_ROLE_COUNT
} led_role_t;

/* ========== Signal States (4 scenarios) ========== */
#define LED_SIGNAL_TT_OFF       0   /* TT module OFF        → red blink    */
#define LED_SIGNAL_NO_SIGNAL    1   /* TT ON, before signal → red solid    */
#define LED_SIGNAL_SEARCHING    2   /* Network searching    → yellow blink */
#define LED_SIGNAL_CONNECTED    3   /* Logged into network  → green solid  */

/* ========== Legacy LED IDs (for backward compatibility) ========== */
typedef enum {
    LED_1 = 0,
    LED_2,
    LED_3,
    LED_4,
    LED_G,
    LED_R,
    LED_COUNT,
} led_id_t;

/**
 * @brief Initialize LED manager
 * Configures all RGB LED GPIOs, starts blink task.
 * Handles deep sleep GPIO hold release.
 */
void led_manager_init(void);

/* ========== RGB LED Status API ========== */

/**
 * @brief Update battery LED based on SOC and charging state
 * @param soc       State of charge (0-100)
 * @param charging  true if currently charging
 * @param full      true if charge complete (SOC=100% or IP5561 done flag)
 */
void led_set_battery_status(uint8_t soc, bool charging, bool full);

/**
 * @brief Update signal LED based on TT module / network state
 * @param state  LED_SIGNAL_TT_OFF / LED_SIGNAL_NO_SIGNAL / LED_SIGNAL_SEARCHING / LED_SIGNAL_CONNECTED
 */
void led_set_signal_status(int state);

/**
 * @brief Swap physical LED assignment between battery and signal roles
 */
void led_swap_assignment(void);

/**
 * @brief Turn off all LEDs (call before deep sleep)
 */
void led_all_off(void);

/* ========== Legacy API (backward compatible) ========== */

void led_set(led_id_t id, bool on);
void led_toggle(led_id_t id);
void led_start_heartbeat(led_id_t id);

#endif /* LED_MANAGER_H */
