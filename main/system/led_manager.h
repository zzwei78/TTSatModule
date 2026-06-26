#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <stdbool.h>
#include "config/hardware_version.h"

/* LED pin definitions */
#define GPIO_LED1           10
#define GPIO_LED2           11
#define GPIO_LED3           12
#define GPIO_LED4           13
#define GPIO_LED_G          41
#define GPIO_LED_R          42

/* LED IDs */
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
 * @brief Initialize all LED GPIOs as output (default LOW = off)
 *        Call this early in app_main, before other init.
 *        Handles deep sleep GPIO hold release.
 */
void led_manager_init(void);

/**
 * @brief Set LED on/off
 * @param id LED ID
 * @param on true=ON (HIGH), false=OFF (LOW)
 */
void led_set(led_id_t id, bool on);

/**
 * @brief Toggle LED
 */
void led_toggle(led_id_t id);

/**
 * @brief Start heartbeat blink on specified LED
 *        ON 800ms, OFF 4200ms, repeat every 5s
 * @param id LED to use for heartbeat
 */
void led_start_heartbeat(led_id_t id);

#endif /* LED_MANAGER_H */
