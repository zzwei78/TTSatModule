#include "led_manager.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"

static const char *TAG = "LED";

/* ========== Physical LED pin tables (legacy API) ========== */
static const int led_pins[LED_COUNT] = {
    GPIO_LED1, GPIO_LED2, GPIO_LED3, GPIO_LED4, GPIO_LED_G, GPIO_LED_R
};

/* ========== RGB LED configuration ========== */
typedef struct {
    int r_pin;
    int g_pin;
    int b_pin;
} rgb_config_t;

static const rgb_config_t s_rgb[2] = {
    {LED1_R_GPIO, LED1_G_GPIO, LED1_B_GPIO},   /* Physical LED1 */
    {LED2_R_GPIO, LED2_G_GPIO, LED2_B_GPIO},    /* Physical LED2 */
};

/* ========== Per-LED state ========== */
typedef struct {
    led_color_t color;
    led_mode_t  mode;
    bool        blink_phase;   /* Current blink visibility (true=on) */
} led_state_t;

static led_state_t s_state[2] = {
    {LED_COLOR_OFF, LED_MODE_OFF, false},
    {LED_COLOR_OFF, LED_MODE_OFF, false},
};

/* Role → physical LED index (0=LED1, 1=LED2) */
static uint8_t s_role_map[LED_ROLE_COUNT] = {0, 1};

/* Heartbeat LED (legacy) */
static led_id_t s_heartbeat_led = LED_COUNT;
static bool s_heartbeat_active = false;

/* ========== Debug helpers ========== */
static const char *led_color_name(led_color_t c)
{
    switch (c) {
    case LED_COLOR_OFF:    return "OFF";
    case LED_COLOR_RED:    return "RED";
    case LED_COLOR_GREEN:  return "GREEN";
    case LED_COLOR_YELLOW: return "YELLOW";
    case LED_COLOR_BLUE:   return "BLUE";
    default:               return "?";
    }
}

static const char *led_mode_name(led_mode_t m)
{
    switch (m) {
    case LED_MODE_OFF:    return "OFF";
    case LED_MODE_SOLID:  return "SOLID";
    case LED_MODE_BLINK:  return "BLINK";
    default:              return "?";
    }
}

/* ========== Internal: apply color to physical LED ========== */
static void led_apply(int phys_idx)
{
    if (phys_idx < 0 || phys_idx >= 2) return;

    led_color_t color = s_state[phys_idx].color;
    led_mode_t mode = s_state[phys_idx].mode;

    /* Determine actual on/off state */
    bool show = false;
    if (mode == LED_MODE_SOLID) {
        show = true;
    } else if (mode == LED_MODE_BLINK) {
        show = s_state[phys_idx].blink_phase;
    }

    if (!show) color = LED_COLOR_OFF;

    /* Drive R/G/B pins */
    gpio_set_level(s_rgb[phys_idx].r_pin,
                   (color == LED_COLOR_RED || color == LED_COLOR_YELLOW) ? 1 : 0);
    gpio_set_level(s_rgb[phys_idx].g_pin,
                   (color == LED_COLOR_GREEN || color == LED_COLOR_YELLOW) ? 1 : 0);
    gpio_set_level(s_rgb[phys_idx].b_pin,
                   (color == LED_COLOR_BLUE) ? 1 : 0);
}

static void led_set_state(int phys_idx, led_color_t color, led_mode_t mode)
{
    if (phys_idx < 0 || phys_idx >= 2) return;
    s_state[phys_idx].color = color;
    s_state[phys_idx].mode = mode;
    /* Reset blink phase to ON when entering blink mode */
    if (mode == LED_MODE_BLINK) {
        s_state[phys_idx].blink_phase = true;
    }
    led_apply(phys_idx);
}

/* ========== Blink task ========== */
static void led_blink_task(void *pv)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));   /* 500ms = half period → 1s full blink cycle */

        for (int i = 0; i < 2; i++) {
            if (s_state[i].mode == LED_MODE_BLINK) {
                s_state[i].blink_phase = !s_state[i].blink_phase;
                led_apply(i);
            }
        }
    }
}

/* ========== Init ========== */
void led_manager_init(void)
{
    /* Release deep sleep hold globally */
    gpio_deep_sleep_hold_dis();

    /* Initialize all 6 LED pins as output, default OFF */
    for (int i = 0; i < LED_COUNT; i++) {
        gpio_hold_dis(led_pins[i]);
        esp_rom_gpio_pad_select_gpio(led_pins[i]);
        gpio_set_direction(led_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(led_pins[i], 0);
    }

    /* Start blink task (low priority) */
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "LED manager initialized (2 RGB LEDs, blink task started)");
}

/* ========== Battery LED ========== */
void led_set_battery_status(uint8_t soc, bool charging, bool full)
{
    int phys = s_role_map[LED_ROLE_BATTERY];
    led_color_t color;
    led_mode_t mode;

    if (full || soc >= 100) {
        /* Fully charged: green solid */
        color = LED_COLOR_GREEN;
        mode = LED_MODE_SOLID;
    } else if (charging) {
        /* Charging */
        if (soc < 75) {
            color = LED_COLOR_RED;
            mode = LED_MODE_BLINK;
        } else {
            color = LED_COLOR_GREEN;
            mode = LED_MODE_BLINK;
        }
    } else {
        /* Not charging */
        if (soc < 30) {
            color = LED_COLOR_RED;
            mode = LED_MODE_SOLID;
        } else if (soc < 75) {
            color = LED_COLOR_YELLOW;
            mode = LED_MODE_SOLID;
        } else {
            color = LED_COLOR_GREEN;
            mode = LED_MODE_SOLID;
        }
    }

    ESP_LOGI(TAG, "[BATT_LED] soc=%u%% charging=%d full=%d -> phys=%d %s/%s [R=GPIO%d G=GPIO%d B=GPIO%d]",
             (unsigned)soc, charging, full, phys,
             led_color_name(color), led_mode_name(mode),
             s_rgb[phys].r_pin, s_rgb[phys].g_pin, s_rgb[phys].b_pin);

    led_set_state(phys, color, mode);
}

/* ========== Signal LED ========== */
void led_set_signal_status(int state)
{
    int phys = s_role_map[LED_ROLE_SIGNAL];
    led_color_t color;
    led_mode_t mode;

    switch (state) {
    case LED_SIGNAL_TT_OFF:        /* TT module OFF → red blink */
        color = LED_COLOR_RED;
        mode = LED_MODE_BLINK;
        break;
    case LED_SIGNAL_NO_SIGNAL:     /* TT ON, before signal → red solid */
        color = LED_COLOR_RED;
        mode = LED_MODE_SOLID;
        break;
    case LED_SIGNAL_SEARCHING:     /* Network searching → yellow blink */
        color = LED_COLOR_YELLOW;
        mode = LED_MODE_BLINK;
        break;
    case LED_SIGNAL_CONNECTED:     /* Logged into network → green solid */
        color = LED_COLOR_GREEN;
        mode = LED_MODE_SOLID;
        break;
    default:
        color = LED_COLOR_OFF;
        mode = LED_MODE_OFF;
        break;
    }

    ESP_LOGI(TAG, "[SIG_LED] state=%d -> phys=%d %s/%s [R=GPIO%d G=GPIO%d B=GPIO%d]",
             state, phys,
             led_color_name(color), led_mode_name(mode),
             s_rgb[phys].r_pin, s_rgb[phys].g_pin, s_rgb[phys].b_pin);

    led_set_state(phys, color, mode);
}

/* ========== Swap assignment ========== */
void led_swap_assignment(void)
{
    uint8_t tmp = s_role_map[LED_ROLE_BATTERY];
    s_role_map[LED_ROLE_BATTERY] = s_role_map[LED_ROLE_SIGNAL];
    s_role_map[LED_ROLE_SIGNAL] = tmp;

    /* Re-apply both LEDs with new physical mapping */
    /* Save current logical states and re-apply */
    /* Battery LED: re-read from fuel gauge will update, for now just swap visuals */
    led_apply(0);
    led_apply(1);

    ESP_LOGI(TAG, "LED assignment swapped: battery→LED%d, signal→LED%d",
             s_role_map[LED_ROLE_BATTERY] + 1, s_role_map[LED_ROLE_SIGNAL] + 1);
}

/* ========== All off ========== */
void led_all_off(void)
{
    led_set_state(0, LED_COLOR_OFF, LED_MODE_OFF);
    led_set_state(1, LED_COLOR_OFF, LED_MODE_OFF);
}

/* ========== Legacy API ========== */
void led_set(led_id_t id, bool on)
{
    if (id < 0 || id >= LED_COUNT) return;
    gpio_set_level(led_pins[id], on ? 1 : 0);
}

void led_toggle(led_id_t id)
{
    if (id < 0 || id >= LED_COUNT) return;
    int lvl = gpio_get_level(led_pins[id]);
    gpio_set_level(led_pins[id], lvl ? 0 : 1);
}

static void led_heartbeat_task(void *pv)
{
    while (1) {
        led_set(s_heartbeat_led, 1);
        vTaskDelay(pdMS_TO_TICKS(800));
        led_set(s_heartbeat_led, 0);
        vTaskDelay(pdMS_TO_TICKS(4200));
    }
}

void led_start_heartbeat(led_id_t id)
{
    if (id < 0 || id >= LED_COUNT) return;
    if (s_heartbeat_active) return;   /* Prevent duplicate task */
    s_heartbeat_led = id;
    s_heartbeat_active = true;
    xTaskCreate(led_heartbeat_task, "led_hb", 2048, NULL, 1, NULL);
    ESP_LOGI(TAG, "Heartbeat started on LED%d (GPIO%d)", id, led_pins[id]);
}
