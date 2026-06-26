#include "led_manager.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"

static const char *TAG = "LED";

static const int led_pins[LED_COUNT] = {
    GPIO_LED1, GPIO_LED2, GPIO_LED3, GPIO_LED4, GPIO_LED_G, GPIO_LED_R
};

void led_manager_init(void)
{
    /* Release deep sleep hold globally */
    gpio_deep_sleep_hold_dis();

    for (int i = 0; i < LED_COUNT; i++) {
        /* Release individual pin hold (persists across deep sleep wakeup) */
        gpio_hold_dis(led_pins[i]);
        /* Reset to GPIO function (clear any IO MUX peripheral assignment) */
        esp_rom_gpio_pad_select_gpio(led_pins[i]);
        gpio_set_direction(led_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(led_pins[i], 0);  /* Default OFF */
    }

    ESP_LOGI(TAG, "LED manager initialized (%d LEDs)", LED_COUNT);
}

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

static led_id_t s_heartbeat_led = LED_COUNT;

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
    s_heartbeat_led = id;
    xTaskCreate(led_heartbeat_task, "led_hb", 4096, NULL, 1, NULL);
    ESP_LOGI(TAG, "Heartbeat started on LED%d (GPIO%d)", id, led_pins[id]);
}
