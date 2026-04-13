/*
 * Tiantong Hardware Layer Test Program
 * This file demonstrates the usage of the hardware abstraction layer
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tt/tt_hardware.h"

static const char *TAG = "TT_HW_TEST";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Tiantong Hardware Layer Test");
    
    // Initialize hardware layer
    esp_err_t ret = tt_hw_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Hardware layer initialized successfully");
    } else {
        ESP_LOGE(TAG, "✗ Hardware layer initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Test power control functions
    ESP_LOGI(TAG, "Testing power control functions...");
    ret = tt_hw_power_on();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Power on successful");
    } else {
        ESP_LOGE(TAG, "✗ Power on failed: %s", esp_err_to_name(ret));
    }
    
    // Wait a moment
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Test reset functions
    ESP_LOGI(TAG, "Testing reset functions...");
    ret = tt_hw_software_reset();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Software reset requested");
    } else {
        ESP_LOGE(TAG, "✗ Software reset failed: %s", esp_err_to_name(ret));
    }
    
    // Wait a moment
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Test USB switch control
    ESP_LOGI(TAG, "Testing USB switch control...");
    ret = tt_hw_set_usb_switch(true);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ USB switch set to ON");
    } else {
        ESP_LOGE(TAG, "✗ USB switch control failed: %s", esp_err_to_name(ret));
    }
    
    // Test UART initialization
    ESP_LOGI(TAG, "Testing UART initialization...");
    ret = tt_hw_init_uart1(115200);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ UART1 initialized at 115200 baud");
    } else {
        ESP_LOGE(TAG, "✗ UART1 initialization failed: %s", esp_err_to_name(ret));
    }
    
    // Test baud rate change
    ret = tt_hw_set_uart1_baud(9600);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ UART1 baud rate changed to 9600");
    } else {
        ESP_LOGE(TAG, "✗ UART1 baud rate change failed: %s", esp_err_to_name(ret));
    }
    
    // Test boot mode setting
    ESP_LOGI(TAG, "Testing boot mode setting...");
    ret = tt_hw_set_boot_mode(false); // Normal mode
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Boot mode set to normal");
    } else {
        ESP_LOGE(TAG, "✗ Boot mode setting failed: %s", esp_err_to_name(ret));
    }
    
    // Test hardware reset (commented out to avoid actual reset)
    /*
    ESP_LOGI(TAG, "Testing hardware reset...");
    ret = tt_hw_reset();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Hardware reset performed");
    } else {
        ESP_LOGE(TAG, "✗ Hardware reset failed: %s", esp_err_to_name(ret));
    }
    */
    
    // Test deinitialization
    ESP_LOGI(TAG, "Testing hardware deinitialization...");
    ret = tt_hw_deinit();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Hardware layer deinitialized successfully");
    } else {
        ESP_LOGE(TAG, "✗ Hardware layer deinitialization failed: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "Tiantong Hardware Layer Test completed!");
    
    // Keep task running
    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
