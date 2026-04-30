/*
 * tt_hardware.c - Tiantong Module Hardware Control Implementation
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "tt/tt_hardware.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "syslog.h"

/* Tag for logging */
static const char *TAG = "TT_HARDWARE";

/* Global Variables */
static bool g_hw_initialized = false;

//#define TT_PWR_PIN_INVERSE

/**
 * @brief Initialize Tiantong Module Hardware
 * 
 * This function initializes all the hardware interfaces for the Tiantong module
 * including GPIO pins, UART interfaces, and power control.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_init(void)
{
    esp_err_t ret = ESP_OK;

    if (g_hw_initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware already initialized");
        return ESP_OK;
    }

    /* Initialize Module Pins */
    gpio_reset_pin(TT_MODULE_RESET);
    gpio_reset_pin(TT_UART_BOOT);

    esp_rom_gpio_pad_select_gpio(TT_MODULE_RESET);
    esp_rom_gpio_pad_select_gpio(TT_UART_BOOT);
    esp_rom_gpio_pad_select_gpio(TT_IOTL_GPIO);

    /* Release GPIO holds from deep sleep (hold persists across wakeup reset on ESP32-S3) */
    gpio_hold_dis(GPIO_TTPWR_EN);
    gpio_hold_dis(AP_WAKEUP_BB_PIN);
    gpio_deep_sleep_hold_dis();

    /* Configure Output Pins */
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TT_MODULE_RESET) | (1ULL << TT_UART_BOOT) | 
                           (1ULL << AP_WAKEUP_BB_PIN) | (1ULL << GPIO_TTPWR_EN) | 
                           (1ULL << TT_IOTL_GPIO) | (1ULL << TT_UART1_RTS_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to configure output pins: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure Input Pins */
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << BB_WAKEUP_AP_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to configure input pins: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set initial pin states */
    gpio_set_pull_mode(TT_UART_BOOT, GPIO_FLOATING);
    gpio_set_pull_mode(TT_MODULE_RESET, GPIO_FLOATING);
    gpio_set_pull_mode(AP_WAKEUP_BB_PIN, GPIO_FLOATING);

    gpio_set_level(TT_IOTL_GPIO, 0);

#ifndef TT_PWR_PIN_INVERSE   
    gpio_set_level(GPIO_TTPWR_EN, 0);
#else 
    gpio_set_level(GPIO_TTPWR_EN, 1);
#endif // TT_PWR_PIN_INVERSE

    gpio_set_level(TT_UART1_RTS_PIN, 0);

    /* Initialize USB Switch */
    ret = gpio_reset_pin(USB_SEL_GPIO);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to reset USB switch pin: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_rom_gpio_pad_select_gpio(USB_SEL_GPIO);

    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << USB_SEL_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to configure USB switch: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_pull_mode(USB_SEL_GPIO, GPIO_FLOATING);
    gpio_set_level(USB_SEL_GPIO, 0); /* Default to normal mode */

    gpio_set_level(AP_WAKEUP_BB_PIN, 0);

    g_hw_initialized = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module Hardware initialized successfully");

    return ESP_OK;
}

/**
 * @brief Deinitialize Tiantong Module Hardware
 * 
 * This function deinitializes all hardware interfaces.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_deinit(void)
{
    if (!g_hw_initialized) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_OK;
    }

    /* Turn off power */
    tt_hw_power_off();

    g_hw_initialized = false;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Tiantong Module Hardware deinitialized");

    return ESP_OK;
}

/**
 * @brief Power On Tiantong Module
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_power_on(void)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Powering on Tiantong Module");
    //gpio_set_level(GPIO_TTPWR_EN, 1);
#ifndef TT_PWR_PIN_INVERSE   
    gpio_set_level(GPIO_TTPWR_EN, 1);
#else 
    gpio_set_level(GPIO_TTPWR_EN, 0);
#endif // TT_PWR_PIN_INVERSE

    return ESP_OK;
}

/**
 * @brief Power Off Tiantong Module
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_power_off(void)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Powering off Tiantong Module");
    //gpio_set_level(GPIO_TTPWR_EN, 0);
#ifndef TT_PWR_PIN_INVERSE   
    gpio_set_level(GPIO_TTPWR_EN, 0);
#else 
    gpio_set_level(GPIO_TTPWR_EN, 1);
#endif // TT_PWR_PIN_INVERSE    

    return ESP_OK;
}

/**
 * @brief Reset Tiantong Module Hardware
 * 
 * This function performs a hardware reset by toggling the reset pin.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_reset(void)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Performing hardware reset");

    /* Toggle reset pin */
    gpio_set_level(TT_MODULE_RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(TT_MODULE_RESET, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(TT_MODULE_RESET, 0);

    return ESP_OK;
}

/**
 * @brief Reset Tiantong Module Software
 * 
 * This function sends a software reset command to the module.
 * Note: This is typically handled by the AT command layer, but included here for completeness.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_software_reset(void)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Software reset is typically handled by AT commands");
    return ESP_OK;
}

/**
 * @brief Enter Download Mode
 * 
 * This function puts the module into download mode for firmware updates.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_enter_download_mode(void)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Entering download mode");

    gpio_set_level(TT_UART_BOOT, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(TT_MODULE_RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(TT_MODULE_RESET, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(TT_MODULE_RESET, 0);

    return ESP_OK;
}

/**
 * @brief Enter Normal Mode
 * 
 * This function puts the module into normal operating mode.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_enter_normal_mode(void)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Entering normal mode");

    gpio_set_level(TT_UART_BOOT, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(TT_MODULE_RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(TT_MODULE_RESET, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(TT_MODULE_RESET, 0);

    return ESP_OK;
}

/**
 * @brief Initialize UART1 for AT Commands
 * 
 * @param baud_rate Baud rate for UART1
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_init_uart1(int baud_rate)
{
    esp_err_t ret = ESP_OK;

    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, //UART_HW_FLOWCTRL_CTS_RTS,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Install UART driver */
    ret = uart_driver_install(UART_NUM_1, 4096, 4096, 0, NULL, 0);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to install UART1 driver: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure UART parameters */
    ret = uart_param_config(UART_NUM_1, &uart_config);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to configure UART1 parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(UART_NUM_1);
        return ret;
    }

    /* Set UART pins */
    //ret = uart_set_pin(UART_NUM_1, TT_UART1_TX_PIN, TT_UART1_RX_PIN, TT_UART1_RTS_PIN, TT_UART1_CTS_PIN);
    ret = uart_set_pin(UART_NUM_1, TT_UART1_TX_PIN, TT_UART1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to set UART1 pins: %s", esp_err_to_name(ret));
        uart_driver_delete(UART_NUM_1);
        return ret;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART1 initialized with baud rate %d", baud_rate);

    return ESP_OK;
}

/**
 * @brief Initialize UART2 for Log Output
 * 
 * @param baud_rate Baud rate for UART2
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_init_uart2(int baud_rate)
{
    esp_err_t ret = ESP_OK;

    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_rom_gpio_pad_select_gpio(TT_UART2_TX_PIN);
    esp_rom_gpio_pad_select_gpio(TT_UART2_RX_PIN);

    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Install UART driver */
    ret = uart_driver_install(UART_NUM_2, 4096, 4096, 0, NULL, 0);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to install UART2 driver: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure UART parameters */
    ret = uart_param_config(UART_NUM_2, &uart_config);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to configure UART2 parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(UART_NUM_2);
        return ret;
    }

    /* Set UART pins */
    ret = uart_set_pin(UART_NUM_2, TT_UART2_TX_PIN, TT_UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to set UART2 pins: %s", esp_err_to_name(ret));
        uart_driver_delete(UART_NUM_2);
        return ret;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART2 initialized with baud rate %d", baud_rate);

    return ESP_OK;
}

/**
 * @brief Set UART1 Baud Rate
 * 
 * @param baud_rate New baud rate
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_set_uart1_baud(int baud_rate)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Disable hardware flow control
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(UART_NUM_1, &uart_config);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to change UART1 baud rate: %s", esp_err_to_name(ret));
        return ret;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "UART1 baud rate changed to %d", baud_rate);

    return ESP_OK;
}

/**
 * @brief Set USB Switch State
 *
 * Controls USB data line routing:
 * - state = false: USB to IP5561 (charge mode - can charge external devices)
 * - state = true:  USB to ESP32 (communication mode - USB serial/programming)
 *
 * @param state false for IP5561, true for ESP32
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_set_usb_switch(bool state)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "USB switch routed to %s",
             state ? "ESP32 (communication)" : "IP5561 (charge mode)");
    gpio_set_level(USB_SEL_GPIO, state ? 0 : 1);

    return ESP_OK;
}

/**
 * @brief Set Boot Mode
 * 
 * @param is_download true for download mode, false for normal mode
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tt_hw_set_boot_mode(bool is_download)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Setting boot mode to %s", is_download ? "download" : "normal");
    gpio_set_level(TT_UART_BOOT, is_download ? 1 : 0);

    return ESP_OK;
}

/**
 * @brief Check if BP (Baseband Processor) is in sleep mode
 *
 * @return true if BP is sleeping (BP_SLEEP_AP is HIGH), false if BP is awake (BP_SLEEP_AP is LOW)
 */
bool tt_hw_is_bp_sleeping(void)
{
    if (!g_hw_initialized) {
        return false;
    }

    /* Read BP_SLEEP_AP pin (BB_WAKEUP_AP_PIN)
     * HIGH = BP is sleeping
     * LOW  = BP is awake
     */
    int level = gpio_get_level(BB_WAKEUP_AP_PIN);
    bool sleeping = (level == 1);

    SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "BP sleep status: %s (BP_SLEEP_AP=%d)",
                   sleeping ? "SLEEPING" : "AWAKE", level);

    return sleeping;
}

/**
 * @brief Wake up BP from sleep mode
 *
 * This function performs the following steps:
 * 1. Check if BP is sleeping (BP_SLEEP_AP pin level)
 * 2. If sleeping, drive AP_WAKEUP_BB from HIGH to LOW (falling edge) to trigger wakeup
 * 3. Wait for BP to respond by pulling BP_SLEEP_AP LOW
 * 4. Return when BP is awake
 *
 * @param timeout_ms Maximum time to wait for BP wakeup in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if BP doesn't wake up in time
 */
esp_err_t tt_hw_wakeup_bp(int timeout_ms)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Check if BP is already awake */
    if (!tt_hw_is_bp_sleeping()) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "BP is already awake");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Waking up BP...");

    /* Step 1: Set AP_WAKEUP_BB to HIGH first (if not already) */
    gpio_set_level(AP_WAKEUP_BB_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Step 2: Drive AP_WAKEUP_BB LOW (falling edge triggers wakeup) */
    gpio_set_level(AP_WAKEUP_BB_PIN, 0);

    /* Step 3: Wait for BP to respond by pulling BP_SLEEP_AP LOW */
    int elapsed = 0;
    int poll_interval = 10; /* Poll every 10ms */

    while (elapsed < timeout_ms) {
        if (!tt_hw_is_bp_sleeping()) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "BP wakeup successful (took %d ms)", elapsed);
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(poll_interval));
        elapsed += poll_interval;
    }

    /* Timeout - BP didn't wake up */
    SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "BP wakeup timeout after %d ms", timeout_ms);

    /* Release AP_WAKEUP_BB */
    gpio_set_level(AP_WAKEUP_BB_PIN, 0);

    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Allow BP to enter sleep mode
 *
 * This function releases AP_WAKEUP_BB to allow BP to enter sleep mode
 * when appropriate. Sets AP_WAKEUP_BB to HIGH (inactive state).
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_hw_allow_bp_sleep(void)
{
    if (!g_hw_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Hardware not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Set AP_WAKEUP_BB to HIGH (inactive, allows BP to sleep) */
    gpio_set_level(AP_WAKEUP_BB_PIN, 1);

    SYS_LOGD_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "AP_WAKEUP_BB set HIGH (BP can sleep now)");

    return ESP_OK;
}
