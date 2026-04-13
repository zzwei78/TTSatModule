/*
 * tt_hardware.h - Tiantong Module Hardware Control Interface
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef TT_HARDWARE_H
#define TT_HARDWARE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* GPIO Definitions */
#define GPIO_TTPWR_EN       GPIO_NUM_1      /* Power Enable */
#define USB_SEL_GPIO        GPIO_NUM_48     /* USB Switch */
#define TT_IOTL_GPIO        GPIO_NUM_47     /* IOTL Control */
#define TT_UART_BOOT        GPIO_NUM_46     /* UART Boot Mode */
#define TT_MODULE_RESET     GPIO_NUM_45     /* Module Reset */
#define AP_WAKEUP_BB_PIN    GPIO_NUM_35     /* AP Wakeup BB */
#define BB_WAKEUP_AP_PIN    GPIO_NUM_21     /* BB Wakeup AP */

/* UART Definitions */
#define TT_UART1_TX_PIN     GPIO_NUM_17     /* UART1 TX */
#define TT_UART1_RX_PIN     GPIO_NUM_18     /* UART1 RX */
#define TT_UART1_CTS_PIN    GPIO_NUM_34     /* UART1 CTS */
#define TT_UART1_RTS_PIN    GPIO_NUM_33     /* UART1 RTS */

#define TT_UART2_TX_PIN     GPIO_NUM_39     /* UART2 TX */
#define TT_UART2_RX_PIN     GPIO_NUM_40     /* UART2 RX */

/* Hardware Initialization */
esp_err_t tt_hw_init(void);
esp_err_t tt_hw_deinit(void);

/* Power Control */
esp_err_t tt_hw_power_on(void);
esp_err_t tt_hw_power_off(void);

/* Reset Control */
esp_err_t tt_hw_reset(void);
esp_err_t tt_hw_software_reset(void);

/* Mode Control */
esp_err_t tt_hw_enter_download_mode(void);
esp_err_t tt_hw_enter_normal_mode(void);

/* UART Control */
esp_err_t tt_hw_init_uart1(int baud_rate);
esp_err_t tt_hw_init_uart2(int baud_rate);
esp_err_t tt_hw_set_uart1_baud(int baud_rate);

/* USB Switch Control */
/**
 * @brief Set USB switch to route USB data lines
 * @param state false=USB to IP5561 (charge mode), true=USB to ESP32 (communication mode)
 */
esp_err_t tt_hw_set_usb_switch(bool state);

/* Wakeup Control */
esp_err_t tt_hw_set_boot_mode(bool is_download);

/**
 * @brief Check if BP (Baseband Processor) is in sleep mode
 *
 * @return true if BP is sleeping (BP_SLEEP_AP is HIGH), false if BP is awake (BP_SLEEP_AP is LOW)
 */
bool tt_hw_is_bp_sleeping(void);

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
esp_err_t tt_hw_wakeup_bp(int timeout_ms);

/**
 * @brief Allow BP to enter sleep mode
 *
 * This function releases AP_WAKEUP_BB to allow BP to enter sleep mode
 * when appropriate. Sets AP_WAKEUP_BB to HIGH (inactive state).
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tt_hw_allow_bp_sleep(void);

#endif /* TT_HARDWARE_H */
