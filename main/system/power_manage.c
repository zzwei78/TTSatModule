/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_bt.h"
#include "IP5561.h"
#include "system/power_manage.h"
#include "syslog.h"
#include "tt/tt_module.h"
#include "ble/gatt_system_server.h"
#include "ble/spp_at_server.h"
#include "config/user_params.h"
#include "config/hardware_version.h"

static const char *TAG = "POWER_MANAGE";

/* Cached battery data for interrupt context access */
static struct {
    uint16_t voltage_mv;
    int16_t current_ma;
    bool is_charging;
    uint32_t last_update_ms;
    bool initialized;
} g_battery_cache = {0};

/* I2C bus handle */
static i2c_master_bus_handle_t g_i2c_bus_handle = NULL;

/* IP5561 device handle (Charging management) */
static ip5561_handle_t g_ip5561_handle = NULL;

/* Fuel gauge device handle (auto-detected BQ27220 or CW2217E) */
static fuel_gauge_handle_t g_fuel_gauge_handle = NULL;

/* DA228EC device handle (3-axis accelerometer, primary) */
static da228ec_handle_t g_da228ec_handle = NULL;

/* SC7A20H device handle (3-axis accelerometer, second source) */
static sc7a20h_handle_t g_sc7a20h_handle = NULL;

/* MMC5603 device handle (3-axis magnetometer) */
static mmc5603_handle_t g_mmc5603_handle = NULL;

/* Cached sensor data */
static struct {
    int16_t ax, ay, az;         /* mg */
    int32_t mx, my, mz;         /* mG */
    uint8_t flags;               /* bit0=accel_valid, bit1=mag_valid */
    bool report_enabled;
} g_sensor_cache = {0};

/* Monitor task handles */
static TaskHandle_t g_ip5561_task_handle = NULL;
static TaskHandle_t g_fuel_gauge_task_handle = NULL;

/* Monitor task running flags */
static bool g_ip5561_task_running = false;
static bool g_fuel_gauge_task_running = false;

/* Cached average current (calculated from CoulombCounter delta, updated every 60s) */
static int16_t g_avg_current_ma = 0;

/* IP5561 configuration fingerprint for reset detection */
typedef struct {
    uint8_t sys_ctl0;
    uint8_t sys_ctl1;
    uint8_t qc_ctrl0;
    uint8_t pd_ctrl;
    uint8_t chg_tmo_ctl1;
    bool initialized;
} ip5561_config_fingerprint_t;

static ip5561_config_fingerprint_t g_ip5561_fingerprint = {0};

#ifdef SUPPORT_HARDWARE_V2
/* V2.0: Second I2C bus for IP5561 */
static i2c_master_bus_handle_t g_i2c_bus_handle_ip5561 = NULL;

/* V2.0: Boost power manager state */
static volatile uint8_t g_boost_ref_count = 0;
static volatile bool g_boost_consumer_active[BOOST_CONSUMER_COUNT] = {false};
static volatile bool g_mcu_low_battery_boost = false;  /* MCU holds boost for low-battery survival */

/* Forward declarations */
static void boost_gpio_init(void);
static void boost_update_gpio(void);
#endif

/* I2C pin definitions */
#define POWER_I2C_SCL_IO        GPIO_NUM_2      /* I2C SCL */
#define POWER_I2C_SDA_IO        GPIO_NUM_3      /* I2C SDA */
#define POWER_I2C_PORT_NUM      I2C_NUM_0       /* I2C port */
#define POWER_I2C_FREQ_HZ       200000          /* 200kHz */

#ifdef SUPPORT_HARDWARE_V2
/* V2.0: IP5561 on separate I2C1 bus */
#define IP5561_I2C_SCL_IO       GPIO_NUM_6
#define IP5561_I2C_SDA_IO       GPIO_NUM_14
#define IP5561_I2C_PORT_NUM     I2C_NUM_1

/* V2.0: Boost IC GPIO definitions */
#define BOOST_PWR_EN_GPIO       GPIO_NUM_1      /* Active LOW: LOW=boost on */
#define BOOST_PWR_MODE_GPIO     GPIO_NUM_36     /* HIGH=normal, LOW=low-power */

/* V2.0: Low-battery boost thresholds (hysteresis) */
#define LOW_BAT_BOOST_OFF_MV    3300            /* Falling: TT off, MCU requests boost */
#define LOW_BAT_BOOST_ON_MV     3400            /* Rising: TT can restart, MCU releases boost */
#endif

/* IP5561 wakeup GPIO definition */
#define IP5561_WAKEUP_GPIO      GPIO_NUM_8      /* GPIO8 for IP5561 wakeup */
#define IP5561_WAKEUP_DELAY_MS  10              /* 10ms delay per state */

/* ========== IP5561 Wakeup Functions ========== */

/**
 * @brief Initialize IP5561 wakeup GPIO
 *
 * Configures GPIO8 as output, initial state LOW
 */
static void ip5561_wakeup_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IP5561_WAKEUP_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(IP5561_WAKEUP_GPIO, 0);  /* Default LOW */
}

/**
 * @brief IP5561 wakeup sequence: LOW -> HIGH -> LOW
 *
 * Each state maintains for 10ms to ensure reliable wakeup
 */
static void ip5561_wakeup_sequence(void)
{
    gpio_set_level(IP5561_WAKEUP_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(IP5561_WAKEUP_DELAY_MS));

    gpio_set_level(IP5561_WAKEUP_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(IP5561_WAKEUP_DELAY_MS));

    gpio_set_level(IP5561_WAKEUP_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(IP5561_WAKEUP_DELAY_MS / 2));  /* 5ms stabilization */
}

/* ========== Boost Power Manager (V2.0 only) ========== */
#ifdef SUPPORT_HARDWARE_V2

static void boost_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOST_PWR_EN_GPIO) | (1ULL << BOOST_PWR_MODE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Default: boost off, low-power mode */
    gpio_set_level(BOOST_PWR_MODE_GPIO, 0);
    gpio_set_level(BOOST_PWR_EN_GPIO, 1);  /* HIGH = off (active low) */
}

static void boost_update_gpio(void)
{
    if (g_boost_ref_count > 0) {
        /* At least one consumer: boost on, normal mode */
        gpio_set_level(BOOST_PWR_MODE_GPIO, 1);  /* Normal mode */
        gpio_set_level(BOOST_PWR_EN_GPIO, 0);    /* LOW = on */
    } else {
        /* No consumers: low-power mode, boost off */
        gpio_set_level(BOOST_PWR_MODE_GPIO, 0);  /* Low-power mode */
        gpio_set_level(BOOST_PWR_EN_GPIO, 1);    /* HIGH = off */
    }
}

esp_err_t power_manage_boost_request(boost_consumer_t who)
{
    if (who < 0 || who >= BOOST_CONSUMER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_boost_consumer_active[who]) {
        g_boost_consumer_active[who] = true;
        g_boost_ref_count++;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "[BOOST] request by %d, ref_count=%u", who, g_boost_ref_count);
        boost_update_gpio();
    }
    return ESP_OK;
}

esp_err_t power_manage_boost_release(boost_consumer_t who)
{
    if (who < 0 || who >= BOOST_CONSUMER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_boost_consumer_active[who]) {
        g_boost_consumer_active[who] = false;
        if (g_boost_ref_count > 0) {
            g_boost_ref_count--;
        }
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "[BOOST] release by %d, ref_count=%u", who, g_boost_ref_count);
        boost_update_gpio();
    }
    return ESP_OK;
}

bool power_manage_boost_is_active(void)
{
    return g_boost_ref_count > 0;
}

void power_manage_boost_deep_sleep_prepare(void)
{
    /* Turn off boost and hold GPIO state for deep sleep */
    gpio_set_level(BOOST_PWR_MODE_GPIO, 0);  /* Low-power mode */
    gpio_set_level(BOOST_PWR_EN_GPIO, 1);    /* HIGH = off (active low) */
    gpio_hold_en(BOOST_PWR_MODE_GPIO);
    gpio_hold_en(BOOST_PWR_EN_GPIO);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[BOOST] Deep sleep: boost OFF, GPIOs held");
}

#endif /* SUPPORT_HARDWARE_V2 */

/* ========== Interrupt-Safe Battery Data Access ========== */

/**
 * @brief Update battery cache with new readings
 *
 * This function should be called periodically from the monitor task
 * to keep the cache fresh for interrupt context access.
 */
static void update_battery_cache(void)
{
    if (g_fuel_gauge_handle == NULL) {
        return;
    }

    /* Read battery data */
    uint16_t voltage = fuel_gauge_get_voltage(g_fuel_gauge_handle);
    int16_t current = fuel_gauge_get_current(g_fuel_gauge_handle);

    /* Determine charging state */
    bool is_charging = (current < -200);

    /* Update cache atomically */
    UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
    g_battery_cache.voltage_mv = voltage;
    g_battery_cache.current_ma = current;
    g_battery_cache.is_charging = is_charging;
    g_battery_cache.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_battery_cache.initialized = true;
    taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
}

/**
 * @brief Get battery voltage safely (works in interrupt context)
 *
 * In interrupt context, returns cached value.
 * In normal context, reads from I2C directly.
 */
static uint16_t safe_get_battery_voltage(void)
{
    if (xPortInIsrContext()) {
        /* In interrupt context - use cached value */
        if (!g_battery_cache.initialized) {
            return 0;  /* No cache available */
        }
        return g_battery_cache.voltage_mv;
    }
    /* In normal context - read from I2C directly */
    if (g_fuel_gauge_handle == NULL) {
        return 0;
    }
    return fuel_gauge_get_voltage(g_fuel_gauge_handle);
}

/**
 * @brief Get battery current safely (works in interrupt context)
 */
static int16_t safe_get_battery_current(void)
{
    if (xPortInIsrContext()) {
        /* In interrupt context - use cached value */
        if (!g_battery_cache.initialized) {
            return 0;
        }
        return g_battery_cache.current_ma;
    }
    /* In normal context - read from I2C directly */
    if (g_fuel_gauge_handle == NULL) {
        return 0;
    }
    return fuel_gauge_get_current(g_fuel_gauge_handle);
}

/* Enable/disable detailed diagnostics in monitor task */
#define POWER_MANAGE_ENABLE_DIAGNOSTICS    1    /* 1=enable, 0=disable */

/* Default BQ27220 CEDV parameters */
static const __attribute__((unused)) parameter_cedv_t default_bq27220_cedv = {
    .full_charge_cap = 650,
    .design_cap = 650,
    .reserve_cap = 0,
    .near_full = 200,
    .self_discharge_rate = 20,
    .EDV0 = 3490,
    .EDV1 = 3511,
    .EDV2 = 3535,
    .EMF = 3670,
    .C0 = 115,
    .R0 = 968,
    .T0 = 4547,
    .R1 = 4764,
    .TC = 11,
    .C1 = 0,
    .DOD0 = 4147,
    .DOD10 = 4002,
    .DOD20 = 3969,
    .DOD30 = 3938,
    .DOD40 = 3880,
    .DOD50 = 3824,
    .DOD60 = 3794,
    .DOD70 = 3753,
    .DOD80 = 3677,
    .DOD90 = 3574,
    .DOD100 = 3490,
};

/* Default BQ27220 gauging configuration */
static const __attribute__((unused)) gauging_config_t default_bq27220_config = {
    .CCT = 1,
    .CSYNC = 0,
    .EDV_CMP = 0,
    .SC = 1,
    .FIXED_EDV0 = 0,
    .FCC_LIM = 1,
    .FC_FOR_VDQ = 1,
    .IGNORE_SD = 1,
    .SME0 = 0,
};

/**
 * @brief IP5561 monitor task
 *
 * Periodically reads IP5561 register status to verify communication
 * on both I2C addresses (0xE8 and 0xEA)
 */
static void __attribute__((unused)) sensor_monitor_task(void *pvParameters);
static esp_err_t power_manage_apply_ip5561_config(void);  /* Forward decl for monitor reset recovery */
static void __attribute__((unused)) ip5561_monitor_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 monitor task started");

    uint8_t prev_sys_ctl0 = 0xFF;  // Previous value for change detection
    uint8_t prev_mos_status = 0xFF;
    int error_count_ctrl = 0;
    int error_count_stat = 0;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[IP5561 Monitor] Starting continuous monitoring (10s interval)");

    while (g_ip5561_task_running) {
        /* ========== Critical State Monitoring ========== */

        uint8_t sys_ctl0 = 0, sys_ctl1 = 0, mos_status = 0;
        esp_err_t ret_ctrl;

        /* Read SYS_CTL0 - Detect Boost state changes */
        ret_ctrl = ip5561_get_sys_ctl0(g_ip5561_handle, &sys_ctl0);
        if (ret_ctrl == ESP_OK) {
            // Detect boost state changes
            if ((sys_ctl0 & 0x02) != (prev_sys_ctl0 & 0x02)) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                         "[IP5561] ⚠️ BOOST STATE CHANGED: 0x%02X -> 0x%02X (Boost now %s)",
                         prev_sys_ctl0, sys_ctl0,
                         (sys_ctl0 & 0x02) ? "ENABLED" : "DISABLED");
            }
            prev_sys_ctl0 = sys_ctl0;
            error_count_ctrl = 0;
        } else {
            error_count_ctrl++;
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                     "[IP5561] Failed to read SYS_CTL0, error count: %d", error_count_ctrl);
        }

        /* Read SYS_CTL1 - Light load detection check & correction */
        ret_ctrl = ip5561_get_sys_ctl1(g_ip5561_handle, &sys_ctl1);
        if (ret_ctrl == ESP_OK) {
            /* Force disable light load detection if enabled */
            if ((sys_ctl1 & 0x04) || (sys_ctl1 & 0x08)) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                         "[IP5561] ⚠️ Light load detected! Forcing disable...");
                sys_ctl1 &= ~(0x04 | 0x08 | 0x30);
                ip5561_set_sys_ctl1(g_ip5561_handle, sys_ctl1);
                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                         "[IP5561] SYS_CTL1 corrected to: 0x%02X", sys_ctl1);
            }

            /* ========== Config Fingerprint Check ========== */
            /* Detect if IP5561 reset or configuration was lost */
            if (g_ip5561_fingerprint.initialized) {
                bool config_changed = false;
                uint8_t current_sys_ctl0 = 0, current_qc = 0, current_pd = 0, current_tmo = 0;

                ip5561_get_sys_ctl0(g_ip5561_handle, &current_sys_ctl0);
                ip5561_read_reg(g_ip5561_handle, 0x81, &current_qc, false);   /* QC_CTRL0 */
                ip5561_read_reg(g_ip5561_handle, 0xD4, &current_pd, false);   /* PD_CTRL */
                ip5561_read_reg(g_ip5561_handle, 0x21, &current_tmo, false);   /* CHG_TMO_CTL1 */

                /* Compare with saved fingerprint */
                if (current_sys_ctl0 != g_ip5561_fingerprint.sys_ctl0) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                             "[IP5561] ⚠️ CONFIG CHANGE! SYS_CTL0: 0x%02X -> 0x%02X (possible reset!)",
                             g_ip5561_fingerprint.sys_ctl0, current_sys_ctl0);
                    config_changed = true;
                }
                if (current_qc != g_ip5561_fingerprint.qc_ctrl0) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                             "[IP5561] ⚠️ CONFIG CHANGE! QC_CTRL0: 0x%02X -> 0x%02X",
                             g_ip5561_fingerprint.qc_ctrl0, current_qc);
                    config_changed = true;
                }
                if (current_pd != g_ip5561_fingerprint.pd_ctrl) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                             "[IP5561] ⚠️ CONFIG CHANGE! PD_CTRL: 0x%02X -> 0x%02X",
                             g_ip5561_fingerprint.pd_ctrl, current_pd);
                    config_changed = true;
                }
                if (current_tmo != g_ip5561_fingerprint.chg_tmo_ctl1) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                             "[IP5561] ⚠️ CONFIG CHANGE! CHG_TMO_CTL1: 0x%02X -> 0x%02X",
                             g_ip5561_fingerprint.chg_tmo_ctl1, current_tmo);
                    config_changed = true;
                }

                if (config_changed) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                             "[IP5561] 🔴 IP5561 RESET detected! Re-applying configuration...");

                    /* Read reset cause indicators before recovery */
                    uint8_t sys_state0 = 0, sys_state1 = 0;
                    ip5561_read_reg(g_ip5561_handle, 0xC4, &sys_state0, true);
                    ip5561_read_reg(g_ip5561_handle, 0xC5, &sys_state1, true);
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                             "[IP5561] Reset cause: SYS_STATE0=0x%02X [VBUSOV:%d VBUSOK:%d] SYS_STATE1=0x%02X [VBATLOW:%d VSYS_OV:%d]",
                             sys_state0, (sys_state0 >> 5) & 1, (sys_state0 >> 4) & 1,
                             sys_state1, (sys_state1 >> 2) & 1, (sys_state1 >> 1) & 1);

                    /* Full re-initialization of all IP5561 registers */
                    power_manage_apply_ip5561_config();
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                             "[IP5561] ✅ Configuration re-applied after reset");
                }
            }
        }

        /* Read MOSFET status - Detect state changes */
        ret_ctrl = ip5561_read_reg(g_ip5561_handle, 0x87, &mos_status, false);
        if (ret_ctrl == ESP_OK) {
            if ((mos_status & 0x01) != (prev_mos_status & 0x01)) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                         "[IP5561] ⚠️ MOSFET STATE CHANGED: 0x%02X -> 0x%02X (MOS now %s)",
                         prev_mos_status, mos_status,
                         (mos_status & 0x01) ? "ON" : "OFF");
            }
            prev_mos_status = mos_status;
        }

        /* ========== Error Summary ========== */
        if (error_count_ctrl >= 3 || error_count_stat >= 3) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                     "[IP5561] ⚠️ I2C ERROR! Ctrl: %d, Stat: %d",
                     error_count_ctrl, error_count_stat);
        }

        /* ========== Visual Separator ========== */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "------------------------------------------------");

        /* ========== VBUS Output Diagnostics ========== */
        {
            uint8_t mos_state = 0, ocp_state = 0, sys_state3 = 0;

            /* MOS_STATE (0xEB @ 0xEA): bit6=VBUS MOS, bit4=VOUT MOS */
            ip5561_read_reg(g_ip5561_handle, 0xEB, &mos_state, true);

            /* OCP_STATE (0xFC @ 0xEA): bit2=boost_uv, bit0=boost_scdt */
            ip5561_read_reg(g_ip5561_handle, 0xFC, &ocp_state, true);

            /* SYS_STATE3 (0xD0 @ 0xEA): bit4=Boost_en, bits2:0=sys_state */
            ip5561_read_reg(g_ip5561_handle, 0xD0, &sys_state3, true);

            /* SYS_STATE0 (0xC4 @ 0xEA): bit5=VBUSOV, bit4=VBUSOK */
            uint8_t sys_state0 = 0;
            ip5561_read_reg(g_ip5561_handle, 0xC4, &sys_state0, true);

            /* VBUS output current (0x5A-0x5B @ 0xEA) */
            uint8_t vbus_il = 0, vbus_ih = 0;
            ip5561_read_reg(g_ip5561_handle, 0x5A, &vbus_il, true);
            ip5561_read_reg(g_ip5561_handle, 0x5B, &vbus_ih, true);
            uint16_t vbus_iadc = ((uint16_t)vbus_ih << 8) | vbus_il;
            int16_t vbus_out_ma = (int16_t)(vbus_iadc * 0.671387f);

            /* VBUS voltage (0x62-0x63 @ 0xEA) */
            uint8_t vbus_vl = 0, vbus_vh = 0;
            ip5561_read_reg(g_ip5561_handle, 0x62, &vbus_vl, true);
            ip5561_read_reg(g_ip5561_handle, 0x63, &vbus_vh, true);
            uint16_t vbus_vadc = ((uint16_t)vbus_vh << 8) | vbus_vl;
            uint16_t vbus_out_mv = (uint16_t)(vbus_vadc * 1.611328f);

            bool vbus_mos_on = (mos_state & (1 << 6)) != 0;
            bool boost_en = (sys_state3 & (1 << 4)) != 0;
            uint8_t sys_state = sys_state3 & 0x07;

            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[IP5561] VBUS: MOS=%s Boost=%s Sys=%d V=%umV I=%dmA | MOS_REG=0x%02X OCP=0x%02X STATE0=0x%02X[VBUSOV:%d VBUSOK:%d]",
                vbus_mos_on ? "ON" : "OFF",
                boost_en ? "EN" : "DIS",
                sys_state,
                vbus_out_mv, vbus_out_ma,
                mos_state, ocp_state,
                sys_state0, (sys_state0 >> 5) & 1, (sys_state0 >> 4) & 1);

            /* ========== OCP Auto-Clear ========== */
            /* bit5 (0x20): reserved, ignore */
            /* bit2 (0x04): boost UV (overcurrent) */
            /* bit0 (0x01): boost short circuit */
            if (ocp_state & 0x05) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[IP5561] ⚠️ OCP triggered! (uv=%d, scdt=%d) Clearing...",
                    (ocp_state >> 2) & 1, ocp_state & 1);
                ip5561_write_reg(g_ip5561_handle, 0xFC, ocp_state | 0x05, true);
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            /* ========== VBUS MOS Auto-Recovery ========== */
            /* If boost is enabled but VBUS MOS went off, try to re-enable VBUS output */
            if (boost_en && !vbus_mos_on) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[IP5561] ⚠️ Boost enabled but VBUS MOS OFF! Attempting recovery...");
                ip5561_configure_vbus_output(g_ip5561_handle, true);
            }
        }

        /* Read every 10 seconds (1s granularity for quick stop response) */
        for (int i = 0; i < 10 && g_ip5561_task_running; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 monitor task stopped");
    g_ip5561_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief BQ27220 monitor task
 *
 * Periodically reads BQ27220 battery fuel gauge status and controls
 * Tiantong module power based on battery voltage thresholds:
 * - V_OFF: 3.5V - Turn off module when voltage drops below this
 * - V_ON:  3.6V - Turn on module when voltage rises above this
 * - Hysteresis: 0.1V to prevent oscillation
 */

/**
 * @brief Apply IR (Internal Resistance) compensation to battery voltage
 *
 * When charging, the terminal voltage is higher than the actual battery voltage
 * due to internal resistance (IR drop). This function compensates for this effect
 * to provide a more accurate representation of the battery's true state.
 *
 * Calculation:
 * - If charging: V_comp = V_term - (|I_chg| × R_internal)
 * - If discharging: V_comp = V_term (no compensation)
 *
 * @param voltage_mv Measured voltage in mV
 * @param current_ma Current in mA (negative for charging)
 * @param is_charging Output parameter: true if charging, false if discharging
 * @return Compensated voltage in mV
 */
static uint16_t apply_ir_compensation(uint16_t voltage_mv, int16_t current_ma, bool *is_charging)
{
    uint32_t ir_drop = 0;

    /* Determine charging state based on current */
    if (current_ma < -200) {
        /* Clearly charging: current < -200mA */
        *is_charging = true;
        ir_drop = ((-current_ma) * POWER_MANAGE_BATT_INTERNAL_R_MOHM) / 1000;
    } else if (current_ma > 200) {
        /* Clearly discharging: current > 200mA */
        *is_charging = false;
        ir_drop = 0;
    } else {
        /* Small current zone: assume discharging (conservative) */
        *is_charging = false;
        ir_drop = 0;
    }

    /* Limit IR compensation to prevent over-compensation */
    if (ir_drop > POWER_MANAGE_MAX_IR_COMP_MV) {
        ir_drop = POWER_MANAGE_MAX_IR_COMP_MV;
    }

    return voltage_mv - ir_drop;
}

/**
 * @brief Get compensated battery voltage for TT module power decision
 *
 * This function implements a hybrid approach to determine the "true" battery voltage:
 * - Uses BQ27220 DSG flag as primary charging state indicator
 * - Uses BQ27220 current sign as verification
 * - Uses IP5561 charging status as cross-check (if available)
 * - Applies IR compensation when charging
 * - Falls back to BQ27220-only mode if IP5561 communication fails
 *
 * @param[out] decision_voltage Pointer to store the compensated voltage for decision
 * @param[out] is_charging Pointer to store charging state (true=charging, false=discharging)
 * @return ESP_OK if successful, ESP_FAIL if both BQ27220 and IP5561 failed
 */
static esp_err_t get_battery_decision_voltage(uint16_t *decision_voltage, bool *is_charging)
{
    if (decision_voltage == NULL || is_charging == NULL) {
        return ESP_FAIL;
    }

    /* Check interrupt context early - use cached values if in ISR */
    if (xPortInIsrContext()) {
        if (!g_battery_cache.initialized) {
            return ESP_FAIL;
        }
        *decision_voltage = g_battery_cache.voltage_mv;
        *is_charging = g_battery_cache.is_charging;
        return ESP_OK;
    }

    esp_err_t ret;
    uint16_t voltage = 0;
    int16_t current = 0;
    bool fg_charging = false;
    bool ip5561_charging = false;
    bool ip5561_available = false;

    /* ========== Step 1: Read BQ27220 data (primary source) ========== */
    voltage = fuel_gauge_get_voltage(g_fuel_gauge_handle);
    if (voltage == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "BQ27220: Failed to read voltage");
        return ESP_FAIL;
    }

    current = fuel_gauge_get_current(g_fuel_gauge_handle);
    /* current > 0 means charging, < 0 means discharging */

    battery_status_t batt_status;
    ret = fuel_gauge_get_battery_status(g_fuel_gauge_handle, &batt_status);
    if (ret == ESP_OK) {
        /* Calculate BQ27220 charging state: prioritize current over DSG flag
         * Current is real-time, DSG flag may have lag during charge/discharge transitions
         */
        if (current < -200) {
            /* Clearly charging: current < -200mA */
            fg_charging = true;
        } else if (current > 200) {
            /* Clearly discharging: current > 200mA */
            fg_charging = false;
        } else {
            /* Small current zone (-200mA ~ +200mA): use DSG flag */
            fg_charging = !batt_status.DSG;
        }
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "BQ27220: V=%umV, I=%dmA, DSG=%d → Charging=%d",
        voltage, current, batt_status.DSG, fg_charging);

    /* ========== Step 2: Read IP5561 charging status (secondary source) ========== */
    if (g_ip5561_handle != NULL) {
        ip5561_charging = ip5561_is_charging(g_ip5561_handle);
        ip5561_available = true;  /* IP5561 communication successful */

        SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "IP5561: Charging=%d", ip5561_charging);
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561: Handle NULL, using BQ27220 only");
    }

    /* ========== Step 3: Determine charging state with cross-validation ========== */

    /* BQ27220 Current Sign Convention:
     * Negative (-) = Charging (current flows INTO battery)
     * Positive (+) = Discharging (current flows OUT of battery)
     *
     * Priority: Current > DSG > IP5561
     * Current is real-time physical quantity, most reliable
     * DSG flag may have lag or update delay
     * IP5561 only checks VBUS presence, not actual charging
     */
    if (ip5561_available) {
        /* Use current as primary indicator with hysteresis */
        bool current_charging;

        if (current < -200) {
            /* Clearly charging: current < -200mA */
            current_charging = true;
        } else if (current > 200) {
            /* Clearly discharging: current > 200mA */
            current_charging = false;
        } else {
            /* Small current zone (-200mA ~ +200mA): use DSG flag */
            current_charging = !batt_status.DSG;
            SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "State: Low current zone (I=%dmA), using DSG=%d", current, current_charging);
        }

        /* Triple verification for logging */
        if (current_charging && fg_charging && ip5561_charging) {
            /* All agree: charging */
            *is_charging = true;
            SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "State: CONFIRMED CHARGING (all 3 agree)");
        } else if (!current_charging && !fg_charging && !ip5561_charging) {
            /* All agree: discharging */
            *is_charging = false;
            SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "State: CONFIRMED DISCHARGING (all 3 agree)");
        } else {
            /* Status mismatch - trust current most */
            *is_charging = current_charging;
            SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "State MISMATCH! BQ27220_charging=%d (DSG=%d), I=%dmA, IP5561=%d → Using CURRENT (Charging=%d)",
                fg_charging, batt_status.DSG, current, ip5561_charging, current_charging);
        }
    } else {
        /* IP5561 not available - trust current */
        if (current < -200) {
            *is_charging = true;
        } else if (current > 200) {
            *is_charging = false;
        } else {
            /* Small current: use DSG flag */
            *is_charging = fg_charging;
            SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "State: Low current zone, using DSG=%d (IP5561 unavailable)", *is_charging);
        }
    }

    /* ========== Step 4: Apply IR compensation if charging ========== */
    *decision_voltage = apply_ir_compensation(voltage, current, is_charging);

    if (*is_charging) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "Charging IR compensation: V_term=%umV, I_chg=%dmA → V_comp=%umV",
            voltage, current, *decision_voltage);
    } else {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "Discharging: V_term=%umV (no IR compensation)", voltage);
    }

    return ESP_OK;
}

static void __attribute__((unused)) fuel_gauge_monitor_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge monitor task started (60s period)");

    int error_count = 0;
    uint16_t prev_coulomb_count = 0;
    bool first_read = true;

    while (g_fuel_gauge_task_running) {
        /* ========== Read all BQ27220 data ========== */
        uint16_t voltage = fuel_gauge_get_voltage(g_fuel_gauge_handle);
        if (voltage == 0) {
            error_count++;
            if (error_count > 3) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "BQ27220 read failed %d times", error_count);
            }
            for (int i = 0; i < 60 && g_fuel_gauge_task_running; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            continue;
        }

        error_count = 0;

        int16_t instant_current = fuel_gauge_get_current(g_fuel_gauge_handle);
        uint16_t coulomb_count = fuel_gauge_get_raw_coulomb_count(g_fuel_gauge_handle);
        uint16_t soc = fuel_gauge_get_state_of_charge(g_fuel_gauge_handle);
        int16_t avg_power = fuel_gauge_get_average_power(g_fuel_gauge_handle);

        /* ========== Update battery cache for interrupt context ========== */
        update_battery_cache();

        /* ========== Calculate average current from CoulombCounter delta ========== */
        int32_t avg_current_ma = 0;
        if (!first_read) {
            int32_t delta = (int32_t)coulomb_count - (int32_t)prev_coulomb_count;
            /* Delta > 0 = discharge (coulomb count increased)
             * Delta < 0 = charge (coulomb count decreased)
             * avg_current = delta_mAh * 3600 / interval_s
             * TODO: confirm CoulombCount unit, adjust if not 1 LSB = 1 mAh */
            avg_current_ma = delta * 3600 / 60;
        }
        first_read = false;
        prev_coulomb_count = coulomb_count;

        /* Cache average current for other modules to read
         * CoulombCount convention is opposite to Current(): negate to match
         * APP convention: discharge positive, charge negative */
        g_avg_current_ma = -(int16_t)avg_current_ma;

        /* ========== Print battery status ========== */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "[BATT] V=%umV, I_now=%dmA, I_avg=%dmA, SOC=%u%%, P_avg=%dmW, Coulomb=%u",
            voltage, instant_current, avg_current_ma, soc, avg_power, coulomb_count);

        /* ========== Low Battery TT Module Auto-Shutdown Check ========== */
        {
            extern bool tt_module_is_force_on(void);
            extern tt_state_t tt_module_get_state(void);
            extern esp_err_t tt_module_low_battery_shutdown(void);

#ifdef SUPPORT_HARDWARE_V2
            /* V2.0: Boost power arbitration with hysteresis (3.3V off / 3.4V on) */
            if (!g_mcu_low_battery_boost && voltage < LOW_BAT_BOOST_OFF_MV) {
                /* Battery critically low: MCU requests boost to stay alive */
                g_mcu_low_battery_boost = true;
                power_manage_boost_request(BOOST_CONSUMER_MCU);
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[BATT] Low battery %umV < %umV: MCU requesting boost for survival",
                    voltage, LOW_BAT_BOOST_OFF_MV);
            } else if (g_mcu_low_battery_boost && voltage >= LOW_BAT_BOOST_ON_MV) {
                /* Battery recovered: MCU releases boost */
                g_mcu_low_battery_boost = false;
                power_manage_boost_release(BOOST_CONSUMER_MCU);
                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[BATT] Battery recovered %umV >= %umV: MCU releasing boost",
                    voltage, LOW_BAT_BOOST_ON_MV);
            }
#endif

            if (!tt_module_is_force_on()) {
                tt_state_t tt_state = tt_module_get_state();

                if (tt_state == TT_STATE_WORKING && voltage < POWER_MANAGE_TT_MODULE_V_OFF_MV) {
                    SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                        "[BATT] Low battery: %umV < %umV, auto-shutdown TT module",
                        voltage, POWER_MANAGE_TT_MODULE_V_OFF_MV);
                    tt_module_low_battery_shutdown();
                }
                else if (tt_state == TT_STATE_LOW_BATTERY_OFF && voltage >= POWER_MANAGE_TT_MODULE_V_ON_MV) {
                    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                        "[BATT] Battery recovered: %umV >= %umV, user can restart TT",
                        voltage, POWER_MANAGE_TT_MODULE_V_ON_MV);
                }
            } else {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[BATT] Force ON active, skip low battery check (%umV)", voltage);
            }
        }

        /* Print BLE TX power */
        {
            int8_t ble_pwr = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[BATT] BLE TX power: %d dBm", ble_pwr);
        }

        /* Wait 60 seconds before next check (1s granularity for quick stop response) */
        for (int i = 0; i < 60 && g_fuel_gauge_task_running; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge monitor task exiting");
    g_fuel_gauge_task_handle = NULL;
    vTaskDelete(NULL);
}

int16_t power_manage_get_avg_current(void)
{
    if (g_avg_current_ma == 0 && g_fuel_gauge_handle != NULL) {
        return fuel_gauge_get_current(g_fuel_gauge_handle);
    }
    return g_avg_current_ma;
}

/* ========== Independent Device Initialization Functions ========== */

/**
 * @brief Apply full IP5561 register configuration and save fingerprint.
 *
 * Called during init and after reset recovery to re-apply all settings.
 */
static esp_err_t power_manage_apply_ip5561_config(void)
{
    /* WPC: disabled (not used, will be developed separately) */
    ip5561_set_wpc_enable(g_ip5561_handle, false);

    /* NTC: disabled (thermistor not connected on current board) */
    ip5561_disable_ntc(g_ip5561_handle);

    /* VOUT: disabled (unused port, save power) */
    ip5561_disable_vout(g_ip5561_handle);

    /* Light load detection: disabled (prevent auto-shutdown of outputs) */
    ip5561_configure_light_load(g_ip5561_handle, false, false, 0);

    /* Charge voltage and current */
    ip5561_set_charge_voltage(g_ip5561_handle, 4350);
    ip5561_set_5v_charge_current(g_ip5561_handle, 3500);
    ip5561_set_9v_charge_current(g_ip5561_handle, 1500);
    ip5561_set_9v_uv_threshold(g_ip5561_handle, 7500);

    /* Input fast charge: enable QC/FCP/AFC/PD */
    ip5561_configure_vbus_input(g_ip5561_handle, true);

    /* VBUS boost output: enabled with QC/PD fast charge support */
    ip5561_configure_vbus_output(g_ip5561_handle, true);

    /* VBUS output current limits (50mA * N, 7-bit)
     *   5V: N=60 → 3000mA (calibrated max ~3.3A)
     *   9V: N=40 → 2000mA (calibrated max ~2.3A) */
    ip5561_write_reg(g_ip5561_handle, 0xB9, 60, false);   /* VBUS_5V */
    ip5561_write_reg(g_ip5561_handle, 0xBB, 40, false);   /* VBUS_9V */

    ip5561_disable_boost_protections(g_ip5561_handle, false);

    /* Save configuration fingerprint (re-read AFTER all config applied) */
    uint8_t sys_ctl0 = 0, sys_ctl1 = 0;
    uint8_t qc_ctrl0 = 0, pd_ctrl = 0, chg_tmo = 0;
    ip5561_get_sys_ctl0(g_ip5561_handle, &sys_ctl0);
    ip5561_get_sys_ctl1(g_ip5561_handle, &sys_ctl1);
    ip5561_read_reg(g_ip5561_handle, 0x81, &qc_ctrl0, false);
    ip5561_read_reg(g_ip5561_handle, 0xD4, &pd_ctrl, false);
    ip5561_read_reg(g_ip5561_handle, 0x21, &chg_tmo, false);

    g_ip5561_fingerprint.sys_ctl0 = sys_ctl0;
    g_ip5561_fingerprint.sys_ctl1 = sys_ctl1;
    g_ip5561_fingerprint.qc_ctrl0 = qc_ctrl0;
    g_ip5561_fingerprint.pd_ctrl = pd_ctrl;
    g_ip5561_fingerprint.chg_tmo_ctl1 = chg_tmo;
    g_ip5561_fingerprint.initialized = true;

    return ESP_OK;
}

/**
 * @brief Initialize IP5561 device independently
 *
 * @return ESP_OK on success, ESP_FAIL on failure (non-fatal, allows other devices to init)
 */
static esp_err_t power_manage_init_ip5561(void)
{
    esp_err_t ret;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "=== Initializing IP5561 (Charging Manager) ===");

    /* Wakeup IP5561 before creating device */
    ip5561_wakeup_sequence();

    /* Create IP5561 device */
    ip5561_config_t ip5561_cfg = {
#ifdef SUPPORT_HARDWARE_V2
        .i2c_bus = g_i2c_bus_handle_ip5561,
#else
        .i2c_bus = g_i2c_bus_handle,
#endif
        .scl_freq_hz = POWER_I2C_FREQ_HZ,
    };

    g_ip5561_handle = ip5561_create(&ip5561_cfg);
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561: Failed to create device");
        return ESP_FAIL;
    }

    /* Register wakeup callback (auto-wakeup before each I2C communication) */
    ip5561_set_wakeup_callback(g_ip5561_handle, ip5561_wakeup_sequence);

    /* Test IP5561 communication */
    uint8_t sys_ctl0 = 0;
    ret = ip5561_get_sys_ctl0(g_ip5561_handle, &sys_ctl0);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561: Communication test failed");
        ip5561_delete(g_ip5561_handle);
        g_ip5561_handle = NULL;
        return ESP_FAIL;
    }
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561: Communication OK, SYS_CTL0: 0x%02X", sys_ctl0);

    /* Apply full register configuration */
    power_manage_apply_ip5561_config();

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561: ✓ Initialization complete");
    return ESP_OK;
}

/**
 * @brief Initialize BQ27220 device independently
 *
 * @return ESP_OK on success, ESP_FAIL on failure (non-fatal, allows other devices to init)
 */
static esp_err_t power_manage_init_fuel_gauge(void)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "=== Initializing Fuel Gauge (auto-detect BQ27220/CW2217E) ===");

    fuel_gauge_config_t fg_cfg = {
        .i2c_bus = g_i2c_bus_handle,
        .gauging_config = &default_bq27220_config,
        .cedv_params = &default_bq27220_cedv,
    };

    g_fuel_gauge_handle = fuel_gauge_create(&fg_cfg);
    if (g_fuel_gauge_handle == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge: No device detected (probed 0x55 and 0x64)");
        return ESP_FAIL;
    }

    /* Test communication with retry */
    uint16_t voltage = 0;
    for (int i = 0; i < 3; i++) {
        voltage = fuel_gauge_get_voltage(g_fuel_gauge_handle);
        if (voltage > 0) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge: ✓ Communication OK, Voltage: %u mV", voltage);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* All retries failed */
    SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge: Communication failed after 3 retries");
    fuel_gauge_delete(g_fuel_gauge_handle);
    g_fuel_gauge_handle = NULL;
    return ESP_FAIL;
}

esp_err_t power_manage_init(void)
{
    esp_err_t ret;
    bool ip5561_ok = false;
    bool fuel_gauge_ok = false;

    if (g_i2c_bus_handle != NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power management already initialized");
        return ESP_OK;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "========================================");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power Management Initialization Start (%s)", HW_VERSION_STRING);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "========================================");

    /* Step 1: Initialize IP5561 wakeup GPIO */
    ip5561_wakeup_gpio_init();
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 wakeup GPIO initialized (GPIO%d, %dms pulse)",
                    IP5561_WAKEUP_GPIO, IP5561_WAKEUP_DELAY_MS);

#ifdef SUPPORT_HARDWARE_V2
    /* V2.0: Initialize boost IC GPIOs */
    boost_gpio_init();
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Boost IC GPIOs initialized (EN: GPIO%d, MODE: GPIO%d)",
                    BOOST_PWR_EN_GPIO, BOOST_PWR_MODE_GPIO);
#endif

    /* Step 2: Initialize I2C bus (fuel gauge + sensors) */
    i2c_master_bus_config_t bus_config = {
        .i2c_port = POWER_I2C_PORT_NUM,
        .sda_io_num = POWER_I2C_SDA_IO,
        .scl_io_num = POWER_I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ret = i2c_new_master_bus(&bus_config, &g_i2c_bus_handle);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "I2C bus0 initialized (SCL: GPIO%d, SDA: GPIO%d)",
                    POWER_I2C_SCL_IO, POWER_I2C_SDA_IO);

#ifdef SUPPORT_HARDWARE_V2
    /* V2.0: Initialize second I2C bus for IP5561 */
    i2c_master_bus_config_t ip5561_bus_config = {
        .i2c_port = IP5561_I2C_PORT_NUM,
        .sda_io_num = IP5561_I2C_SDA_IO,
        .scl_io_num = IP5561_I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ret = i2c_new_master_bus(&ip5561_bus_config, &g_i2c_bus_handle_ip5561);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to initialize IP5561 I2C bus: %s", esp_err_to_name(ret));
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "I2C bus1 initialized for IP5561 (SCL: GPIO%d, SDA: GPIO%d)",
                        IP5561_I2C_SCL_IO, IP5561_I2C_SDA_IO);
    }
#endif

    /* Step 3: Initialize IP5561 (independent, non-blocking) */
    ret = power_manage_init_ip5561();
    if (ret == ESP_OK) {
        ip5561_ok = true;
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 initialization failed - continuing without it");
    }

    /* Step 4: Initialize Fuel Gauge (auto-detect BQ27220/CW2217E, non-blocking) */
    ret = power_manage_init_fuel_gauge();
    if (ret == ESP_OK) {
        fuel_gauge_ok = true;
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge initialization failed - continuing without it");
    }

    /* Step 5: Check if at least one device initialized successfully */
    if (!ip5561_ok && !fuel_gauge_ok) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "========================================");
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "CRITICAL: Both devices failed to init!");
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "========================================");
        i2c_del_master_bus(g_i2c_bus_handle);
        g_i2c_bus_handle = NULL;
        return ESP_FAIL;
    }

    /* Step 6: Print initialization summary */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "========================================");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power Management Init Summary:");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  IP5561 (Charging):  %s", ip5561_ok ? "OK" : "FAIL");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  Fuel Gauge (Batt):  %s", fuel_gauge_ok ? "OK" : "FAIL");

    /* Step 7: Initialize accelerometer (DA228EC primary, SC7A20H fallback) */
    {
        da228ec_config_t da228ec_cfg = {
            .i2c_bus = g_i2c_bus_handle,
            .scl_freq_hz = POWER_I2C_FREQ_HZ,
        };
        g_da228ec_handle = da228ec_create(&da228ec_cfg);
        if (g_da228ec_handle != NULL) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  DA228EC (Accel):    OK");
        } else {
            /* Try SC7A20H as second source */
            sc7a20h_config_t sc7a20h_cfg = {
                .i2c_bus = g_i2c_bus_handle,
                .scl_freq_hz = POWER_I2C_FREQ_HZ,
            };
            g_sc7a20h_handle = sc7a20h_create(&sc7a20h_cfg);
            if (g_sc7a20h_handle != NULL) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  SC7A20H (Accel):   OK");
            } else {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  Accel:             not found");
            }
        }
    }

    /* Step 8: Initialize MMC5603 magnetometer (dynamic detect) */
    {
        mmc5603_config_t mmc5603_cfg = {
            .i2c_bus = g_i2c_bus_handle,
            .scl_freq_hz = POWER_I2C_FREQ_HZ,
        };
        g_mmc5603_handle = mmc5603_create(&mmc5603_cfg);
        if (g_mmc5603_handle != NULL) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  MMC5603 (Mag):      OK");
        } else {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "  MMC5603 (Mag):      not found");
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "========================================");

    /* Start sensor monitor task if any sensor detected */
    if (g_da228ec_handle != NULL || g_sc7a20h_handle != NULL || g_mmc5603_handle != NULL) {
        xTaskCreate(sensor_monitor_task, "sensor_mon", 3072, NULL, 1, NULL);
    }

    return ESP_OK;
}

/**
 * @brief Shared sensor monitor task (DA228EC + MMC5603)
 *
 * Reads sensors, caches data, sends BLE notification if report enabled.
 * Sensor data is read every 500ms; debug log printed every 30s.
 */

/* Debug calculation toggles (for hardware comparison testing) */
#define SENSOR_DEBUG_LOG_INTERVAL_MS   30000   /* Debug log interval (30s) */
#define SENSOR_DEBUG_TILT_ANGLE        1       /* 1=calculate and log tilt angle */
#define SENSOR_DEBUG_COMPASS_HEADING   1       /* 1=calculate and log compass heading */

#if SENSOR_DEBUG_TILT_ANGLE
/* Calculate tilt angle (仰角/pitch) from accelerometer data.
 * Returns signed angle in degrees:
 *   0° = flat, positive = tilted up, negative = tilted down
 * Range: -90° ~ +90°
 * ax/ay/az in mg. */
static float sensor_calc_tilt(int16_t ax, int16_t ay, int16_t az)
{
    float x = ax, y = ay, z = az;
    /* Pitch around X axis: atan2(ay, sqrt(ax² + az²))
     * Uses Y axis as tilt direction (front edge up/down) */
    return atan2f(y, sqrtf(x * x + z * z)) * 180.0f / 3.14159265f;
}
#endif

#if SENSOR_DEBUG_COMPASS_HEADING
/* Calculate compass heading from magnetometer data.
 * Returns heading in degrees (0=North, 90=East, 180=South, 270=West).
 * mx/my/mz in mG. */
static int16_t sensor_calc_heading(int32_t mx, int32_t my, int32_t mz)
{
    (void)mz;  /* 2D compass for simplicity */
    float heading = atan2f((float)my, (float)mx) * 180.0f / 3.14159265f;
    if (heading < 0) heading += 360.0f;
    return (int16_t)heading;
}
#endif

static void sensor_monitor_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Sensor monitor task started");

    uint32_t log_counter = 0;  /* Counts 500ms ticks; log every 60 ticks (30s) */

    while (1) {
        uint8_t flags = 0;

        if (g_da228ec_handle != NULL) {
            int16_t x, y, z;
            if (da228ec_read_xyz(g_da228ec_handle, &x, &y, &z) == ESP_OK) {
                g_sensor_cache.ax = x;
                g_sensor_cache.ay = y;
                g_sensor_cache.az = z;
                flags |= 0x01;
            }
        } else if (g_sc7a20h_handle != NULL) {
            int16_t x, y, z;
            if (sc7a20h_read_xyz(g_sc7a20h_handle, &x, &y, &z) == ESP_OK) {
                g_sensor_cache.ax = x;
                g_sensor_cache.ay = y;
                g_sensor_cache.az = z;
                flags |= 0x01;
            }
        }

        if (g_mmc5603_handle != NULL) {
            int32_t x, y, z;
            if (mmc5603_read_xyz(g_mmc5603_handle, &x, &y, &z) == ESP_OK) {
                g_sensor_cache.mx = x;
                g_sensor_cache.my = y;
                g_sensor_cache.mz = z;
                flags |= 0x02;
            }
        }

        g_sensor_cache.flags = flags;

        /* Debug log (every 30s, not every 500ms) */
        if (++log_counter >= (SENSOR_DEBUG_LOG_INTERVAL_MS / 500)) {
            log_counter = 0;

#if SENSOR_DEBUG_TILT_ANGLE && SENSOR_DEBUG_COMPASS_HEADING
            float tilt = sensor_calc_tilt(g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az);
            int16_t heading = sensor_calc_heading(g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[SENSOR] accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG | tilt=%.1f° heading=%d°",
                g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az,
                g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz,
                tilt, heading);
#elif SENSOR_DEBUG_TILT_ANGLE
            float tilt = sensor_calc_tilt(g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[SENSOR] accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG | tilt=%.1f°",
                g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az,
                g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz, tilt);
#elif SENSOR_DEBUG_COMPASS_HEADING
            int16_t heading = sensor_calc_heading(g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[SENSOR] accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG | heading=%d°",
                g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az,
                g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz, heading);
#else
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[SENSOR] accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG",
                g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az,
                g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz);
#endif
        }

        /* Send BLE notification if report enabled (every cycle, not throttled) */
        if (g_sensor_cache.report_enabled && flags) {
            gatt_system_server_send_sensor_data();
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

uint8_t power_manage_get_sensor_flags(void)
{
    uint8_t flags = 0;
    if (g_da228ec_handle != NULL || g_sc7a20h_handle != NULL) flags |= 0x01;
    if (g_mmc5603_handle != NULL) flags |= 0x02;
    return flags;
}

void power_manage_get_sensor_data(uint8_t *flags,
                                   int16_t *ax, int16_t *ay, int16_t *az,
                                   int32_t *mx, int32_t *my, int32_t *mz)
{
    if (flags) *flags = g_sensor_cache.flags;
    if (ax) *ax = g_sensor_cache.ax;
    if (ay) *ay = g_sensor_cache.ay;
    if (az) *az = g_sensor_cache.az;
    if (mx) *mx = g_sensor_cache.mx;
    if (my) *my = g_sensor_cache.my;
    if (mz) *mz = g_sensor_cache.mz;
}

void power_manage_set_sensor_report(bool enable)
{
    g_sensor_cache.report_enabled = enable;
}

bool power_manage_is_sensor_report_enabled(void)
{
    return g_sensor_cache.report_enabled;
}

esp_err_t power_manage_deinit(void)
{
    if (g_i2c_bus_handle == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power management not initialized");
        return ESP_OK;
    }

    /* Stop monitor tasks if running */
    power_manage_task_stop();

    /* Delete IP5561 device */
    if (g_ip5561_handle != NULL) {
        ip5561_delete(g_ip5561_handle);
        g_ip5561_handle = NULL;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 device deleted");
    }

    /* Delete fuel gauge device */
    if (g_fuel_gauge_handle != NULL) {
        fuel_gauge_delete(g_fuel_gauge_handle);
        g_fuel_gauge_handle = NULL;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge device deleted");
    }

    /* Delete DA228EC device */
    if (g_da228ec_handle != NULL) {
        da228ec_delete(g_da228ec_handle);
        g_da228ec_handle = NULL;
    }

    /* Delete SC7A20H device */
    if (g_sc7a20h_handle != NULL) {
        sc7a20h_delete(g_sc7a20h_handle);
        g_sc7a20h_handle = NULL;
    }

    /* Delete MMC5603 device */
    if (g_mmc5603_handle != NULL) {
        mmc5603_delete(g_mmc5603_handle);
        g_mmc5603_handle = NULL;
    }

    /* Delete I2C bus */
    esp_err_t ret = i2c_del_master_bus(g_i2c_bus_handle);
    if (ret == ESP_OK) {
        g_i2c_bus_handle = NULL;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "I2C bus deleted");
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Power management deinitialized");
    return ret;
}

esp_err_t power_manage_task_start(void)
{
    if (g_ip5561_handle == NULL && g_fuel_gauge_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "No devices initialized");
        return ESP_ERR_INVALID_ARG;
    }

    /* Start IP5561 monitor task */
    if (g_ip5561_handle != NULL && g_ip5561_task_handle == NULL) {
        g_ip5561_task_running = true;
        BaseType_t ret = xTaskCreate(
            ip5561_monitor_task,
            "ip5561_monitor",
            4096,
            NULL,
            5,  /* Priority */
            &g_ip5561_task_handle
        );

        if (ret != pdPASS) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to create IP5561 monitor task");
            g_ip5561_task_running = false;
            return ESP_FAIL;
        }
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 monitor task started");
    }

    /* Start fuel gauge monitor task */
    if (g_fuel_gauge_handle != NULL && g_fuel_gauge_task_handle == NULL) {
        g_fuel_gauge_task_running = true;
        BaseType_t ret = xTaskCreate(
            fuel_gauge_monitor_task,
            "fuel_gauge_monitor",
            4096,
            NULL,
            5,  /* Priority */
            &g_fuel_gauge_task_handle
        );

        if (ret != pdPASS) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to create fuel gauge monitor task");
            g_fuel_gauge_task_running = false;
            /* Continue without fuel gauge monitor */
        } else {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge monitor task started");
        }
    }

    return ESP_OK;
}

esp_err_t power_manage_task_stop(void)
{
    /* Stop IP5561 monitor task */
    if (g_ip5561_task_handle != NULL) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Stopping IP5561 monitor task...");
        g_ip5561_task_running = false;

        int timeout = 20;  /* 2 second timeout */
        while (g_ip5561_task_handle != NULL && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (g_ip5561_task_handle != NULL) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 monitor task stop timeout");
        } else {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 monitor task stopped");
        }
    }

    /* Stop fuel gauge monitor task */
    if (g_fuel_gauge_task_handle != NULL) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Stopping fuel gauge monitor task...");
        g_fuel_gauge_task_running = false;

        int timeout = 20;  /* 2 second timeout */
        while (g_fuel_gauge_task_handle != NULL && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (g_fuel_gauge_task_handle != NULL) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge monitor task stop timeout");
        } else {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Fuel gauge monitor task stopped");
        }
    }

    return ESP_OK;
}

ip5561_handle_t power_manage_get_ip5561_handle(void)
{
    return g_ip5561_handle;
}

fuel_gauge_handle_t power_manage_get_fuel_gauge_handle(void)
{
    return g_fuel_gauge_handle;
}

i2c_master_bus_handle_t power_manage_get_i2c_bus(void)
{
    return g_i2c_bus_handle;
}

da228ec_handle_t power_manage_get_da228ec_handle(void)
{
    return g_da228ec_handle;
}

mmc5603_handle_t power_manage_get_mmc5603_handle(void)
{
    return g_mmc5603_handle;
}

bool power_manage_task_is_running(void)
{
    return g_ip5561_task_running || g_fuel_gauge_task_running;
}

/* ========== Wireless Charger Control Functions ========== */

esp_err_t power_manage_set_wpc_enable(bool enable)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ip5561_set_wpc_enable(g_ip5561_handle, enable);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to %s wireless charger: %s",
                 enable ? "enable" : "disable", esp_err_to_name(ret));
    } else {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Wireless charger %s", enable ? "enabled" : "disabled");
    }

    return ret;
}

bool power_manage_get_wpc_enable(void)
{
    if (g_ip5561_handle == NULL) {
        return false;
    }

    return ip5561_get_wpc_enable(g_ip5561_handle);
}

esp_err_t power_manage_get_wpc_status(ip5561_wpc_status_t *status)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    if (status == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Invalid status pointer");
        return ESP_ERR_INVALID_ARG;
    }

    return ip5561_get_wpc_status(g_ip5561_handle, status);
}

bool power_manage_is_wpc_present(void)
{
    if (g_ip5561_handle == NULL) {
        return false;
    }

    return ip5561_is_wpc_present(g_ip5561_handle);
}

bool power_manage_is_wpc_charging(void)
{
    if (g_ip5561_handle == NULL) {
        return false;
    }

    return ip5561_is_wpc_charging(g_ip5561_handle);
}

/* ========== PowerBank API (IP5561) ========== */

esp_err_t power_manage_get_work_mode(uint8_t *mode, uint8_t *status_flags)
{
    if (mode == NULL || status_flags == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Read SYS_CTL0 register (0xE8) */
    uint8_t sys_ctl0 = 0;
    esp_err_t ret = ip5561_read_reg(g_ip5561_handle, 0x00, &sys_ctl0, false);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to read SYS_CTL0");
        return ret;
    }

    /* Determine mode from SYS_CTL0 bits
     * Bit 0: EN_CHARGER - Charger enable
     * Bit 1: EN_BOOST   - Boost output enable
     */
    bool charge_enabled = (sys_ctl0 & 0x01) != 0;
    bool boost_enabled = (sys_ctl0 & 0x02) != 0;

    if (charge_enabled) {
        *mode = POWER_MANAGE_MODE_CHARGE;  /* Charging mode (VBUS input) */
    } else if (boost_enabled) {
        *mode = POWER_MANAGE_MODE_BOOST;   /* Boost mode (VBUS output) */
    } else {
        *mode = POWER_MANAGE_MODE_STANDBY; /* Standby mode */
    }

    /* Build status flags */
    *status_flags = 0;
    if (charge_enabled) {
        *status_flags |= 0x01;  /* bit0: charge_enabled */
    }
    if (boost_enabled) {
        *status_flags |= 0x02;  /* bit1: boost_enabled */
    }

    /* Check VBUS present */
    ip5561_chg_status_t chg_status;
    if (ip5561_get_charge_status(g_ip5561_handle, &chg_status) == ESP_OK) {
        if (chg_status.vbus_valid) {
            *status_flags |= 0x04;  /* bit2: vbus_present */
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Work mode: %d, flags: 0x%02X", *mode, *status_flags);
    return ESP_OK;
}

esp_err_t power_manage_get_charging_status(uint8_t *status_flags)
{
    if (status_flags == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ip5561_chg_status_t chg_status;
    esp_err_t ret = ip5561_get_charge_status(g_ip5561_handle, &chg_status);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to get charge status");
        return ret;
    }

    /* Read SYS_CTL0 to get charger/boost enable status */
    uint8_t sys_ctl0 = 0;
    ip5561_get_sys_ctl0(g_ip5561_handle, &sys_ctl0);

    /* Build status flags
     * bit0: charger_enabled
     * bit1: boost_enabled
     * bit2: charging
     * bit3: discharging
     * bit4: charge_done
     */
    *status_flags = 0;
    if (sys_ctl0 & 0x01) {
        *status_flags |= 0x01;  /* bit0: charger_enabled */
    }
    if (sys_ctl0 & 0x02) {
        *status_flags |= 0x02;  /* bit1: boost_enabled */
    }
    if (chg_status.charging) {
        *status_flags |= 0x04;  /* bit2: charging */
    }
    if (chg_status.vbus_valid && (sys_ctl0 & 0x02)) {
        *status_flags |= 0x08;  /* bit3: discharging (boost active with load) */
    }
    if (chg_status.chg_done) {
        *status_flags |= 0x10;  /* bit4: charge_done */
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Charging status: 0x%02X", *status_flags);
    return ESP_OK;
}

esp_err_t power_manage_get_wireless_status(uint8_t *status_flags)
{
    if (status_flags == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ip5561_wpc_status_t wpc_status;
    esp_err_t ret = ip5561_get_wpc_status(g_ip5561_handle, &wpc_status);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to get WPC status");
        return ret;
    }

    /* Build status flags
     * bit0: wpc_enabled
     * bit1: wpc_present
     * bit2: wpc_charging
     * bit3: wpc_done
     */
    *status_flags = 0;
    if (wpc_status.wpc_enabled) {
        *status_flags |= 0x01;
    }
    if (wpc_status.wpc_present) {
        *status_flags |= 0x02;
    }
    if (wpc_status.wpc_charging) {
        *status_flags |= 0x04;
    }
    if (wpc_status.wpc_done) {
        *status_flags |= 0x08;
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Wireless status: 0x%02X", *status_flags);
    return ESP_OK;
}

esp_err_t power_manage_get_vbus_adc(uint16_t *vbus_voltage, int16_t *vbus_current)
{
    if (vbus_voltage == NULL || vbus_current == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Get VBUS voltage (scaled: raw * 1.611328 mV) */
    uint16_t vbus_adc = ip5561_get_vbus_voltage(g_ip5561_handle);
    *vbus_voltage = (uint16_t)(vbus_adc * 1.611328);

    /* Get VBUS current (scaled: raw * 0.671387 mA, signed) */
    int16_t ibus_adc = ip5561_get_ibus_current(g_ip5561_handle);
    *vbus_current = (int16_t)(ibus_adc * 0.671387);

    SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "VBUS ADC: V=%u mV, I=%d mA", *vbus_voltage, *vbus_current);
    return ESP_OK;
}

esp_err_t power_manage_get_ntc_data(uint16_t *ntc_voltage, int16_t *temperature)
{
    if (ntc_voltage == NULL || temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Get NTC temperature */
    uint16_t ntc_adc = 0;
    int16_t ntc_temp_c = 0;
    esp_err_t ret = ip5561_get_ntc_temperature(g_ip5561_handle, &ntc_temp_c, &ntc_adc);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to get NTC temperature");
        return ret;
    }

    /* Convert ADC to voltage (scaled: raw * 0.26855 mV) */
    *ntc_voltage = (uint16_t)(ntc_adc * 0.26855);

    /* Temperature is in 0.1°C units for API compatibility */
    *temperature = ntc_temp_c * 10;

    SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "NTC: V=%u mV, T=%d°C", *ntc_voltage, ntc_temp_c);
    return ESP_OK;
}

esp_err_t power_manage_set_charge_voltage(uint16_t voltage_mv)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Validate voltage range (4200-4400 mV) */
    if (voltage_mv < 4200 || voltage_mv > 4400) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Invalid charge voltage: %u mV (range: 4200-4400)", voltage_mv);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ip5561_set_charge_voltage(g_ip5561_handle, voltage_mv);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Charge voltage set to %u mV", voltage_mv);
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to set charge voltage: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t power_manage_set_charge_current_9v(uint16_t current_ma)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Validate current range (500-3500 mA) */
    if (current_ma < 500 || current_ma > 3500) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Invalid 9V charge current: %u mA (range: 500-3500)", current_ma);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ip5561_set_9v_charge_current(g_ip5561_handle, current_ma);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "9V charge current set to %u mA", current_ma);
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to set 9V charge current: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t power_manage_set_uv_threshold_9v(uint16_t threshold_mv)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Validate threshold range (6000-9000 mV) */
    if (threshold_mv < 6000 || threshold_mv > 9000) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Invalid 9V UV threshold: %u mV (range: 6000-9000)", threshold_mv);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ip5561_set_9v_uv_threshold(g_ip5561_handle, threshold_mv);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "9V UV threshold set to %u mV", threshold_mv);
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to set 9V UV threshold: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t power_manage_set_vbus_output_current_9v(uint16_t current_ma)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Validate current range (500-2000 mA) */
    if (current_ma < 500 || current_ma > 2000) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Invalid VBUS 9V output current: %u mA (range: 500-2000)", current_ma);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ip5561_set_vbus_9v_current(g_ip5561_handle, current_ma);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "VBUS 9V output current set to %u mA", current_ma);
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to set VBUS 9V output current: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t power_manage_set_wireless_charging(bool enable)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ip5561_set_wpc_enable(g_ip5561_handle, enable);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Wireless charging %s", enable ? "enabled" : "disabled");
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to set wireless charging: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t power_manage_ip5561_print_diagnostics(void)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t reg_value;
    uint16_t adc_value;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "==================== IP5561 DIAGNOSTICS ====================");

    /* ========== System Control Registers ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x00, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "SYS_CTL0 (0x00): 0x%02X [ChgEN:%d|BoostEN:%d|WPCEN:%d]",
            reg_value,
            (reg_value & 0x01) ? 1 : 0,
            (reg_value & 0x02) ? 1 : 0,
            (reg_value & 0x04) ? 1 : 0);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x03, &reg_value, true);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "SYS_CTL1 (0x03): 0x%02X [ILOW_CUR:%d|ILOW_PWR:%d|ILOW_TMO:%d]",
            reg_value,
            (reg_value & 0x04) ? 1 : 0,
            (reg_value & 0x08) ? 1 : 0,
            (reg_value & 0x30) >> 4);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0xE9, &reg_value, true);
    if (ret == ESP_OK) {
        /* Parse charge state (bits 6:4) */
        uint8_t chg_state = (reg_value >> 4) & 0x07;
        const char *chg_state_str;
        switch (chg_state) {
            case 0x0: chg_state_str = "Idle"; break;
            case 0x1: chg_state_str = "Pre-charge"; break;
            case 0x2: chg_state_str = "CC (Const Current)"; break;
            case 0x3: chg_state_str = "CV (Const Voltage)"; break;
            case 0x5: chg_state_str = "Charge Complete"; break;
            default:  chg_state_str = "Unknown"; break;
        }

        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "CHG_STATE2 (0xE9): 0x%02X [VBUS:%d|ChgDone:%d|Chg:%d|State:%s(0x%X)]",
            reg_value,
            (reg_value & 0x01) ? 1 : 0,
            (reg_value & 0x02) ? 1 : 0,
            (reg_value & 0x08) ? 1 : 0,
            chg_state_str,
            chg_state);
    }

    /* ========== SYS_CTL3 (System Control 3) ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x25, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "SYS_CTL3 (0x25): 0x%02X", reg_value);
    }

    /* ========== Status Registers ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x03, &reg_value, true);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "BATLOW (0x03): 0x%02X [Battery Low Voltage Threshold]",
            reg_value);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x19, &reg_value, true);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "GPIO_20UA_EN (0x19): 0x%02X [GPIO 20uA Enable]", reg_value);
    }

    /* ========== MOSFET Status ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x87, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "MOS_STATUS (0x87): 0x%02X [ChgFet:%d|DsgFet:%d]",
            reg_value,
            (reg_value & 0x01) ? 1 : 0,
            (reg_value & 0x02) ? 1 : 0);
    }

    /* ========== NTC Protection (NTC_CTL1 Register) ==========
     * Datasheet: I2C 0xE8, Register 0xFD
     * Bit 7: En_chg_ml  - Charge mid-low temp (5°C) current reduction
     * Bit 6: En_chg_mh  - Charge mid-high temp (41°C) current reduction
     * Bit 5: En_boost_lt - Boost low temp (-20°C) protection [RESET=1]
     * Bit 4: En_boost_ht - Boost high temp (60°C) protection [RESET=1]
     * Bit 3: En_chg_lt   - Charge low temp (0°C) protection [RESET=1]
     * Bit 2: En_chg_ht   - Charge high temp (45°C) protection [RESET=1]
     * Bit 1: Reserved
     * Bit 0: En_ntc1     - NTC1 enable
     */
    ret = ip5561_read_reg(g_ip5561_handle, 0xFD, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "NTC_CTL1 (0xFD): 0x%02X [ChgML:%d|ChgMH:%d|BoostLT:%d|BoostHT:%d|ChgLT:%d|ChgHT:%d|NTC1:%d]",
            reg_value,
            (reg_value & 0x80) ? 1 : 0,  /* bit 7 */
            (reg_value & 0x40) ? 1 : 0,  /* bit 6 */
            (reg_value & 0x20) ? 1 : 0,  /* bit 5 */
            (reg_value & 0x10) ? 1 : 0,  /* bit 4 */
            (reg_value & 0x08) ? 1 : 0,  /* bit 3 */
            (reg_value & 0x04) ? 1 : 0,  /* bit 2 */
            (reg_value & 0x01) ? 1 : 0); /* bit 0 */

        /* TEST CODE: Try to clear NTC charge protection bits (2,3) every cycle */
        static bool ntc_test_done = false;
        if (!ntc_test_done && (reg_value & 0x0C)) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                ">>> TEST: Attempting to clear NTC charge protection bits...");

            uint8_t ntc_original = reg_value;
            uint8_t ntc_clear = reg_value & ~0x0C;  /* Clear bits 2 and 3 */

            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                ">>> TEST: Original: 0x%02X, Writing: 0x%02X", ntc_original, ntc_clear);

            /* Write the cleared value */
            ret = ip5561_write_reg(g_ip5561_handle, 0xFD, ntc_clear, false);
            if (ret == ESP_OK) {
                /* Read back to verify */
                vTaskDelay(pdMS_TO_TICKS(10));  /* Small delay */
                ret = ip5561_read_reg(g_ip5561_handle, 0xFD, &reg_value, false);
                if (ret == ESP_OK) {
                    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                        ">>> TEST: Readback: 0x%02X [ChgHT:%d|ChgLT:%d|BoostHT:%d|BoostLT:%d] %s",
                        reg_value,
                        (reg_value & 0x04) ? 1 : 0,
                        (reg_value & 0x08) ? 1 : 0,
                        (reg_value & 0x10) ? 1 : 0,
                        (reg_value & 0x20) ? 1 : 0,
                        (reg_value == ntc_clear) ? "✓ SUCCESS" : "✗ FAILED");

                    /* If bits 2 or 3 are still set, try writing 0x00 to force all protection off */
                    if (reg_value & 0x0C) {
                        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                            ">>> TEST: Bits still set! Trying to force write 0x21...");
                        /* 0x21 = 0010 0001: Keep bit 0 (NTC enable) and bit 4 (Boost HT) */
                        uint8_t ntc_force = 0x21;
                        ip5561_write_reg(g_ip5561_handle, 0xFD, ntc_force, false);

                        vTaskDelay(pdMS_TO_TICKS(10));
                        ip5561_read_reg(g_ip5561_handle, 0xFD, &reg_value, false);
                        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                            ">>> TEST: After force: 0x%02X [ChgHT:%d|ChgLT:%d|BoostHT:%d|BoostLT:%d]",
                            reg_value,
                            (reg_value & 0x04) ? 1 : 0,
                            (reg_value & 0x08) ? 1 : 0,
                            (reg_value & 0x10) ? 1 : 0,
                            (reg_value & 0x20) ? 1 : 0);
                    }
                }
            }
            ntc_test_done = true;  /* Only run once */
        }
    }

    /* NTC Control Register - Current source configuration */
    ret = ip5561_read_reg(g_ip5561_handle, 0xF6, &reg_value, false);
    if (ret == ESP_OK) {
        uint8_t current_src = (reg_value >> 4) & 0x03;
        const char *current_str;
        switch (current_src) {
            case 0: current_str = "Auto"; break;
            case 1: current_str = "Auto"; break;
            case 2: current_str = "80uA"; break;
            case 3: current_str = "20uA"; break;
            default: current_str = "Unknown"; break;
        }
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "NTC_CTRL (0xF6): 0x%02X [Current:%s|raw:0x%X]",
            reg_value, current_str, current_src);
    }

    /* ========== WPC Status ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x14, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "VWPC_CTL0 (0x14): 0x%02X [WPC_EN:%d|WPC_V:%d|Chg:%d]",
            reg_value,
            (reg_value & 0x01) ? 1 : 0,
            (reg_value & 0x02) ? 1 : 0,
            (reg_value & 0x04) ? 1 : 0);
    }

    /* ========== VBUS/VOUT Control ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x18, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "VBUS_CTL0 (0x18): 0x%02X", reg_value);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x24, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "PPATH_CTL0 (0x24): 0x%02X [Power Path Control]", reg_value);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x10, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "VOUT_CTL0 (0x10): 0x%02X [OVP:%d|OCP:%d]",
            reg_value,
            (reg_value & 0x40) ? 1 : 0,
            (reg_value & 0x80) ? 1 : 0);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x13, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "VOUT_CTL1 (0x13): 0x%02X", reg_value);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x1C, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "VOUT_CTL2 (0x1C): 0x%02X", reg_value);
    }

    /* ========== Fast Charge Protocols ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x81, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "QC_CTRL0 (0x81): 0x%02X [QC_EN:%d]",
            reg_value,
            (reg_value & 0x01) ? 1 : 0);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x84, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "QC_CTRL1 (0x84): 0x%02X", reg_value);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x85, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "QC_CTRL2 (0x85): 0x%02X", reg_value);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0xD4, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "PD_CTRL (0xD4): 0x%02X", reg_value);
    }

    /* ========== Charging Control ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x01, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "CHG_CTL0 (0x01): 0x%02X", reg_value);
    }

    /* Read CHG_VOLT_SET (0x3A) - Charging voltage setting */
    ret = ip5561_read_reg(g_ip5561_handle, 0x3A, &reg_value, false);
    if (ret == ESP_OK) {
        /* Parse voltage setting bits [3:2] */
        uint8_t vset = (reg_value >> 2) & 0x03;
        const char *vset_str;
        uint16_t vset_mv;
        switch (vset) {
            case 0: vset_str = "4.2V"; vset_mv = 4200; break;
            case 1: vset_str = "4.3V"; vset_mv = 4300; break;
            case 2: vset_str = "4.35V"; vset_mv = 4350; break;
            case 3: vset_str = "4.4V"; vset_mv = 4400; break;
            default: vset_str = "Unknown"; vset_mv = 0; break;
        }

        /* Parse RCV (Recharge Voltage) bits [1:0] */
        uint8_t rcv = reg_value & 0x03;
        const char *rcv_str;
        uint16_t rcv_mv;
        switch (rcv) {
            case 0: rcv_str = "0mV"; rcv_mv = 0; break;
            case 1: rcv_str = "14mV"; rcv_mv = 14; break;
            case 2: rcv_str = "28mV"; rcv_mv = 28; break;
            case 3: rcv_str = "42mV"; rcv_mv = 42; break;
            default: rcv_str = "Unknown"; rcv_mv = 0; break;
        }

        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "CHG_VOLT_SET (0x3A): 0x%02X [VSET:%s(%dmV) | RCV:%s(%dmV) | CV_Total:%dmV]",
            reg_value, vset_str, vset_mv, rcv_str, rcv_mv, vset_mv + rcv_mv);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x0C, &reg_value, false);
    if (ret == ESP_OK) {
        /* Parse UV threshold from high 4 bits [7:4] */
        uint8_t uv_threshold = (reg_value >> 4) & 0x0F;
        uint16_t uv_mv = 6000 + (uv_threshold * 500);  // Convert to mV
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "CHG_CTL1 (0x0C): 0x%02X [9V UV Loop: %d mV | raw: 0x%X]",
            reg_value, uv_mv, uv_threshold);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0x0D, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "CHG_CTL2 (0x0D): 0x%02X [5V Input UV Loop]", reg_value);
    }

    /* ========== Charge Timeout ========== */
    ret = ip5561_read_reg(g_ip5561_handle, 0x21, &reg_value, false);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "CHG_TMO_CTL1 (0x21): 0x%02X [Tmo_EN:%d]",
            reg_value,
            (reg_value & 0x80) ? 1 : 0);
    }

    /* ========== ADC Readings ========== */
    // Note: IP5561 ADC values need scaling factors according to datasheet:
    // - VBAT_ADC: raw * 0.26855 mV
    // - VBUS_ADC/VOUT_ADC: raw * 1.611328 mV
    // - IBUS_ADC: raw * 0.671387 mA

    adc_value = ip5561_get_vbus_voltage(g_ip5561_handle);
    uint32_t vbus_mv = (uint32_t)(adc_value * 1.611328);  // VBUS scaling
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "VBUS_ADC: %u mV (raw: %u)", vbus_mv, adc_value);

    int16_t ibus_current = ip5561_get_ibus_current(g_ip5561_handle);
    int16_t ibus_ma = (int16_t)(ibus_current * 0.671387);  // IBUS scaling
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "IBUS_ADC: %d mA (raw: %d)", ibus_ma, ibus_current);

    adc_value = ip5561_get_vout_voltage(g_ip5561_handle);
    uint32_t vout_mv = (uint32_t)(adc_value * 1.611328);  // VOUT scaling
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "VOUT_ADC: %u mV (raw: %u)", vout_mv, adc_value);

    adc_value = ip5561_get_battery_voltage(g_ip5561_handle);
    uint32_t vbat_mv = (uint32_t)(adc_value * 0.26855);  // VBAT scaling
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "VBAT_ADC: %u mV (raw: %u)", vbat_mv, adc_value);

    /* Read IBAT (battery charging current) - KEY INDICATOR! */
    if (g_ip5561_handle != NULL) {
        int16_t ibat_current = ip5561_get_battery_current(g_ip5561_handle);
        /*
         * IBAT ADC scaling: IP5561 provides signed 16-bit ADC value
         * LSB = 1.0 mA per IP5561 datasheet
         * Positive value = charging, Negative value = discharging
         * Conversion already applied in ip5561_get_battery_current()
         */
        int16_t ibat_ma = ibat_current;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "IBAT_ADC: %d mA (raw: %d)",
            ibat_ma, ibat_current);
    }

    /* Read NTC thermistor temperature */
    int16_t ntc_temp_c = 0;
    uint16_t ntc_adc = 0;
    ret = ip5561_get_ntc_temperature(g_ip5561_handle, &ntc_temp_c, &ntc_adc);
    if (ret == ESP_OK) {
        uint32_t ntc_mv = (uint32_t)(ntc_adc * 0.26855);  // NTC scaling
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "NTC1: %d°C (ADC: %u, %u mV)", ntc_temp_c, ntc_adc, ntc_mv);
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to read NTC temperature");
    }

    /* ========== Fast Charge Protocol Status Registers ========== */
    uint8_t fcp_status = 0, status_src0 = 0, status_src1 = 0;

    ret = ip5561_read_reg(g_ip5561_handle, 0xA1, &fcp_status, true);
    if (ret == ESP_OK) {
        /* FCP_STATUS: bit 7:6 = FCP voltage selection */
        uint8_t fcp_vsel = (fcp_status >> 6) & 0x03;
        const char *vsel_str;
        switch (fcp_vsel) {
            case 0: vsel_str = "5V"; break;
            case 1: vsel_str = "9V"; break;
            case 2: vsel_str = "12V"; break;
            default: vsel_str = "Unknown"; break;
        }
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "FCP_STATUS (0xA1): 0x%02X [FCP_VSEL:%s]", fcp_status, vsel_str);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0xA4, &status_src0, true);
    if (ret == ESP_OK) {
        /* STATUS_SRC0: bit 3:0 = VOUT fast charge protocol */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "STATUS_SRC0 (0xA4): 0x%02X [VOUT_FCP_State:0x%X]",
            status_src0, status_src0 & 0x0F);
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0xA5, &status_src1, true);
    if (ret == ESP_OK) {
        /* STATUS_SRC1: VBUS/VOUT protocol status */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "STATUS_SRC1 (0xA5): 0x%02X", status_src1);
    }

    /* ========== NTC1 and OCP State Registers ========== */
    uint8_t ntc1_state = 0, ocp_state = 0;

    ret = ip5561_read_reg(g_ip5561_handle, 0xFB, &ntc1_state, true);
    if (ret == ESP_OK) {
        /* NTC1_STATE: NTC temperature flags and output MOS current status
         * Bit 7: Ntc1_ht    - NTC1 high temperature flag
         * Bit 6: Ntc1_mht   - NTC1 mid-high temperature flag
         * Bit 5: Ntc1_mlt   - NTC1 mid-low temperature flag
         * Bit 4: Ntc1_lt    - NTC1 low temperature flag
         * Bit 3: Mos_vbus_ilow  - VBUS output light load
         * Bit 2: Mos_vwpc_ilow  - VWPC output light load
         * Bit 1: Mos_vout_ilow  - VOUT output light load
         * Bit 0: Reserved
         */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "NTC1_STATE (0xFB): 0x%02X [Hot:%d|MHot:%d|MLow:%d|Cold:%d]",
            ntc1_state,
            (ntc1_state & 0x80) ? 1 : 0,  /* bit 7: Ntc1_ht */
            (ntc1_state & 0x40) ? 1 : 0,  /* bit 6: Ntc1_mht */
            (ntc1_state & 0x20) ? 1 : 0,  /* bit 5: Ntc1_mlt */
            (ntc1_state & 0x10) ? 1 : 0);  /* bit 4: Ntc1_lt */
    }

    ret = ip5561_read_reg(g_ip5561_handle, 0xFC, &ocp_state, true);
    if (ret == ESP_OK) {
        /* OCP_STATE: System overcurrent status
         * Bit 7:3: Reserved
         * Bit 2: Boost_uv  - Boost overcurrent flag
         * Bit 1: Reserved
         * Bit 0: Boost_scdt - Boost short circuit flag
         */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "OCP_STATE (0xFC): 0x%02X [Boost_OCP:%d|Boost_Short:%d]",
            ocp_state,
            (ocp_state & 0x04) ? 1 : 0,   /* bit 2: Boost overcurrent */
            (ocp_state & 0x01) ? 1 : 0);   /* bit 0: Boost short circuit */
    }

    /* ========== Charging Status Summary ========== */
    ip5561_chg_status_t chg_status;
    if (ip5561_get_charge_status(g_ip5561_handle, &chg_status) == ESP_OK) {
        /* Parse detailed charge state from chg_status register */
        uint8_t chg_state = (chg_status.chg_status >> 4) & 0x07;
        const char *chg_state_detail;
        switch (chg_state) {
            case 0x0: chg_state_detail = "IDLE"; break;
            case 0x1: chg_state_detail = "PRE-CHARGE"; break;
            case 0x2: chg_state_detail = "CC (恒流)"; break;
            case 0x3: chg_state_detail = "CV (恒压)"; break;
            case 0x5: chg_state_detail = "FULL (充满)"; break;
            default:  chg_state_detail = "UNKNOWN"; break;
        }

        /* Read SYS_CTL0 to get ChgEN and BoostEN status */
        uint8_t sys_ctl0 = 0;
        ip5561_get_sys_ctl0(g_ip5561_handle, &sys_ctl0);

        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "========== CHARGING STATUS ==========");
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "  Charging: %s | Done: %s | VBUS: %s",
            chg_status.charging ? "YES" : "NO",
            chg_status.chg_done ? "YES" : "NO",
            chg_status.vbus_valid ? "Valid" : "Invalid");
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "  Phase: %s (raw: 0x%X)", chg_state_detail, chg_state);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "  ChgEN: %s | BoostEN: %s",
            (sys_ctl0 & 0x01) ? "ON" : "OFF",
            (sys_ctl0 & 0x02) ? "ON" : "OFF");
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "============================================================");

    return ESP_OK;
}

/* ========== Battery SOC Calibration API ========== */

/**
 * @brief Get battery state of charge with IR compensation
 *
 * This function calculates SOC based on compensated voltage to improve accuracy
 * during charging. When charging, the terminal voltage is higher than the actual
 * battery voltage due to internal resistance (IR drop). This function compensates
 * for this effect to provide a more accurate SOC estimate.
 *
 * Calculation:
 * - If charging: V_comp = V_term - (|I_chg| × R_internal)
 * - If discharging: V_comp = V_term (no compensation)
 * - SOC% = (V_comp - 3.0V) / (4.35V - 3.0V) × 100
 *
 * @param soc_percent Output: calibrated SOC percentage (0-100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_manage_get_calibrated_soc(uint16_t *soc_percent)
{
    if (soc_percent == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    fuel_gauge_handle_t battery_handle = g_fuel_gauge_handle;
    if (battery_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Get battery voltage and current (interrupt-safe)
    uint16_t voltage_mv = safe_get_battery_voltage();
    int16_t current_ma = safe_get_battery_current();

    if (voltage_mv == 0 && !xPortInIsrContext()) {
        // I2C read failed and not in interrupt context
        return ESP_ERR_INVALID_STATE;
    }

    // Use apply_ir_compensation to get V_comp
    bool is_charging = false;
    uint16_t v_comp_mv = apply_ir_compensation(voltage_mv, current_ma, &is_charging);

    if (is_charging) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "Battery SOC calibration: V_term=%umV, I_chg=%dmA → V_comp=%umV",
            voltage_mv, current_ma, v_comp_mv);
    }

    // Calculate SOC based on compensated voltage
    // Li-ion battery: 3.0V (empty) to 4.35V (full)
    const uint16_t V_EMPTY_MV = 3000;
    const uint16_t V_FULL_MV = 4350;

    uint16_t soc;
    if (v_comp_mv <= V_EMPTY_MV) {
        soc = 0;
    } else if (v_comp_mv >= V_FULL_MV) {
        soc = 100;
    } else {
        // Linear interpolation: SOC% = (V_comp - 3.0V) / (4.35V - 3.0V) × 100
        soc = (uint16_t)((v_comp_mv - V_EMPTY_MV) * 100 / (V_FULL_MV - V_EMPTY_MV));
    }

    *soc_percent = soc;

    SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "Calibrated SOC: %d%% (based on V_comp=%umV%s)",
        soc, v_comp_mv, (v_comp_mv != voltage_mv) ? ", IR compensated" : "");

    return ESP_OK;
}

/**
 * @brief Get battery voltage with IR compensation (V_comp)
 *
 * This function returns the compensated battery voltage, which accounts for
 * internal resistance (IR) voltage drop during charging. When charging,
 * the terminal voltage is higher than the actual battery voltage due to IR drop.
 *
 * Calculation:
 * - If charging: V_comp = V_term - (|I_chg| × R_internal)
 * - If discharging: V_comp = V_term (no compensation)
 *
 * This provides a more accurate representation of the actual battery state
 * for reporting to clients.
 *
 * @param[out] voltage_mv Pointer to store compensated voltage in mV
 * @param[out] is_charging Pointer to store charging state (true=charging, false=discharging)
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if voltage_mv or is_charging is NULL
 *     - ESP_ERR_INVALID_STATE if BQ27220 not initialized
 */
esp_err_t power_manage_get_battery_voltage_compensated(uint16_t *voltage_mv, bool *is_charging)
{
    if (voltage_mv == NULL || is_charging == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    fuel_gauge_handle_t battery_handle = g_fuel_gauge_handle;
    if (battery_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Get battery voltage and current (interrupt-safe) */
    uint16_t voltage = safe_get_battery_voltage();
    if (voltage == 0 && !xPortInIsrContext()) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to read battery voltage");
        return ESP_FAIL;
    }

    int16_t current = safe_get_battery_current();

    /* Determine charging state based on current with hysteresis */
    bool charging = false;

    if (!xPortInIsrContext() && battery_handle != NULL) {
        /* Not in interrupt context - we can read battery status */
        battery_status_t bat_status;
        if (fuel_gauge_get_battery_status(battery_handle, &bat_status) == ESP_OK) {
            if (current < -200) {
                /* Clearly charging: current < -200mA */
                charging = true;
            } else if (current > 200) {
                /* Clearly discharging: current > 200mA */
                charging = false;
            } else {
                /* Small current zone (-200mA ~ +200mA): use DSG flag */
                charging = !bat_status.DSG;
            }
        }
    } else {
        /* In interrupt context - use cache or current-based detection */
        charging = (current < -200) ? true : (current > 200) ? false : false;
    }

    /* Use apply_ir_compensation for consistency */
    bool charging_state;
    uint16_t v_comp = apply_ir_compensation(voltage, current, &charging_state);

    if (charging_state) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "V_comp: V_term=%umV, I_chg=%dmA → V_comp=%umV",
            voltage, current, v_comp);
    } else {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "V_comp: V_term=%umV (no IR compensation)", voltage);
    }

    *voltage_mv = v_comp;
    *is_charging = charging_state;

    return ESP_OK;
}
