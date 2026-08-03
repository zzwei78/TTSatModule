/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_bt.h"
#include "esp_rom_gpio.h"
#include "IP5561.h"
#include "system/power_manage.h"
#include "syslog.h"
#include "tt/tt_module.h"
#include "tt/tt_hardware.h"
#include "ble/gatt_system_server.h"
#include "ble/spp_at_server.h"
#include "config/user_params.h"
#include "config/hardware_version.h"
#include "system/led_manager.h"

#define SENSOR_IDLE_SLEEP_ENABLE 0

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

/* Return fuel gauge chip name for logging */
static const char *fuel_gauge_name(void)
{
    if (g_fuel_gauge_handle == NULL) return "N/A";
    return (fuel_gauge_get_type(g_fuel_gauge_handle) == FUEL_GAUGE_BQ27220)
           ? "BQ27220" : "CW2217E";
}

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
static TaskHandle_t g_ip5561_manage_task = NULL;
static TaskHandle_t g_fuel_gauge_task_handle = NULL;
static TaskHandle_t g_sensor_task_handle = NULL;

/* Sensor report session timer (reset on each ENABLE, used for 30s auto-disable) */
static volatile uint32_t g_sensor_session_ms = 0;

/* Monitor task running flags */
static bool g_ip5561_manage_running = false;
static bool g_fuel_gauge_task_running = false;

/* Cached average current (calculated from CoulombCounter delta, updated every 60s) */
static int16_t g_avg_current_ma = 0;

/* ========== Battery Event Report State ========== */
#define BATT_TEMP_HIGH_THRESHOLD    55      /* °C: high temperature alert */
#define BATT_TEMP_HIGH_RECOVERY     52      /* °C: hysteresis recovery */
#define BATT_TEMP_LOW_THRESHOLD     (-10)   /* °C: low temperature alert */
#define BATT_TEMP_LOW_RECOVERY      (-7)    /* °C: hysteresis recovery */
#define BATT_CHARGE_CURRENT_MA      200     /* mA: threshold to distinguish charging */

static bool g_batt_report_enabled = false;
static uint8_t g_batt_report_flags = 0;
static uint8_t g_batt_soc_threshold = BATTERY_REPORT_DEFAULT_SOC_THRESHOLD;

/* State tracking for event detection */
static bool g_batt_event_inited = false;    /* First-read initialization */
static uint8_t g_batt_last_soc = 0;
static bool g_batt_last_charging = false;
static bool g_batt_temp_high_active = false;
static bool g_batt_temp_low_active = false;
static uint16_t g_batt_periodic_cnt = 0;   /* Periodic heartbeat counter (×10s) */
#define BATT_PERIODIC_INTERVAL    12       /* 12 × 10s = 120s = 2 minutes */

/* IP5561 configuration fingerprint for reset detection */
typedef struct {
    uint8_t sys_ctl0;
    uint8_t sys_ctl4;
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

/* V2.0: Low-battery boost thresholds (hysteresis) */
#define LOW_BAT_BOOST_OFF_MV    3300            /* Falling: TT off, MCU requests boost */
#define LOW_BAT_BOOST_ON_MV     3400            /* Rising: TT can restart, MCU releases boost */
#endif

/* IP5561 wakeup GPIO definition */
#define IP5561_WAKEUP_GPIO      GPIO_NUM_8      /* GPIO8 for IP5561 wakeup (KEY) */
#define IP5561_INT_GPIO         GPIO_NUM_5      /* GPIO5: IP5561 INT (I2C mode ready) */

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
 * @brief IP5561 long wakeup: hold KEY high for 2 seconds.
 *
 * Required to wake from deep shutdown mode (e.g. after light-load
 * shutdown or prolonged power-off). Call once during init before
 * any I2C communication.
 */
void power_manage_ip5561_long_wakeup(void)
{
    gpio_set_level(IP5561_WAKEUP_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(IP5561_WAKEUP_GPIO, 1);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 long wakeup (2s key press)...");
    vTaskDelay(pdMS_TO_TICKS(2500));

    gpio_set_level(IP5561_WAKEUP_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* ========== IP5561 INT GPIO (GPIO5, common for V1/V2) ========== */

/**
 * @brief Configure INT (GPIO5) as input
 *
 * INT is driven HIGH by IP5561 when I2C mode is ready.
 */
static void ip5561_int_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IP5561_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Disable RTC pull-down left over from deep sleep (if woken from deep sleep).
     * During deep sleep, sleep_manager enables RTC pull-down on GPIO5 for clean
     * LOW level; clear it here so IP5561's INT signal is read unmodified. */
    rtc_gpio_pulldown_dis(IP5561_INT_GPIO);
}

/**
 * @brief Wait for INT pin to stay HIGH continuously for >=100ms
 *
 * Per datasheet: after IP5561 raises INT, MCU must wait at least 100ms
 * of continuous HIGH before starting I2C communication.
 *
 * @param timeout_ms Maximum time to wait
 * @return true if INT confirmed HIGH for 100ms, false on timeout
 */
static bool ip5561_wait_int_ready(uint32_t timeout_ms)
{
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t high_since_tick = 0;
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    TickType_t hold_ticks = pdMS_TO_TICKS(100);  /* 100ms continuous HIGH required */

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        if (gpio_get_level(IP5561_INT_GPIO) == 1) {
            if (high_since_tick == 0) {
                high_since_tick = xTaskGetTickCount();
            } else if ((xTaskGetTickCount() - high_since_tick) >= hold_ticks) {
                return true;  /* INT sustained HIGH for >=100ms */
            }
        } else {
            high_since_tick = 0;  /* INT dropped — restart timer */
        }
        vTaskDelay(pdMS_TO_TICKS(10));  /* ≥1 tick: yields to IDLE (avoids watchdog) */
    }
    return false;  /* Timeout */
}

/* ========== IP5561 I2C Handshake (V2 only, dedicated I2C1 bus) ========== */
#ifdef SUPPORT_HARDWARE_V2

/**
 * @brief Set I2C1 SDA/SCL to input + internal pullup (high-Z but pulled to 3.3V)
 *
 * IP5561 multiplexes LED1=SDA and LED2=SCL. When IP5561 wakes from shutdown,
 * it checks whether LED1/LED2 are pulled HIGH. If so, it enters I2C mode and
 * raises INT. This must be called BEFORE the wakeup key press so the pins are
 * ready when IP5561 checks them.
 */
static void ip5561_i2c_pins_high_z(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IP5561_I2C_SCL_IO) | (1ULL << IP5561_I2C_SDA_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

#endif /* SUPPORT_HARDWARE_V2 */

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

void power_manage_ip5561_deep_sleep_prepare(void)
{
    /* Reset I2C1 pins (GPIO6/GPIO14) to high-Z so they don't drive during deep sleep.
     * On wakeup, app_main() re-runs and sets them high-Z before IP5561 handshake. */
    gpio_set_direction(IP5561_I2C_SCL_IO, GPIO_MODE_DISABLE);
    gpio_set_direction(IP5561_I2C_SDA_IO, GPIO_MODE_DISABLE);

    /* Hold KEY (GPIO8) LOW during deep sleep */
    gpio_set_level(IP5561_WAKEUP_GPIO, 0);
    gpio_hold_en(IP5561_WAKEUP_GPIO);

    /* INT (GPIO5) is input — no action needed, left in default high-Z */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[IP5561] Deep sleep: I2C pins high-Z, KEY held LOW");
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

bool power_manage_is_charging(void)
{
    /* IP5561 is the charge IC — its CHG_STATE2 bit3 is the authoritative
     * "charging in progress" flag. Use it when IP5561 is available. */
    if (g_ip5561_handle != NULL) {
        return ip5561_is_charging(g_ip5561_handle);
    }
    /* Fallback: fuel gauge current direction (unified API: negative = charging) */
    if (g_fuel_gauge_handle != NULL) {
        return fuel_gauge_get_current(g_fuel_gauge_handle) < -BATT_CHARGE_CURRENT_MA;
    }
    return false;
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
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[%s]: Failed to read voltage", fuel_gauge_name());
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
        "[%s]: V=%umV, I=%dmA, DSG=%d → Charging=%d", fuel_gauge_name(),
        voltage, current, batt_status.DSG, fg_charging);

    /* ========== Step 2: Read IP5561 charging status (secondary source) ========== */
    if (g_ip5561_handle != NULL) {
        ip5561_charging = ip5561_is_charging(g_ip5561_handle);
        ip5561_available = true;  /* IP5561 communication successful */

        SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "IP5561: Charging=%d", ip5561_charging);
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561: Handle NULL, using %s only", fuel_gauge_name());
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
                "State MISMATCH! %s_charging=%d (DSG=%d), I=%dmA, IP5561=%d → Using CURRENT (Charging=%d)",
                fuel_gauge_name(),
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

/* ========== Battery Event Detection ========== */

/* Get real battery temperature from IP5561 NTC thermistor.
 * Returns 0.1°K (same format as fuel_gauge_get_temperature).
 * The CW2217E fuel gauge has no real temp sensor (its reg 0x06 is a host-written
 * compensation value that is never written → bogus), so the actual battery
 * temperature must come from the IP5561 NTC1 pin (real thermistor).
 * Falls back to fuel gauge temp only if IP5561 NTC read fails. */
uint16_t power_manage_get_battery_temp_0_1k(void)
{
    if (g_ip5561_handle != NULL) {
        int16_t temp_c = 0;
        if (ip5561_get_ntc_temperature(g_ip5561_handle, &temp_c, NULL) == ESP_OK) {
            return (uint16_t)(temp_c * 10 + 2732);  /* °C → 0.1°K */
        }
        /* NTC read failed (e.g. not yet settled at boot) — fall through to gauge */
    }
    if (g_fuel_gauge_handle != NULL) {
        return fuel_gauge_get_temperature(g_fuel_gauge_handle);  /* degraded fallback */
    }
    return 2732;  /* 0°C default */
}

static void battery_event_check(void)
{
    if (!g_batt_report_enabled || g_fuel_gauge_handle == NULL) {
        return;
    }

    uint16_t voltage = fuel_gauge_get_voltage(g_fuel_gauge_handle);
    int16_t current = fuel_gauge_get_current(g_fuel_gauge_handle);
    uint16_t soc = fuel_gauge_get_state_of_charge(g_fuel_gauge_handle);
    uint16_t temp_0_1k = power_manage_get_battery_temp_0_1k();

    /* Convert temperature: 0.1°K → °C */
    int temp_c = (int)temp_0_1k / 10 - 273;

    /* Charging detection: IP5561 first (authoritative), fallback to fuel gauge current.
     * IP5561 knows the actual charge state; fuel gauge current is an approximation. */
    bool charging_now = power_manage_is_charging();

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "[BATT_EVENT] V=%umV I=%dmA SOC=%u%% T=%d°C charging=%d (flags=0x%02X)",
        voltage, current, soc, temp_c, charging_now, g_batt_report_flags);

    /* Update battery LED (every check cycle) */
    led_set_battery_status(soc, charging_now, soc >= 100);

    /* Initialize baseline on first call */
    if (!g_batt_event_inited) {
        g_batt_last_soc = soc;
        g_batt_last_charging = charging_now;
        g_batt_event_inited = true;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "[BATT_EVENT] Initialized: soc=%u%%, charging=%d, temp=%d°C",
            soc, g_batt_last_charging, temp_c);
        return;
    }

    /* --- SOC change detection --- */
    if ((g_batt_report_flags & BATTERY_REPORT_FLAG_SOC) &&
        (uint8_t)abs((int)soc - (int)g_batt_last_soc) >= g_batt_soc_threshold) {
        gatt_system_server_send_battery_event(BATTERY_EVENT_SOC_CHANGE,
                                              soc, voltage, temp_0_1k, charging_now);
        g_batt_last_soc = soc;
    }

    /* --- Charging status detection --- */
    if (g_batt_report_flags & BATTERY_REPORT_FLAG_CHARGE) {
        if (charging_now != g_batt_last_charging) {
            if (charging_now) {
                /* Started charging */
                gatt_system_server_send_battery_event(BATTERY_EVENT_CHARGE_START,
                                                      soc, voltage, temp_0_1k, 1);
            } else {
                /* Stopped charging — check if full */
                if (g_batt_last_charging && soc >= 99) {
                    gatt_system_server_send_battery_event(BATTERY_EVENT_CHARGE_FULL,
                                                          soc, voltage, temp_0_1k, 0);
                } else {
                    gatt_system_server_send_battery_event(BATTERY_EVENT_CHARGE_STOP,
                                                          soc, voltage, temp_0_1k, 0);
                }
            }
            g_batt_last_charging = charging_now;
        }
    }

    /* --- Temperature high alert (with hysteresis) --- */
    if (g_batt_report_flags & BATTERY_REPORT_FLAG_TEMP) {
        if (temp_c >= BATT_TEMP_HIGH_THRESHOLD && !g_batt_temp_high_active) {
            gatt_system_server_send_battery_event(BATTERY_EVENT_TEMP_HIGH,
                                                  soc, voltage, temp_0_1k, charging_now);
            g_batt_temp_high_active = true;
        } else if (temp_c <= BATT_TEMP_HIGH_RECOVERY) {
            g_batt_temp_high_active = false;
        }

        if (temp_c <= BATT_TEMP_LOW_THRESHOLD && !g_batt_temp_low_active) {
            gatt_system_server_send_battery_event(BATTERY_EVENT_TEMP_LOW,
                                                  soc, voltage, temp_0_1k, charging_now);
            g_batt_temp_low_active = true;
        } else if (temp_c >= BATT_TEMP_LOW_RECOVERY) {
            g_batt_temp_low_active = false;
        }
    }

    /* --- Periodic heartbeat (every 2 min) --- */
    /* Ensures APP receives current state even if change events were missed */
    if (++g_batt_periodic_cnt >= BATT_PERIODIC_INTERVAL) {
        g_batt_periodic_cnt = 0;
        gatt_system_server_send_battery_event(BATTERY_EVENT_PERIODIC,
                                              soc, voltage, temp_0_1k, charging_now);
    }
}

void power_manage_set_battery_report(uint8_t flags, uint8_t soc_threshold)
{
    g_batt_report_flags = flags;
    g_batt_soc_threshold = (soc_threshold > 0) ? soc_threshold : 1;
    g_batt_event_inited = false;  /* Re-init baseline on enable */
    g_batt_periodic_cnt = 0;     /* Reset periodic counter */
    g_batt_report_enabled = (flags != 0);

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "[BATT_EVENT] Report %s (flags=0x%02X, soc_thr=%u%%)",
        g_batt_report_enabled ? "ENABLED" : "DISABLED",
        g_batt_report_flags, g_batt_soc_threshold);
}

void power_manage_disable_battery_report(void)
{
    g_batt_report_enabled = false;
    g_batt_report_flags = 0;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[BATT_EVENT] Report DISABLED");
}

static void __attribute__((unused)) fuel_gauge_monitor_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[%s] monitor task started (60s period)", fuel_gauge_name());

    int error_count = 0;
    uint16_t prev_coulomb_count = 0;
    bool first_read = true;

    while (g_fuel_gauge_task_running) {
        /* ========== Read battery data ========== */
        uint16_t voltage = fuel_gauge_get_voltage(g_fuel_gauge_handle);
        if (voltage == 0) {
            error_count++;
            if (error_count > 3) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[%s] read failed %d times", fuel_gauge_name(), error_count);
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

        /* ========== Update LED indicators (always, regardless of report setting) ========== */
        {
            bool led_charging = power_manage_is_charging();
            bool led_full = (soc >= 100);
            led_set_battery_status(soc, led_charging, led_full);
        }

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

                if ((tt_state == TT_STATE_WORKING || tt_state == TT_STATE_UPGRADE_MODE)
                    && voltage < POWER_MANAGE_TT_MODULE_V_OFF_MV) {
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
            SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[BATT] BLE TX power: %d dBm", ble_pwr);
        }

        /* Wait 60 seconds before next check (1s granularity for quick stop response).
         * Battery event detection runs every 10 seconds within this window. */
        for (int i = 0; i < 60 && g_fuel_gauge_task_running; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (g_batt_report_enabled && (i % 10 == 9)) {
                battery_event_check();
            }
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
    /* NTC temperature protection */
#if !IP5561_ENABLE_NTC_PROT
    ip5561_disable_ntc(g_ip5561_handle);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "NTC protection: disabled");
#else
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "NTC protection: enabled");
#endif

    /* VOUT: disabled (unused port, save power) */
    ip5561_disable_vout(g_ip5561_handle);

    /* Light load auto-shutdown */
#if !IP5561_ENABLE_LIGHT_LOAD_PROT
    ip5561_configure_light_load(g_ip5561_handle, false, false, 0);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Light-load protection: disabled");
#else
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Light-load protection: enabled");
#endif

    /* Charge safety timeout (0x21 bit7) */
    {
        uint8_t chg_tmo = 0;
        ip5561_read_reg(g_ip5561_handle, 0x21, &chg_tmo, false);
#if !IP5561_ENABLE_CHARGE_TIMEOUT
        chg_tmo &= ~(1 << 7);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Charge timeout: disabled");
#else
        chg_tmo |= (1 << 7);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Charge timeout: enabled");
#endif
        ip5561_write_reg(g_ip5561_handle, 0x21, chg_tmo, false);
    }

    /* SYS_CTL4 (0x31): Enable long-press 2S key wakeup + keep default key-off mode */
    {
        uint8_t sys_ctl4 = 0;
        ip5561_read_reg(g_ip5561_handle, 0x31, &sys_ctl4, false);
        sys_ctl4 |= (1 << 2);               /* En_Long_Wk = 1 (long press 2s wakeup) */
        ip5561_write_reg(g_ip5561_handle, 0x31, sys_ctl4, false);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "SYS_CTL4: 0x%02X (En_Long_Wk=1)", sys_ctl4);
    }

    /* SYS_CTL5 (0x33): Enable low-current always-on mode.
     * En_Lowcur=1 keeps chip alive in low-power mode even after VBUS removal,
     * preventing unwanted shutdown. */
    {
        uint8_t sys_ctl5 = 0;
        ip5561_read_reg(g_ip5561_handle, 0x33, &sys_ctl5, false);
        sys_ctl5 |= (1 << 5);               /* En_Lowcur = 1 (enable always-on mode) */
        ip5561_write_reg(g_ip5561_handle, 0x33, sys_ctl5, false);
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "SYS_CTL5: 0x%02X (En_Lowcur=1)", sys_ctl5);
    }

    /* Charge voltage and current */
    ip5561_set_charge_voltage(g_ip5561_handle, 4350);
    ip5561_set_5v_charge_current(g_ip5561_handle, 3500);
    ip5561_set_9v_charge_current(g_ip5561_handle, 2500);
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

    /* Boost OCP/OVP protection */
#if !IP5561_ENABLE_BOOST_PROT
    ip5561_disable_boost_protections(g_ip5561_handle, false);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Boost protection: disabled");
#else
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Boost protection: enabled");
#endif

    /* Clear OCP flags before enabling WPC (per factory test reference) */
    ip5561_write_reg(g_ip5561_handle, 0xFC, 0x05, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* WPC: enable LAST (after Boost is stable) — per factory test sequence */
    ip5561_set_wpc_enable(g_ip5561_handle, true);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "WPC enabled (after Boost stable)");

    /* Save configuration fingerprint (only stable registers) */
    ip5561_get_sys_ctl0(g_ip5561_handle, &g_ip5561_fingerprint.sys_ctl0);
    ip5561_read_reg(g_ip5561_handle, 0x31, &g_ip5561_fingerprint.sys_ctl4, false);
    g_ip5561_fingerprint.initialized = true;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "IP5561 fingerprint saved: CTL0=0x%02X CTL4=0x%02X",
        g_ip5561_fingerprint.sys_ctl0, g_ip5561_fingerprint.sys_ctl4);

    return ESP_OK;
}

/**
 * @brief Initialize IP5561 device independently
 *
 * @return ESP_OK on success, ESP_FAIL on failure (non-fatal, allows other devices to init)
 */
static esp_err_t power_manage_init_ip5561(void)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "=== Initializing IP5561 (Charging Manager) ===");

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

    /* Check if IP5561 had internal reset by comparing stable registers.
     * Uses CTL0 + CTL4 (not QC_CTRL0 — it's a trigger that auto-clears). */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
        "IP5561: fingerprint %s (CTL0=0x%02X CTL4=0x%02X)",
        g_ip5561_fingerprint.initialized ? "EXISTS" : "NONE",
        g_ip5561_fingerprint.sys_ctl0, g_ip5561_fingerprint.sys_ctl4);

    if (g_ip5561_fingerprint.initialized) {
        uint8_t cur_ctl0 = 0;
        uint8_t cur_ctl4 = 0;
        ip5561_get_sys_ctl0(g_ip5561_handle, &cur_ctl0);
        ip5561_read_reg(g_ip5561_handle, 0x31, &cur_ctl4, false);

        if (cur_ctl0 == g_ip5561_fingerprint.sys_ctl0 &&
            cur_ctl4 == g_ip5561_fingerprint.sys_ctl4) {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "IP5561: No reset detected, config intact (CTL0=0x%02X CTL4=0x%02X)",
                cur_ctl0, cur_ctl4);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561: ✓ Initialization complete");
            return ESP_OK;
        }

        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
            "IP5561: Reset detected! CTL0: 0x%02X->0x%02X, CTL4: 0x%02X->0x%02X, re-applying config",
            g_ip5561_fingerprint.sys_ctl0, cur_ctl0,
            g_ip5561_fingerprint.sys_ctl4, cur_ctl4);
    }

    /* Apply full register configuration (first init or after reset) */
    power_manage_apply_ip5561_config();

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561: ✓ Initialization complete");
    return ESP_OK;
}

/* ========== IP5561 Public Control API ========== */

void power_manage_ip5561_double_click_shutdown(void)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[IP5561] Double-click shutdown (KEY pulse x2)");

    /* Simulate physical double-click: two short KEY pulses */
    gpio_set_level(IP5561_WAKEUP_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(IP5561_WAKEUP_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    gpio_set_level(IP5561_WAKEUP_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(IP5561_WAKEUP_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
}

esp_err_t power_manage_ip5561_set_vbus_output(bool enable)
{
    if (g_ip5561_handle == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ip5561_configure_vbus_output(g_ip5561_handle, enable);
    if (ret == ESP_OK) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "VBUS output %s", enable ? "enabled" : "disabled");
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to %s VBUS output: %s",
                        enable ? "enable" : "disable", esp_err_to_name(ret));
    }
    return ret;
}

/* ========== IP5561 Manage Task ========== */

/* Forward declarations: tasks defined later in this file */
static void sensor_monitor_task(void *pvParameters);

/**
 * @brief IP5561 unified manage task
 *
 * Replaces ip5561_monitor_task + ip5561_retry_task.
 *
 * Period:
 *   - If not initialized: 2 minutes (retry init)
 *   - If initialized:     1 minute  (health check)
 *
 * Each cycle:
 *   1. If handle == NULL → try init, wait 2 min
 *   2. If handle != NULL → probe SYS_CTL0:
 *      - Comm OK → check OCP, NTC, config fingerprint (reset detection)
 *      - Comm fail → long wakeup → re-probe → if still fail, handle = NULL
 */
static void ip5561_manage_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[IP5561 Manage] Task started");

    /* Initial delay to let system stabilize after boot */
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    while (g_ip5561_manage_running) {
        uint32_t next_delay_ms;

        if (g_ip5561_handle == NULL) {
            /* === Case 1: Not initialized — short KEY then attempt init === */
            int int_level = gpio_get_level(IP5561_INT_GPIO);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                            "[IP5561 Manage] Not initialized (INT=%d), trying init...",
                            int_level);
        
            /* Short KEY pulse only during init retry, NOT during health check */
            gpio_set_level(IP5561_WAKEUP_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(IP5561_WAKEUP_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(50));

            esp_err_t ret = power_manage_init_ip5561();
            if (ret == ESP_OK) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                                "[IP5561 Manage] Init success!");
            } else {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                                "[IP5561 Manage] Init failed (INT=%d), will retry in 2 min",
                                int_level);
            }
            next_delay_ms = 120000;  /* 2 min between init attempts */
        } else {
            /* === Case 2: Initialized — health check (no KEY pulse!) === */
            uint8_t sys_ctl0 = 0;
            esp_err_t ret = ip5561_get_sys_ctl0(g_ip5561_handle, &sys_ctl0);

            if (ret == ESP_OK) {
                /* --- Charging status (0xE9 @ 0xEA) --- */
                uint8_t chg_state2 = 0;
                ip5561_read_reg(g_ip5561_handle, 0xE9, &chg_state2, true);
                /* 0xE9: bit0=VBUS_IN_OK, bit1=CHG_DONE, bit3=CHG_ACTIV,
                 *       bits6:4 = charge state (0=Idle,1=PreChg,2=CC,3=CV,5=Full) */
                bool vbus_in   = (chg_state2 & 0x01) != 0;
                bool chg_done  = (chg_state2 & 0x02) != 0;
                bool chg_activ = (chg_state2 & 0x08) != 0;
                uint8_t chg_phase = (chg_state2 >> 4) & 0x07;

                int16_t ibat = ip5561_get_battery_current(g_ip5561_handle);
                uint16_t vbat_adc = ip5561_get_battery_voltage(g_ip5561_handle);
                uint16_t vbat_mv = (uint16_t)(vbat_adc * 0.26855f);

                /* --- NTC temperature state (0xFB @ 0xEA) ---
                 * bits7:4 = Hot / MHot / MLow / Cold */
                uint8_t ntc_state = 0;
                ip5561_read_reg(g_ip5561_handle, 0xFB, &ntc_state, true);
                bool ntc_hot  = (ntc_state & 0x80) != 0;
                bool ntc_mhot = (ntc_state & 0x40) != 0;
                bool ntc_mlow = (ntc_state & 0x20) != 0;
                bool ntc_cold = (ntc_state & 0x10) != 0;

                /* --- OCP / short-circuit state (0xFC @ 0xEA) ---
                 * bit0=VBUS OCP, bit1=VOUT OCP, bit2=Boost OCP, bit3=Boost short-circuit */
                uint8_t ocp_state = 0;
                ip5561_read_reg(g_ip5561_handle, 0xFC, &ocp_state, true);
                bool ocp_vbus    = (ocp_state & 0x01) != 0;
                bool ocp_vout    = (ocp_state & 0x02) != 0;
                bool ocp_boost   = (ocp_state & 0x04) != 0;
                bool ocp_boostsc = (ocp_state & 0x08) != 0;

                /* Compact fault flag string: empty when everything OK */
                char fault_str[64];
                if (ntc_hot || ntc_mhot || ntc_mlow || ntc_cold ||
                    ocp_vbus || ocp_vout || ocp_boost || ocp_boostsc) {
                    snprintf(fault_str, sizeof(fault_str),
                             "FLT:NTC[0x%02X]%s%s%s%s OCP[0x%02X]%s%s%s%s",
                             ntc_state,
                             ntc_hot ? " Hot" : "", ntc_mhot ? " MHot" : "",
                             ntc_mlow ? " MLow" : "", ntc_cold ? " Cold" : "",
                             ocp_state,
                             ocp_vbus ? " vbus" : "", ocp_vout ? " vout" : "",
                             ocp_boost ? " boost" : "", ocp_boostsc ? " SC" : "");
                } else {
                    snprintf(fault_str, sizeof(fault_str), "FLT:OK");
                }

                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[IP5561 Manage] CTL0=0x%02X | CHG: vbus=%d activ=%d done=%d phase=%d | "
                    "Vbat=%umV Ibat=%dmA | %s",
                    sys_ctl0, vbus_in, chg_activ, chg_done, chg_phase, vbat_mv, ibat, fault_str);

                /* --- OCP check: auto-clear VBUS/Boost OCP (0xFC @ 0xEA) --- */
                if (ocp_state & 0x05) {
                    SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                                    "[IP5561 Manage] OCP triggered (0x%02X), clearing...",
                                    ocp_state);
                    ip5561_write_reg(g_ip5561_handle, 0xFC, ocp_state | 0x05, true);
                }

                /* --- NTC alert (keep warning when any temperature flag set) --- */
                if (ntc_state & 0xF0) {
                    SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                                    "[IP5561 Manage] NTC alert (0x%02X): Hot=%d MHot=%d MLow=%d Cold=%d",
                                    ntc_state, ntc_hot, ntc_mhot, ntc_mlow, ntc_cold);
                }

                /* --- Reset detection via config fingerprint --- */
                if (g_ip5561_fingerprint.initialized) {
                    uint8_t cur_sys_ctl4 = 0;
                    ip5561_read_reg(g_ip5561_handle, 0x31, &cur_sys_ctl4, false);

                    if (cur_sys_ctl4 != g_ip5561_fingerprint.sys_ctl4) {
                        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                                        "[IP5561 Manage] Reset detected! SYS_CTL4: 0x%02X -> 0x%02X, re-applying config",
                                        g_ip5561_fingerprint.sys_ctl4, cur_sys_ctl4);
                        power_manage_apply_ip5561_config();
                    }
                }

                next_delay_ms = 60000;  /* 1 min for normal health check */
            } else {
                /* === Communication lost === */
                SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[IP5561 Manage] Comm lost, fingerprint %s (CTL4=0x%02X), marking NULL",
                    g_ip5561_fingerprint.initialized ? "EXISTS" : "NONE",
                    g_ip5561_fingerprint.sys_ctl4);

                /* NOTE: long wakeup (2s KEY) is disabled because it triggers
                 * IP5561 shutdown when the chip is already running, causing
                 * ESP32 power loss. Just mark handle as NULL and retry init
                 * on next cycle (VBUS plug-in will unlock IP5561). */
                ip5561_delete(g_ip5561_handle);
                g_ip5561_handle = NULL;
                /* Keep fingerprint — on reconnect we check if chip reset */
                next_delay_ms = 120000;  /* 2 min before next init attempt */
            }
        }
        
        //next_delay_ms = 5000;

        /* Wait with 10s granularity for quick stop response */
        uint32_t remaining = next_delay_ms;
        while (remaining >= 10000 && g_ip5561_manage_running) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            remaining -= 10000;
        }
        if (remaining > 0 && g_ip5561_manage_running) {
            vTaskDelay(pdMS_TO_TICKS(remaining));
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[IP5561 Manage] Task stopped");
    g_ip5561_manage_task = NULL;
    vTaskDelete(NULL);
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
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "[%s]: ✓ Communication OK, Voltage: %u mV", fuel_gauge_name(), voltage);
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
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 wakeup GPIO initialized (GPIO%d)",
                    IP5561_WAKEUP_GPIO);

    /* Step 1a: Configure INT (GPIO5) as input — common for V1/V2 */
    ip5561_int_gpio_init();

#ifdef SUPPORT_HARDWARE_V2
    /* V2.0: Initialize boost IC GPIOs */
    boost_gpio_init();
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Boost IC GPIOs initialized (EN: GPIO%d, MODE: GPIO%d)",
                    BOOST_PWR_EN_GPIO, BOOST_PWR_MODE_GPIO);

    /* Step 1b: Set I2C1 SDA/SCL (GPIO6/GPIO14) to high-Z + pullup.
     * IP5561 checks LED1/LED2 levels on wakeup to decide I2C mode. */
    ip5561_i2c_pins_high_z();

    /* Step 1c: Long wakeup DISABLED.
     * 2s KEY pulse triggers IP5561 shutdown when chip is already running,
     * causing ESP32 power loss. Manage task will retry init without wakeup. */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 long wakeup disabled");
#endif

    /* Step 1d: Wait for INT (GPIO5) sustained HIGH >=100ms — common for V1/V2.
     * This confirms IP5561 has entered I2C mode and is ready for communication. */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Waiting for IP5561 INT (GPIO%d)...", IP5561_INT_GPIO);
    bool int_ready = ip5561_wait_int_ready(2000);  /* 2s timeout */
    if (int_ready) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                        "IP5561 INT high for 100ms, I2C mode confirmed");
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                        "IP5561 INT timeout, trying I2C anyway");
    }

#ifdef SUPPORT_HARDWARE_V2
    /* Step 1e: Short KEY press (300ms) to activate IP5561 before I2C init */
    gpio_set_level(IP5561_WAKEUP_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(IP5561_WAKEUP_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
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

    /* Step 3: Initialize IP5561 (non-blocking, manage task will retry if failed) */
    ret = power_manage_init_ip5561();
    if (ret == ESP_OK) {
        ip5561_ok = true;
    } else {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 init failed - manage task will retry");
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
        xTaskCreate(sensor_monitor_task, "sensor_mon", 3072, NULL, 1, &g_sensor_task_handle);
    }

    /* Initial battery LED update (don't wait for first 60s cycle) */
    if (g_fuel_gauge_handle != NULL) {
        uint16_t init_soc = fuel_gauge_get_state_of_charge(g_fuel_gauge_handle);
        bool init_charging = power_manage_is_charging();
        led_set_battery_status(init_soc, init_charging, init_soc >= 100);
    }
    /* Signal LED: TT not started yet → red blink (TT off indication) */
    led_set_signal_status(LED_SIGNAL_TT_OFF);

    return ESP_OK;
}

/**
 * @brief Shared sensor monitor task (DA228EC + MMC5603)
 *
 * Reads sensors, caches data, sends BLE notification if report enabled.
 * Sensor data is read every 500ms; debug log printed every 30s.
 */

/* Debug calculation toggles (for hardware comparison testing) */
#define SENSOR_DEBUG_LOG_INTERVAL_MS   2000   /* Debug log interval (time-based) */
#define SENSOR_DEBUG_TILT_ANGLE        1       /* 1=calculate and log tilt angle */
#define SENSOR_DEBUG_COMPASS_HEADING   1       /* 1=calculate and log compass heading */

/* Sensor high-rate report intervals (active push when enabled) */
#define SENSOR_REPORT_FAST_INTERVAL_MS     50      /* 20Hz when idle */
#define SENSOR_REPORT_THROTTLE_INTERVAL_MS 500     /* 2Hz during voice call (avoid impacting voice) */
#define SENSOR_REPORT_SESSION_TIMEOUT_MS   30000   /* 30s no re-enable → auto-disable */

/* Idle sampling policy (configurable):
 *   1 = idle(report disabled)时任务阻塞休眠，不采样 → 省电(默认)
 *   0 = idle时仍以慢速后台采样(保持 cache/调试日志新鲜)，但不推送 */
#ifndef SENSOR_IDLE_SLEEP_ENABLE
#define SENSOR_IDLE_SLEEP_ENABLE           1
#endif
#define SENSOR_IDLE_BACKGROUND_MS          500     /* idle 后台采样间隔(仅当 IDLE_SLEEP=0 时生效) */

/* Calculate tilt angle (仰角/elevation) from accelerometer data.
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

/* Calculate compass heading (方位角/azimuth) from magnetometer data.
 * Returns heading in degrees (0=North, 90=East, 180=South, 270=West).
 * mx/my/mz in mG. */
static int16_t sensor_calc_heading(int32_t mx, int32_t my, int32_t mz)
{
    (void)mz;  /* 2D compass for simplicity */
    float heading = atan2f((float)my, (float)mx) * 180.0f / 3.14159265f;
    if (heading < 0) heading += 360.0f;
    return (int16_t)heading;
}

/* Public accessor: compute pointing angles from the latest cached sensor data.
 * 方位角=azimuth (from magnetometer), 仰角=elevation (from accelerometer). */
void power_manage_get_sensor_angles(int16_t *azimuth, float *elevation)
{
    int16_t az = 0;
    float el = 0.0f;

    if (g_sensor_cache.flags & 0x01) {           /* accel valid → elevation */
        el = sensor_calc_tilt(g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az);
    }
    if (g_sensor_cache.flags & 0x02) {           /* mag valid → azimuth */
        az = sensor_calc_heading(g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz);
    }

    if (azimuth) *azimuth = az;
    if (elevation) *elevation = el;
}

static void sensor_monitor_task(void *pvParameters)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Sensor monitor task started");

    uint32_t debug_log_ms = 0;   /* Time accumulator for debug log */

    extern bool spp_voice_server_is_call_active(void);

    while (1) {
        bool call_active = spp_voice_server_is_call_active();
        uint32_t delay_ms;

#if SENSOR_IDLE_SLEEP_ENABLE
        /* Power-save: when report is disabled, block until ENABLE wakes us.
         * No I2C reads while idle. */
        if (!g_sensor_cache.report_enabled) {
            xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
            debug_log_ms = 0;
            continue;  /* re-check report_enabled */
        }
        delay_ms = call_active ? SENSOR_REPORT_THROTTLE_INTERVAL_MS
                               : SENSOR_REPORT_FAST_INTERVAL_MS;
#else
        /* Always sample. Fast push when enabled; slow background when idle. */
        if (g_sensor_cache.report_enabled) {
            delay_ms = call_active ? SENSOR_REPORT_THROTTLE_INTERVAL_MS
                                   : SENSOR_REPORT_FAST_INTERVAL_MS;
        } else {
            delay_ms = SENSOR_IDLE_BACKGROUND_MS;
        }
#endif

        /* Read sensors */
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

        /* Push sample to APP only when report enabled (fire-and-forget NOTIFY; no retry) */
        if (g_sensor_cache.report_enabled && flags) {
            gatt_system_server_send_sensor_data();
        }

        /* Session timeout: auto-disable if APP hasn't re-enabled in time (active push only) */
        if (g_sensor_cache.report_enabled) {
            g_sensor_session_ms += delay_ms;
            if (g_sensor_session_ms >= SENSOR_REPORT_SESSION_TIMEOUT_MS) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "[SENSOR] report session timeout (%ums), auto-disable", g_sensor_session_ms);
                g_sensor_cache.report_enabled = false;
                continue;  /* next iteration sleeps / goes background */
            }
        }

        /* Debug log (time-based, independent of dynamic rate) */
        debug_log_ms += delay_ms;
        if (debug_log_ms >= SENSOR_DEBUG_LOG_INTERVAL_MS) {
            debug_log_ms = 0;

#if SENSOR_DEBUG_TILT_ANGLE && SENSOR_DEBUG_COMPASS_HEADING
            float tilt = sensor_calc_tilt(g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az);
            int16_t heading = sensor_calc_heading(g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[SENSOR] accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG | tilt=%.1f° heading=%d° (%ums)",
                g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az,
                g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz,
                tilt, heading, delay_ms);
#elif SENSOR_DEBUG_TILT_ANGLE
            float tilt = sensor_calc_tilt(g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[SENSOR] accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG | tilt=%.1f° (%ums)",
                g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az,
                g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz, tilt, delay_ms);
#elif SENSOR_DEBUG_COMPASS_HEADING
            int16_t heading = sensor_calc_heading(g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz);
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[SENSOR] accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG | heading=%d° (%ums)",
                g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az,
                g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz, heading, delay_ms);
#else
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                "[SENSOR] accel=[%d,%d,%d]mg mag=[%ld,%ld,%ld]mG (%ums)",
                g_sensor_cache.ax, g_sensor_cache.ay, g_sensor_cache.az,
                g_sensor_cache.mx, g_sensor_cache.my, g_sensor_cache.mz, delay_ms);
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
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
    if (enable) {
        g_sensor_session_ms = 0;  /* each ENABLE renews the 30s session */
        if (g_sensor_task_handle != NULL) {
            /* Wake the sampling task to start pushing immediately */
            xTaskNotifyGive(g_sensor_task_handle);
        }
    }
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

#if 1
    /* Start IP5561 manage task (always start — handles init retry + health check) */
    if (g_ip5561_manage_task == NULL) {
        g_ip5561_manage_running = true;
        BaseType_t ret = xTaskCreate(
            ip5561_manage_task,
            "ip5561_mgr",
            8192,
            NULL,
            2,  /* Priority: low (background management) */
            &g_ip5561_manage_task
        );

        if (ret != pdPASS) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to create IP5561 manage task");
            g_ip5561_manage_running = false;
            return ESP_FAIL;
        }
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 manage task started");
    }
 #endif   

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
    /* Stop IP5561 manage task */
    if (g_ip5561_manage_task != NULL) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Stopping IP5561 manage task...");
        g_ip5561_manage_running = false;

        int timeout = 20;  /* 2 second timeout */
        while (g_ip5561_manage_task != NULL && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (g_ip5561_manage_task != NULL) {
            SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 manage task stop timeout");
        } else {
            SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "IP5561 manage task stopped");
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
    return g_ip5561_manage_running || g_fuel_gauge_task_running;
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

    /* WPC is disabled in config — skip I2C read to avoid unnecessary traffic */
    return false;
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

    if (g_fuel_gauge_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Read battery data from fuel gauge (no IP5561 dependency) */
    int16_t current = fuel_gauge_get_current(g_fuel_gauge_handle);
    uint16_t soc = fuel_gauge_get_state_of_charge(g_fuel_gauge_handle);
    battery_status_t batt_status;
    fuel_gauge_get_battery_status(g_fuel_gauge_handle, &batt_status);

    /* Build status flags (same bit layout as before)
     * bit0: charger_enabled   — power flowing into battery
     * bit1: boost_enabled     — power flowing out of battery
     * bit2: charging          — active charging (current < -200mA)
     * bit3: discharging       — active discharging (current > 200mA)
     * bit4: charge_done       — full charged (FC flag or SOC>=99 + low current)
     */
    *status_flags = 0;

    if (current < -200) {
        /* Clearly charging */
        *status_flags |= 0x01;  /* bit0: charger active */
        *status_flags |= 0x04;  /* bit2: charging */
    } else if (current > 200) {
        /* Clearly discharging */
        *status_flags |= 0x02;  /* bit1: boost active */
        *status_flags |= 0x08;  /* bit3: discharging */
    } else if (!batt_status.DSG && current < -100) {
        /* Trickle charge: negative current >100mA + not in discharge mode */
        *status_flags |= 0x01;
        *status_flags |= 0x04;
    } else {
        /* Idle zone (|I| < 100mA): sensor offset, neither charging nor discharging */
    }

    /* Charge done: FC flag from fuel gauge, or SOC>=99 with near-zero current */
    if (batt_status.FC || (soc >= 99 && current > -100 && current < 100)) {
        *status_flags |= 0x10;  /* bit4: charge_done */
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG,
                    "Charging status: 0x%02X (I=%dmA, SOC=%u%%, DSG=%d, FC=%d)",
                    *status_flags, current, soc, batt_status.DSG, batt_status.FC);
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
