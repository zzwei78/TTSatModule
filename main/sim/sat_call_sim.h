/*
 * sat_call_sim.h - Satellite Call Simulation Engine
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef SAT_CALL_SIM_H
#define SAT_CALL_SIM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Enable/Disable simulation via macro (set to 0 to compile out) */
#ifndef ENABLE_SAT_CALL_SIM
#define ENABLE_SAT_CALL_SIM     1
#endif

/* Fixed test phone number */
#define SIM_TEST_NUMBER         "10086"

/* Simulation scenario configuration */
typedef enum {
    SIM_SCENARIO_NORMAL = 0,       /* Normal call (answered successfully) */
    SIM_SCENARIO_BUSY = 1,         /* Remote party busy */
    SIM_SCENARIO_NO_ANSWER = 2,    /* No answer (timeout) */
    SIM_SCENARIO_NETWORK_DROP = 3, /* Network drop during call (5-15s) */
    SIM_SCENARIO_REJECT = 5,       /* Remote party rejects */
} sim_scenario_t;

/* Simulation call state */
typedef enum {
    SIM_CALL_IDLE = 0,
    SIM_CALL_DIALING,       /* MO: ATD sent, waiting for orig/conf */
    SIM_CALL_ALERTING,      /* MO: Remote ringing */
    SIM_CALL_ACTIVE,        /* Call connected, voice data flowing */
    SIM_CALL_INCOMING,      /* MT: Incoming call, waiting for ATA */
    SIM_CALL_DISCONNECTING,  /* Call ending */
} sim_call_state_t;

/**
 * @brief Initialize simulation engine
 *
 * Called once during system init. Does NOT enable simulation.
 */
void sat_call_sim_init(void);

/**
 * @brief Check if simulation mode is enabled
 */
bool sat_call_sim_is_enabled(void);

/**
 * @brief Enable or disable simulation mode
 *
 * @param enable true to enable, false to disable
 * @param scenario Default scenario for calls
 * @return 0 on success, negative on error
 */
int sat_call_sim_set_enabled(bool enable, sim_scenario_t scenario);

/**
 * @brief Get current simulation scenario
 */
sim_scenario_t sat_call_sim_get_scenario(void);

/**
 * @brief Set simulated network parameters
 *
 * @param csq Signal quality (0-31, 99=unknown)
 * @param creg Registration status (0-5)
 */
void sat_call_sim_set_network(uint8_t csq, uint8_t creg);

/**
 * @brief Update GATT connection handle for URC push
 *
 * Called when SIM_CTRL is received via system command (not AT channel).
 * Without this, sim_net_push would have no conn_handle to send to.
 */
void sat_call_sim_set_conn_handle(uint16_t conn_handle);

/**
 * @brief Trigger an incoming call simulation
 *
 * @param delay_ms Delay before the first RING (0 = immediate)
 * @return 0 on success, negative on error
 */
int sat_call_sim_trigger_incoming(uint32_t delay_ms);

/**
 * @brief Handle GAT-side AT command in simulation mode
 *
 * Called by tt_module when simulation is enabled.
 * Generates simulated responses instead of forwarding to MUX.
 *
 * @param cmd AT command string
 * @param conn_handle GATT connection handle for responses
 * @return 0 on success, negative on error
 */
int sat_call_sim_handle_at_gatt(const char *cmd, uint16_t conn_handle);

/**
 * @brief Notify simulation that voice service was enabled
 *
 * Starts voice data injection if call is active.
 */
void sat_call_sim_voice_enabled(void);

/**
 * @brief Notify simulation that voice service was disabled
 *
 * Stops voice data injection.
 */
void sat_call_sim_voice_disabled(void);

/**
 * @brief Get current call state
 */
sim_call_state_t sat_call_sim_get_state(void);

/**
 * @brief Get current call state as string
 */
const char *sat_call_sim_state_str(sim_call_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* SAT_CALL_SIM_H */
