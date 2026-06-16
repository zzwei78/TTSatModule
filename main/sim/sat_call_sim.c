/*
 * sat_call_sim.c - Satellite Call Simulation Engine
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * Intercepts GATT-path AT commands and simulates satellite call flow.
 * LOCAL AT commands are NOT affected.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_partition.h"
#include "esp_partition.h"
#include "sim/sat_call_sim.h"
#include "ble/spp_at_server.h"
#include "audio/voice_packet_handler.h"
#include "system/syslog.h"

#if ENABLE_SAT_CALL_SIM

static const char *TAG = "SAT_SIM";

/* Audio frame size: 20ms @ 8kHz, 16-bit = 320 bytes = 160 samples */
#define SIM_AUDIO_FRAME_SAMPLES  160
#define SIM_AUDIO_FRAME_BYTES    (SIM_AUDIO_FRAME_SAMPLES * sizeof(int16_t))

/* Call timing constants (ms) - each state lasts a visible duration */
#define SIM_DIALING_DELAY_MS     3000    /* DIALING state: 3 seconds */
#define SIM_ALERTING_DELAY_MS    5000    /* ALERTING (ringing): 5 seconds */
#define SIM_CONN_DELAY_MS        5000    /* Active call holds before voice: 5 seconds */
#define SIM_BUSY_DELAY_MS        5000    /* Busy after alerting: 5 seconds */
#define SIM_NO_ANSWER_DELAY_MS   30000   /* No answer timeout: 30 seconds */
#define SIM_REJECT_DELAY_MS      3000    /* Reject after alerting: 3 seconds */
#define SIM_NET_DROP_MIN_MS      5000
#define SIM_NET_DROP_MAX_MS      15000
#define SIM_RING_INTERVAL_MS     2000
#define SIM_INCOMING_TIMEOUT_MS  30000
#define SIM_NET_DROP_MAX_MS      15000
#define SIM_RING_INTERVAL_MS     2000
#define SIM_INCOMING_TIMEOUT_MS  30000

/* Ringtone: 440Hz + 480Hz (North America standard) */
#define SIM_RINGTONE_FREQ1       440.0f
#define SIM_RINGTONE_FREQ2       480.0f
/* Ring pattern: 1s on, 2s off */
#define SIM_RING_ON_SAMPLES      (8000 * 1)   /* 1 second */
#define SIM_RING_OFF_SAMPLES     (8000 * 2)   /* 2 seconds */
#define SIM_RING_CYCLE_SAMPLES   (SIM_RING_ON_SAMPLES + SIM_RING_OFF_SAMPLES)

/* Voice: 600Hz tone (simple test tone, not comfort noise) */
#define SIM_VOICE_FREQ           600.0f

/* ============================================================
 * URC step types (explicit, no dynamic guessing)
 * ============================================================ */

enum {
    URC_NONE = 0,
    URC_ORIG_DIAL,           /* ^ORIG + ^DSCI(dialing=2) */
    URC_CONF_ALERT,          /* ^CONF + ^DSCI(alerting=3) */
    URC_CONN_ACTIVE,         /* ^CONN + ^DSCI(active=0) + start voice */
    URC_CEND_BUSY,           /* ^CEND busy + ^DSCI(disconnected=6) */
    URC_CEND_NOANSWER,       /* ^CEND no_answer + ^DSCI(disconnected=6) */
    URC_CEND_REJECT,         /* ^CEND reject + ^DSCI(disconnected=6) */
    URC_CEND_DROP,           /* ^CEND drop + ^DSCI(disconnected=6) */
    URC_END = 0xFF,
};

typedef struct {
    uint8_t urc_type;           /* URC type enum */
    uint32_t delay_ms;          /* Delay from previous step */
    sim_call_state_t state;     /* State to set (IDLE = no change) */
    bool start_voice;           /* Start voice injection */
} sim_step_t;

/* ---- Outgoing call sequences ---- */

/* NORMAL: ATD → dialing(3s) → alerting(5s, ringback tone) → connected(5s, voice) */
static const sim_step_t seq_normal[] = {
    { URC_ORIG_DIAL,    SIM_DIALING_DELAY_MS,  SIM_CALL_DIALING,   false },
    { URC_CONF_ALERT,   SIM_ALERTING_DELAY_MS, SIM_CALL_ALERTING,  true  },  /* start ringback */
    { URC_CONN_ACTIVE,  SIM_CONN_DELAY_MS,     SIM_CALL_ACTIVE,    true  },  /* start voice */
    { URC_END,          0,                     SIM_CALL_IDLE,      false },
};

/* BUSY: dialing → alerting(ringback) → busy */
static const sim_step_t seq_busy[] = {
    { URC_ORIG_DIAL,    SIM_DIALING_DELAY_MS,  SIM_CALL_DIALING,   false },
    { URC_CONF_ALERT,   SIM_ALERTING_DELAY_MS, SIM_CALL_ALERTING,  true  },  /* start ringback */
    { URC_CEND_BUSY,    SIM_BUSY_DELAY_MS,     SIM_CALL_IDLE,      false },
    { URC_END,          0,                     SIM_CALL_IDLE,      false },
};

/* NO_ANSWER: dialing → alerting(ringback) → timeout */
static const sim_step_t seq_no_answer[] = {
    { URC_ORIG_DIAL,    SIM_DIALING_DELAY_MS,  SIM_CALL_DIALING,   false },
    { URC_CONF_ALERT,   SIM_ALERTING_DELAY_MS, SIM_CALL_ALERTING,  true  },  /* start ringback */
    { URC_CEND_NOANSWER,SIM_NO_ANSWER_DELAY_MS,SIM_CALL_IDLE,      false },
    { URC_END,          0,                     SIM_CALL_IDLE,      false },
};

/* REJECT: dialing → alerting(ringback) → reject */
static const sim_step_t seq_reject[] = {
    { URC_ORIG_DIAL,    SIM_DIALING_DELAY_MS,  SIM_CALL_DIALING,   false },
    { URC_CONF_ALERT,   SIM_ALERTING_DELAY_MS, SIM_CALL_ALERTING,  true  },  /* start ringback */
    { URC_CEND_REJECT,  SIM_REJECT_DELAY_MS,   SIM_CALL_IDLE,      false },
    { URC_END,          0,                     SIM_CALL_IDLE,      false },
};

/* ---- Simulation context ---- */

typedef struct {
    bool enabled;
    sim_scenario_t scenario;
    sim_call_state_t state;
    uint8_t sim_csq;
    uint8_t sim_creg;
    uint16_t gatt_conn;
    esp_timer_handle_t urc_timer;
    TaskHandle_t voice_task;
    bool voice_active;
    uint32_t sample_pos;        /* Audio sample position (for continuous generation) */
    uint8_t call_id;            /* Call ID (1 for MO, 2 for MT) */
    bool voice_svc_enabled;     /* Track voice service enable state */
    bool incoming_pending;      /* True = one-shot timer for delayed incoming */
    uint32_t ring_count;        /* Ring counter for incoming timeout */
    char call_number[16];       /* Phone number for current call */
    /* PCM flash playback */
    const uint8_t *pcm_data;    /* mmap pointer to pcm_data partition (NULL if no PCM) */
    size_t pcm_size;            /* PCM data size in bytes */
    size_t pcm_pos;             /* Current playback position in bytes */
    esp_partition_mmap_handle_t pcm_mmap_handle;
} sim_context_t;

static sim_context_t g_sim = {
    .enabled = false,
    .scenario = SIM_SCENARIO_NORMAL,
    .state = SIM_CALL_IDLE,
    .sim_csq = 25,
    .sim_creg = 1,
    .gatt_conn = 0,
    .urc_timer = NULL,
    .voice_task = NULL,
    .voice_active = false,
    .sample_pos = 0,
    .call_id = 0,
    .voice_svc_enabled = false,
    .incoming_pending = false,
    .ring_count = 0,
    .pcm_data = NULL,
    .pcm_size = 0,
    .pcm_pos = 0,
    .pcm_mmap_handle = (esp_partition_mmap_handle_t)0,
};

/* Forward declarations for PCM */
static esp_err_t sim_pcm_init(void);
static void sim_pcm_deinit(void);

extern QueueHandle_t voice_data_queue;

/* Forward declarations */
static void sim_send_urc(const char *urc);
static void sim_send_response(const char *resp);
static void sim_start_voice_inject(void);
static void sim_stop_voice_inject(void);
static void sim_transition_state(sim_call_state_t new_state);
static void sim_urc_timer_cb(void *arg);
static void sim_start_urc_sequence(void);
static void sim_generate_ringtone(int16_t *buf, size_t samples);
static void sim_generate_voice(int16_t *buf, size_t samples);
static int sim_handle_atd(const char *cmd);
static int sim_handle_ata(void);
static int sim_handle_chup(void);
static int sim_handle_csq(void);
static int sim_handle_creg(void);
static int sim_handle_cpin(void);
static int sim_handle_clcc(void);
static int sim_handle_ok(void);
static void sim_send_dsci(uint8_t dsci_state);
static void sim_send_cend(uint8_t cause1, uint8_t cause2);
static void sim_call_end(void);

/* Current sequence state */
static const sim_step_t *g_current_seq = NULL;
static int g_seq_index = 0;

/* ============================================================
 * Public API
 * ============================================================ */

void sat_call_sim_init(void)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "Satellite call simulation engine initialized (disabled)");

    /* Create URC timer (not started) */
    esp_timer_create(&(esp_timer_create_args_t){
        .callback = sim_urc_timer_cb,
        .name = "sim_urc",
        .arg = NULL,
    }, &g_sim.urc_timer);
}

bool sat_call_sim_is_enabled(void)
{
    return g_sim.enabled;
}

int sat_call_sim_set_enabled(bool enable, sim_scenario_t scenario)
{
    if (enable && g_sim.state != SIM_CALL_IDLE) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
            "Cannot change sim state while call is active (state=%d)", g_sim.state);
        return -1;
    }

    g_sim.enabled = enable;
    g_sim.scenario = scenario;

    if (enable) {
        /* Map PCM partition for voice playback */
        sim_pcm_init();
    } else {
        /* Stop any active call */
        if (g_sim.state != SIM_CALL_IDLE) {
            sim_stop_voice_inject();
            sim_transition_state(SIM_CALL_IDLE);
        }
        if (g_sim.urc_timer) {
            esp_timer_stop(g_sim.urc_timer);
        }
        g_current_seq = NULL;
        g_sim.incoming_pending = false;
        /* Release PCM mapping */
        sim_pcm_deinit();
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "Simulation %s, scenario=%d", enable ? "ENABLED" : "DISABLED", scenario);
    return 0;
}

sim_scenario_t sat_call_sim_get_scenario(void)
{
    return g_sim.scenario;
}

void sat_call_sim_set_network(uint8_t csq, uint8_t creg)
{
    g_sim.sim_csq = csq;
    g_sim.sim_creg = creg;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "Sim network set: CSQ=%d, CREG=%d", csq, creg);
}

int sat_call_sim_trigger_incoming(uint32_t delay_ms)
{
    if (!g_sim.enabled) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Simulation not enabled");
        return -1;
    }
    if (g_sim.state != SIM_CALL_IDLE) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
            "Cannot trigger incoming, call active (state=%d)", g_sim.state);
        return -1;
    }

    g_sim.call_id = 2; /* MT call */
    strncpy(g_sim.call_number, SIM_TEST_NUMBER, sizeof(g_sim.call_number) - 1);
    g_sim.ring_count = 0;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "Incoming call simulation in %d ms", delay_ms);

    if (delay_ms > 0) {
        /* Set pending flag so timer callback knows to start incoming */
        g_sim.incoming_pending = true;
        esp_timer_start_once(g_sim.urc_timer, delay_ms * 1000ULL);
    } else {
        /* Immediate: start incoming call */
        sim_transition_state(SIM_CALL_INCOMING);
        sim_send_urc("RING");
        sim_send_dsci(4);  /* Incoming */
        g_sim.ring_count = 1;

        /* Start periodic ring timer */
        esp_timer_start_periodic(g_sim.urc_timer, SIM_RING_INTERVAL_MS * 1000ULL);
    }
    return 0;
}

sim_call_state_t sat_call_sim_get_state(void)
{
    return g_sim.state;
}

const char *sat_call_sim_state_str(sim_call_state_t state)
{
    switch (state) {
    case SIM_CALL_IDLE:         return "IDLE";
    case SIM_CALL_DIALING:      return "DIALING";
    case SIM_CALL_ALERTING:     return "ALERTING";
    case SIM_CALL_ACTIVE:       return "ACTIVE";
    case SIM_CALL_INCOMING:     return "INCOMING";
    case SIM_CALL_DISCONNECTING: return "DISCONNECTING";
    default:                    return "UNKNOWN";
    }
}

void sat_call_sim_voice_enabled(void)
{
    g_sim.voice_svc_enabled = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "Voice service enabled (state=%d, voice_active=%d)",
        g_sim.state, g_sim.voice_active);
    if (g_sim.state == SIM_CALL_ACTIVE) {
        sim_start_voice_inject();
    }
}

void sat_call_sim_voice_disabled(void)
{
    g_sim.voice_svc_enabled = false;
    sim_stop_voice_inject();
}

/* ============================================================
 * GATT AT Command Handler
 * ============================================================ */

int sat_call_sim_handle_at_gatt(const char *cmd, uint16_t conn_handle)
{
    if (cmd == NULL) return -1;

    g_sim.gatt_conn = conn_handle;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "SIM AT > %s", cmd);

    /* Add delay to simulate real module response time */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Call control commands */
    if (strncmp(cmd, "ATD", 3) == 0) {
        return sim_handle_atd(cmd);
    }
    if (strcmp(cmd, "ATA") == 0) {
        return sim_handle_ata();
    }
    if (strstr(cmd, "CHUP") != NULL || strcmp(cmd, "ATH") == 0) {
        return sim_handle_chup();
    }

    /* Network query commands */
    if (strstr(cmd, "CSQ") != NULL) {
        return sim_handle_csq();
    }
    if (strstr(cmd, "CREG") != NULL) {
        return sim_handle_creg();
    }
    if (strstr(cmd, "CPIN") != NULL) {
        return sim_handle_cpin();
    }

    /* SIM/device query commands */
    if (strstr(cmd, "CIMI") != NULL) {
        sim_send_response("+CIMI: 460116072498875\r\n\r\nOK");
        return 0;
    }
    if (strstr(cmd, "CCID") != NULL) {
        sim_send_response("+CCID: 89860326247551943462\r\n\r\nOK");
        return 0;
    }
    if (strstr(cmd, "CGMR") != NULL) {
        sim_send_response("+CGMR: HWA BP HS 6.0.5.260206\r\n\r\nOK");
        return 0;
    }
    if (strstr(cmd, "CGMM") != NULL) {
        sim_send_response("+CGMM: HTDM1310E\r\n\r\nOK");
        return 0;
    }

    /* Call status commands */
    if (strstr(cmd, "CLCC") != NULL) {
        return sim_handle_clcc();
    }
    if (strstr(cmd, "CLIP") != NULL) {
        return sim_handle_ok();
    }
    if (strstr(cmd, "DSCI") != NULL) {
        return sim_handle_ok();
    }

    /* Default: OK */
    return sim_handle_ok();
}

/* ============================================================
 * Individual AT Command Handlers
 * ============================================================ */

static int sim_handle_atd(const char *cmd)
{
    if (g_sim.state != SIM_CALL_IDLE) {
        sim_send_response("ERROR");
        return -1;
    }

    /* Extract phone number from ATD<number>; */
    const char *num_start = cmd + 3; /* skip "ATD" */
    size_t num_len = 0;
    while (num_start[num_len] >= '0' && num_start[num_len] <= '9') {
        num_len++;
    }
    if (num_len > 0 && num_len < sizeof(g_sim.call_number)) {
        memcpy(g_sim.call_number, num_start, num_len);
        g_sim.call_number[num_len] = '\0';
    } else {
        strncpy(g_sim.call_number, SIM_TEST_NUMBER, sizeof(g_sim.call_number) - 1);
    }

    g_sim.call_id = 1; /* MO call */
    sim_transition_state(SIM_CALL_DIALING);

    /* Start outgoing call URC sequence */
    sim_start_urc_sequence();

    sim_send_response("OK");
    return 0;
}

static int sim_handle_ata(void)
{
    if (g_sim.state != SIM_CALL_INCOMING) {
        sim_send_response("ERROR");
        return -1;
    }

    /* Stop ring timer */
    esp_timer_stop(g_sim.urc_timer);

    /* Answer the call */
    sim_send_response("OK");

    sim_send_urc("^CONN: 2,0");
    sim_send_dsci(0);  /* Active */

    sim_transition_state(SIM_CALL_ACTIVE);

    /* Reset voice idle timer so app gets a fresh 30s window */
    extern void spp_voice_server_reset_idle_timer(void);
    spp_voice_server_reset_idle_timer();

    /* Start voice if voice service is enabled */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "ATA answered: voice_svc=%d, starting inject", g_sim.voice_svc_enabled);
    if (g_sim.voice_svc_enabled) {
        sim_start_voice_inject();
    }

    /* For REJECT scenario, schedule immediate disconnect after answering */
    if (g_sim.scenario == SIM_SCENARIO_REJECT) {
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
            "REJECT scenario: will disconnect after %d ms", SIM_REJECT_DELAY_MS);
        esp_timer_start_once(g_sim.urc_timer, SIM_REJECT_DELAY_MS * 1000ULL);
    }
    /* For NETWORK_DROP scenario, schedule random disconnect */
    else if (g_sim.scenario == SIM_SCENARIO_NETWORK_DROP) {
        uint32_t drop_ms = SIM_NET_DROP_MIN_MS +
            (esp_random() % (SIM_NET_DROP_MAX_MS - SIM_NET_DROP_MIN_MS));
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
            "NETWORK_DROP scheduled in %d ms", drop_ms);
        esp_timer_start_once(g_sim.urc_timer, drop_ms * 1000ULL);
    }

    return 0;
}

static int sim_handle_chup(void)
{
    if (g_sim.state == SIM_CALL_IDLE) {
        sim_send_response("ERROR");
        return -1;
    }

    esp_timer_stop(g_sim.urc_timer);
    sim_send_response("OK");
    sim_call_end();
    return 0;
}

static int sim_handle_csq(void)
{
    char resp[64];
    snprintf(resp, sizeof(resp), "+CSQ: %d,0\r\n\r\nOK", g_sim.sim_csq);
    sim_send_response(resp);
    return 0;
}

static int sim_handle_creg(void)
{
    char resp[64];
    snprintf(resp, sizeof(resp), "+CREG: 0,%d\r\n\r\nOK", g_sim.sim_creg);
    sim_send_response(resp);
    return 0;
}

static int sim_handle_cpin(void)
{
    sim_send_response("+CPIN: READY, left times = 3\r\n\r\nOK");
    return 0;
}

static int sim_handle_clcc(void)
{
    char resp[128];
    int offset = 0;

    if (g_sim.state == SIM_CALL_ACTIVE || g_sim.state == SIM_CALL_DIALING ||
        g_sim.state == SIM_CALL_ALERTING || g_sim.state == SIM_CALL_INCOMING) {
        int dir = (g_sim.call_id == 2) ? 1 : 0; /* 0=MO, 1=MT */
        int stat = 0;
        switch (g_sim.state) {
        case SIM_CALL_DIALING:   stat = 2; break;
        case SIM_CALL_ALERTING:  stat = 3; break;
        case SIM_CALL_INCOMING:  stat = 4; break;
        case SIM_CALL_ACTIVE:    stat = 0; break;
        default: break;
        }
        offset = snprintf(resp, sizeof(resp), "+CLCC: %d,%d,%d,0,0,\"%s\",129\r\n\r\n",
            g_sim.call_id, dir, stat, g_sim.call_number);
    }
    snprintf(resp + offset, sizeof(resp) - offset, "OK");
    sim_send_response(resp);
    return 0;
}

static int sim_handle_ok(void)
{
    sim_send_response("OK");
    return 0;
}

/* ============================================================
 * URC Generation & State Machine
 * ============================================================ */

/* Helper: get call_id_type for ^DSCI and ^CEND */
static int sim_call_id_type(void)
{
    return (g_sim.call_id == 2) ? 1 : 0;  /* 0=MO, 1=MT */
}

/* Helper: send ^DSCI with given state */
static void sim_send_dsci(uint8_t dsci_state)
{
    char urc[80];
    snprintf(urc, sizeof(urc), "^DSCI: %d,%d,%d,0,0,\"%s\",129",
        g_sim.call_id, sim_call_id_type(), dsci_state, g_sim.call_number);
    sim_send_urc(urc);
}

/* Helper: send ^CEND */
static void sim_send_cend(uint8_t cause1, uint8_t cause2)
{
    char urc[64];
    snprintf(urc, sizeof(urc), "^CEND: %d,%d,%d,%d,0",
        g_sim.call_id, sim_call_id_type(), cause1, cause2);
    sim_send_urc(urc);
}

/* Helper: end the call (send CEND + DSCI disconnect, reset state) */
static void sim_call_end(void)
{
    sim_stop_voice_inject();

    /* Determine cause based on current state */
    uint8_t cause1 = 101, cause2 = 17;
    if (g_sim.state == SIM_CALL_INCOMING) {
        /* User rejected incoming call */
        cause1 = 105;
    }

    sim_send_cend(cause1, cause2);
    sim_send_dsci(6);  /* Disconnected */

    sim_transition_state(SIM_CALL_IDLE);
}

/* Generate URC for a given step type */
static void sim_emit_urc(uint8_t urc_type)
{
    char urc[64];

    switch (urc_type) {
    case URC_ORIG_DIAL:
        snprintf(urc, sizeof(urc), "^ORIG: %d,0", g_sim.call_id);
        sim_send_urc(urc);
        sim_send_dsci(2);  /* Dialing */
        break;
    case URC_CONF_ALERT:
        snprintf(urc, sizeof(urc), "^CONF: %d,0", g_sim.call_id);
        sim_send_urc(urc);
        vTaskDelay(pdMS_TO_TICKS(1000));
        sim_send_dsci(3);  /* Alerting */
        break;
    case URC_CONN_ACTIVE:
        snprintf(urc, sizeof(urc), "^CONN: %d,0", g_sim.call_id);
        sim_send_urc(urc);
        vTaskDelay(pdMS_TO_TICKS(1000));
        sim_send_dsci(0);  /* Active */
        /* Reset voice idle timer so app gets a fresh 30s window */
        extern void spp_voice_server_reset_idle_timer(void);
        spp_voice_server_reset_idle_timer();
        break;
    case URC_CEND_BUSY:
        sim_send_cend(103, 17);
        sim_send_dsci(6);
        sim_transition_state(SIM_CALL_IDLE);
        break;
    case URC_CEND_NOANSWER:
        sim_send_cend(102, 19);
        sim_send_dsci(6);
        sim_transition_state(SIM_CALL_IDLE);
        break;
    case URC_CEND_REJECT:
        sim_send_cend(105, 17);
        sim_send_dsci(6);
        sim_transition_state(SIM_CALL_IDLE);
        break;
    case URC_CEND_DROP:
        sim_send_cend(101, 3);
        sim_send_dsci(6);
        sim_stop_voice_inject();
        sim_transition_state(SIM_CALL_IDLE);
        break;
    default:
        break;
    }
}

static void sim_start_urc_sequence(void)
{
    switch (g_sim.scenario) {
    case SIM_SCENARIO_NORMAL:       g_current_seq = seq_normal; break;
    case SIM_SCENARIO_BUSY:         g_current_seq = seq_busy; break;
    case SIM_SCENARIO_NO_ANSWER:    g_current_seq = seq_no_answer; break;
    case SIM_SCENARIO_REJECT:       g_current_seq = seq_reject; break;
    case SIM_SCENARIO_NETWORK_DROP: g_current_seq = seq_normal; break;
    default:                        g_current_seq = seq_normal; break;
    }
    g_seq_index = 0;

    /* Start first step */
    const sim_step_t *first = &g_current_seq[0];
    if (first->urc_type != URC_END) {
        /* First step always has delay, schedule timer */
        if (first->delay_ms > 0) {
            esp_timer_start_once(g_sim.urc_timer, first->delay_ms * 1000ULL);
        } else {
            /* Zero delay: process immediately by calling callback logic */
            /* This shouldn't happen in practice, but handle it */
            sim_emit_urc(first->urc_type);
            if (first->state != SIM_CALL_IDLE) {
                sim_transition_state(first->state);
            }
            if (first->start_voice && g_sim.voice_svc_enabled) {
                sim_start_voice_inject();
            }
            /* Move to next step */
            g_seq_index = 1;
            const sim_step_t *next = &g_current_seq[1];
            if (next->urc_type != URC_END && next->delay_ms > 0) {
                esp_timer_start_once(g_sim.urc_timer, next->delay_ms * 1000ULL);
            }
        }
    }
}

static void sim_urc_timer_cb(void *arg)
{
    /* ---- Delayed incoming call trigger ---- */
    if (g_sim.incoming_pending) {
        g_sim.incoming_pending = false;
        sim_transition_state(SIM_CALL_INCOMING);
        sim_send_urc("RING");
        sim_send_dsci(4);  /* Incoming */
        g_sim.ring_count = 1;
        /* Start periodic ring */
        esp_timer_start_periodic(g_sim.urc_timer, SIM_RING_INTERVAL_MS * 1000ULL);
        return;
    }

    /* ---- Incoming call: periodic RING ---- */
    if (g_sim.state == SIM_CALL_INCOMING) {
        g_sim.ring_count++;
        /* Check incoming timeout */
        uint32_t elapsed_ms = g_sim.ring_count * SIM_RING_INTERVAL_MS;
        if (elapsed_ms >= SIM_INCOMING_TIMEOUT_MS) {
            /* Timeout: end incoming call */
            esp_timer_stop(g_sim.urc_timer);
            sim_send_cend(102, 19);  /* No answer */
            sim_send_dsci(6);        /* Disconnected */
            sim_transition_state(SIM_CALL_IDLE);
            return;
        }
        sim_send_urc("RING");
        sim_send_dsci(4);  /* Incoming */
        return;
    }

    /* ---- Active call: scheduled disconnect (NETWORK_DROP or REJECT) ---- */
    if (g_sim.state == SIM_CALL_ACTIVE) {
        /* REJECT scenario on incoming call answered */
        if (g_sim.scenario == SIM_SCENARIO_REJECT) {
            sim_send_cend(105, 17);
            sim_send_dsci(6);
            sim_stop_voice_inject();
            sim_transition_state(SIM_CALL_IDLE);
            return;
        }
        /* NETWORK_DROP scenario */
        if (g_sim.scenario == SIM_SCENARIO_NETWORK_DROP) {
            sim_emit_urc(URC_CEND_DROP);
            return;
        }
    }

    /* ---- Outgoing call URC sequence ---- */
    if (g_current_seq == NULL) return;

    const sim_step_t *step = &g_current_seq[g_seq_index];
    if (step->urc_type == URC_END) {
        g_current_seq = NULL;
        return;
    }

    /* Emit URC */
    sim_emit_urc(step->urc_type);

    /* State transition (skip if URC already did it, e.g. CEND steps) */
    if (step->state != SIM_CALL_IDLE && g_sim.state != SIM_CALL_IDLE) {
        sim_transition_state(step->state);
    }

    /* Start voice if needed */
    if (step->start_voice && g_sim.voice_svc_enabled) {
        sim_start_voice_inject();
    }

    /* Schedule next step */
    g_seq_index++;
    const sim_step_t *next = &g_current_seq[g_seq_index];
    if (next->urc_type != URC_END && next->delay_ms > 0) {
        esp_timer_start_once(g_sim.urc_timer, next->delay_ms * 1000ULL);
    } else if (next->urc_type == URC_END) {
        g_current_seq = NULL;
    }
}

/* ============================================================
 * Voice Data Injection
 * ============================================================ */

static void sim_voice_inject_task(void *arg)
{
    int16_t frame[SIM_AUDIO_FRAME_SAMPLES];
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Voice inject task started");

    while (g_sim.voice_active) {
        /* Generate audio based on call state */
        if (g_sim.state == SIM_CALL_ALERTING) {
            /* Ringback tone during outgoing call alerting */
            sim_generate_ringtone(frame, SIM_AUDIO_FRAME_SAMPLES);
        } else if (g_sim.state == SIM_CALL_INCOMING) {
            /* Ringtone during incoming call */
            sim_generate_ringtone(frame, SIM_AUDIO_FRAME_SAMPLES);
        } else if (g_sim.state == SIM_CALL_ACTIVE) {
            /* Active call: play PCM from flash, or fallback to synthetic tone */
            if (g_sim.pcm_data != NULL && g_sim.pcm_size >= SIM_AUDIO_FRAME_BYTES) {
                memcpy(frame, g_sim.pcm_data + g_sim.pcm_pos, SIM_AUDIO_FRAME_BYTES);
                g_sim.pcm_pos += SIM_AUDIO_FRAME_BYTES;
                if (g_sim.pcm_pos + SIM_AUDIO_FRAME_BYTES > g_sim.pcm_size) {
                    g_sim.pcm_pos = 0;  /* Loop */
                }
            } else {
                sim_generate_voice(frame, SIM_AUDIO_FRAME_SAMPLES);
            }
        } else {
            break;
        }

        /* Send to voice_data_queue */
        if (voice_data_queue != NULL) {
            if (xQueueSend(voice_data_queue, frame, pdMS_TO_TICKS(10)) != pdPASS) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
                    "Voice queue full, frame dropped");
            }
        }

        /* Timing based on frame mode:
         * 1-frame: 1 frame per 20ms
         * 3-frame: inject 3 frames back-to-back, then delay 60ms */
        uint8_t frame_mode = voice_packet_get_frame_mode();
        if (frame_mode <= 1) {
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            /* For 3-frame mode, inject remaining frames then delay 60ms */
            for (int i = 1; i < frame_mode; i++) {
                /* Generate next frame */
                if (g_sim.state == SIM_CALL_ACTIVE &&
                    g_sim.pcm_data != NULL && g_sim.pcm_size >= SIM_AUDIO_FRAME_BYTES) {
                    memcpy(frame, g_sim.pcm_data + g_sim.pcm_pos, SIM_AUDIO_FRAME_BYTES);
                    g_sim.pcm_pos += SIM_AUDIO_FRAME_BYTES;
                    if (g_sim.pcm_pos + SIM_AUDIO_FRAME_BYTES > g_sim.pcm_size) {
                        g_sim.pcm_pos = 0;
                    }
                } else {
                    sim_generate_voice(frame, SIM_AUDIO_FRAME_SAMPLES);
                }
                if (voice_data_queue != NULL) {
                    xQueueSend(voice_data_queue, frame, pdMS_TO_TICKS(10));
                }
            }
            vTaskDelay(pdMS_TO_TICKS(60));
        }
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Voice inject task stopped");
    g_sim.voice_task = NULL;
    vTaskDelete(NULL);
}

static void sim_start_voice_inject(void)
{
    if (g_sim.voice_active && g_sim.voice_task != NULL) {
        return; /* Already running */
    }

    g_sim.voice_active = true;
    g_sim.sample_pos = 0;
    g_sim.pcm_pos = 0;  /* Reset PCM playback position */

    BaseType_t ret = xTaskCreatePinnedToCore(
        sim_voice_inject_task,
        "sim_voice",
        4096,
        NULL,
        5,  /* Same priority as voice tasks */
        &g_sim.voice_task,
        0   /* CPU0 (same as encode task) */
    );

    if (ret != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "Failed to create voice inject task");
        g_sim.voice_active = false;
    }
}

static void sim_stop_voice_inject(void)
{
    g_sim.voice_active = false;
    /* Task will exit on next loop iteration */
}

/* ============================================================
 * PCM Flash Playback
 * ============================================================ */

static esp_err_t sim_pcm_init(void)
{
    /* Find pcm_data partition */
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "pcm_data");
    if (part == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
            "pcm_data partition not found, using synthetic audio");
        return ESP_ERR_NOT_FOUND;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "pcm_data partition: offset=0x%x, size=0x%x (%d KB)",
        part->address, part->size, part->size / 1024);

    /* Check if partition has valid data (non-0xFF) */
    uint8_t first_byte;
    esp_partition_read(part, 0, &first_byte, 1);
    if (first_byte == 0xFF) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
            "pcm_data partition is empty, using synthetic audio");
        return ESP_ERR_NOT_FOUND;
    }

    /* Read 4-byte header: actual PCM data size (little-endian uint32) */
    uint32_t pcm_data_len = 0;
    esp_partition_read(part, 0, &pcm_data_len, sizeof(pcm_data_len));

    /* Sanity check: data length must fit in partition and be frame-aligned */
    if (pcm_data_len == 0 || pcm_data_len > part->size - 4 ||
        (pcm_data_len % SIM_AUDIO_FRAME_BYTES) != 0) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
            "pcm_data invalid header: len=%u, using synthetic audio", pcm_data_len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Memory-map the partition (read-only, zero-copy) */
    const uint8_t *mapped = NULL;
    esp_err_t ret = esp_partition_mmap(part, 0, part->size,
        ESP_PARTITION_MMAP_DATA, (const void **)&mapped,
        &g_sim.pcm_mmap_handle);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
            "Failed to mmap pcm_data: %s", esp_err_to_name(ret));
        return ret;
    }

    /* PCM data starts after 4-byte header */
    g_sim.pcm_data = mapped + 4;
    g_sim.pcm_size = pcm_data_len;
    g_sim.pcm_pos = 0;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "PCM playback ready: %d KB (%.1f seconds @ 8kHz 16-bit mono)",
        pcm_data_len / 1024, (float)pcm_data_len / 320.0f / 50.0f);
    return ESP_OK;
}

static void sim_pcm_deinit(void)
{
    if (g_sim.pcm_data != NULL) {
        esp_partition_munmap(g_sim.pcm_mmap_handle);
        g_sim.pcm_data = NULL;
        g_sim.pcm_size = 0;
        g_sim.pcm_pos = 0;
        g_sim.pcm_mmap_handle = 0;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "PCM playback released");
    }
}

/* ============================================================
 * Audio Generation
 * ============================================================ */

static void sim_generate_ringtone(int16_t *buf, size_t samples)
{
    float sample_rate = 8000.0f;
    uint32_t pos = g_sim.sample_pos;

    for (size_t i = 0; i < samples; i++) {
        uint32_t cycle_pos = pos % SIM_RING_CYCLE_SAMPLES;

        if (cycle_pos < SIM_RING_ON_SAMPLES) {
            /* Ring ON: dual-tone 440Hz + 480Hz */
            float t = (float)pos / sample_rate;
            float val = 0.5f * sinf(2.0f * (float)M_PI * SIM_RINGTONE_FREQ1 * t)
                      + 0.5f * sinf(2.0f * (float)M_PI * SIM_RINGTONE_FREQ2 * t);
            buf[i] = (int16_t)(val * 8000.0f); /* ~1/4 amplitude */
        } else {
            /* Ring OFF: silence */
            buf[i] = 0;
        }
        pos++;
    }
    g_sim.sample_pos = pos;
}

static void sim_generate_voice(int16_t *buf, size_t samples)
{
    /* No PCM data available — generate silence for ACTIVE state.
     * The 600Hz test tone was a debug placeholder that should not
     * play during normal simulated calls.  Real audio comes from
     * the PCM flash partition or from the TT module. */
    memset(buf, 0, samples * sizeof(int16_t));
}

/* ============================================================
 * Helpers
 * ============================================================ */

static void sim_send_urc(const char *urc)
{
    if (g_sim.gatt_conn == 0) return;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "SIM URC < %s", urc);

    /* Add \r\n prefix and suffix */
    char buf[256];
    snprintf(buf, sizeof(buf), "\r\n%s\r\n", urc);
    spp_at_server_send_response_async(g_sim.gatt_conn,
        (const uint8_t *)buf, strlen(buf));
}

static void sim_send_response(const char *resp)
{
    if (g_sim.gatt_conn == 0) return;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG, "SIM RSP < %s", resp);

    char buf[256];
    snprintf(buf, sizeof(buf), "\r\n%s\r\n", resp);
    spp_at_server_send_response_async(g_sim.gatt_conn,
        (const uint8_t *)buf, strlen(buf));
}

static void sim_transition_state(sim_call_state_t new_state)
{
    if (g_sim.state == new_state) return;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, TAG,
        "State: %s -> %s",
        sat_call_sim_state_str(g_sim.state),
        sat_call_sim_state_str(new_state));
    g_sim.state = new_state;
}

#endif /* ENABLE_SAT_CALL_SIM */
