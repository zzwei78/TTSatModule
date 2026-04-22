/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "console/console.h"
#include "mbedtls/base64.h"
#include "audio/audiosvc.h"
#include "audio/base64.h"
#include "audio/voice_packet_handler.h"
#include "tt/tt_module.h"
#include "tt/gsm0710_manager.h"
#include "system/syslog.h"

static const char *TAG = "VOICE_PKT";

/* Module state */
typedef enum {
    VOICE_STATE_UNINITIALIZED = 0,
    VOICE_STATE_RUNNING,
    VOICE_STATE_STOPPED
} voice_state_t;

static volatile voice_state_t g_voice_state = VOICE_STATE_UNINITIALIZED;

/* Statistics for monitoring */
typedef struct {
    uint32_t uplink_packets_total;
    uint32_t uplink_packets_dropped;
    uint32_t uplink_encode_errors;
    uint32_t downlink_packets_total;
    uint32_t downlink_packets_dropped;
    uint32_t downlink_decode_errors;
} voice_stats_t;

static voice_stats_t g_voice_stats = {0};

/* 20ms Timer Statistics */
typedef struct {
    uint32_t total_ticks;           /* Total timer ticks (runtime / 20ms) */
    uint32_t data_processed;        /* Ticks with data processed */
    uint32_t data_skipped;          /* Ticks with no data (empty queue) */
    uint32_t queue_max_depth;       /* Maximum queue depth observed */
    uint32_t timer_auto_starts;     /* Timer auto-start count */
    uint32_t timer_auto_stops;      /* Timer auto-stop count */
} voice_20ms_stats_t;

static voice_20ms_stats_t g_20ms_stats = {0};

/* 20ms Timer Control (Downlink - Decode) */
static esp_timer_handle_t g_voice_20ms_timer = NULL;
static bool g_decode_timer_running = false;
static uint8_t g_decode_empty_count = 0;      /* Consecutive empty queue count */
static SemaphoreHandle_t g_decode_timer_mutex = NULL;

/* 20ms Timer Control (Uplink - Encode) */
static esp_timer_handle_t g_encode_20ms_timer = NULL;
static bool g_encode_timer_running = false;
static uint8_t g_encode_empty_count = 0;
static SemaphoreHandle_t g_encode_timer_mutex = NULL;

#define EMPTY_COUNT_THRESHOLD 10       /* Stop timer after 200ms (10 * 20ms) with no data */

/* Voice data output callback (for uplink: TT -> BLE) */
/* Note: These callbacks are set once during init and read by tasks, so volatile is sufficient */
static volatile voice_data_output_callback_t g_voice_output_callback = NULL;
static void * volatile g_voice_output_user_data = NULL;

/* Voice data downlink callback (for downlink: BLE -> TT via MUX ch9) */
static volatile voice_data_downlink_callback_t g_voice_downlink_callback = NULL;
static void * volatile g_voice_downlink_user_data = NULL;

/* Voice data queues */
QueueHandle_t voice_data_queue = NULL;
QueueHandle_t voice_raw_data_queue = NULL;

/* Task handles */
static TaskHandle_t g_encode_task_handle = NULL;
static TaskHandle_t g_decode_task_handle = NULL;

/* Constants */
#define VOICE_QUEUE_SIZE            18
#define VOICE_TASK_STACK_SIZE       8192
#define VOICE_TASK_PRIORITY         5
#define AT_AUDPCM_PREFIX            "AT^AUDPCM=\""
#define AT_AUDPCM_SUFFIX            "\""
#define AMRNB_MAX_FRAME_SIZE        32
#define VOICE_MAX_AMR_DATA_SIZE     (AMRNB_MAX_FRAME_SIZE * VOICE_MAX_FRAMES_PER_CMD)

/* Auto-detected frames per AT command (updated from downlink pattern) */
static volatile uint8_t g_frames_per_cmd = 1;

/* Manual frame mode lock: when true, downlink auto-detection is disabled */
static volatile bool g_frame_mode_manual = false;

/**
 * @brief 20ms Timer Callback for Decode (ISR context)
 *
 * This callback is triggered every 20ms by the hardware timer.
 * It notifies the decode task to process one frame of data.
 */
static void IRAM_ATTR voice_20ms_timer_callback(void* arg)
{
    TaskHandle_t task = (TaskHandle_t)arg;
    BaseType_t higher_woken = pdFALSE;

    /* Notify decode task: 20ms tick arrived */
    vTaskNotifyGiveFromISR(task, &higher_woken);
    portYIELD_FROM_ISR(higher_woken);
}

/**
 * @brief 20ms Timer Callback for Encode (ISR context)
 *
 * This callback is triggered every 20ms by the hardware timer.
 * It notifies the encode task to process one frame of data.
 */
static void IRAM_ATTR voice_encode_20ms_timer_callback(void* arg)
{
    TaskHandle_t task = (TaskHandle_t)arg;
    BaseType_t higher_woken = pdFALSE;

    /* Notify encode task: 20ms tick arrived */
    vTaskNotifyGiveFromISR(task, &higher_woken);
    portYIELD_FROM_ISR(higher_woken);
}


/**
 * @brief Voice decode task (downlink: AT command -> Base64 -> AMRNB -> PCM)
 *
 * Runs on CPU1 for decoding operations, paced by 20ms timer
 */
static void voice_decode_task(void *pvParameters)
{
    (void)pvParameters;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice decode task started on CPU1 (20ms auto-paced mode)");

    voice_raw_data_t raw_data;
    uint8_t amrnb_data[VOICE_MAX_AMR_DATA_SIZE];
    int16_t pcm_data[AUDIO_FRAME_SIZE];
    uint8_t pcm_combined[VOICE_MAX_FRAMES_PER_CMD * AUDIO_FRAME_SIZE];

    const size_t prefix_len = strlen(AT_AUDPCM_PREFIX);
    const size_t suffix_len = strlen(AT_AUDPCM_SUFFIX);

    while (1) {
        /* Wait for 20ms timer tick */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        g_20ms_stats.total_ticks++;

        /* Non-blocking receive from queue */
        if (xQueueReceive(voice_raw_data_queue, &raw_data, 0) == pdPASS) {
            /* ===== Data available: process it ===== */
            g_20ms_stats.data_processed++;
            g_decode_empty_count = 0;  /* Reset empty counter */
            g_voice_stats.downlink_packets_total++;

            SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                           "[Tick #%u] Processing packet (%u bytes)",
                           g_20ms_stats.total_ticks, raw_data.data_len);

            /* Validate AT command format: AT^AUDPCM="base64_data" */
            if (raw_data.data_len < prefix_len + suffix_len + 1) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Invalid AT cmd length: %d", raw_data.data_len);
                g_voice_stats.downlink_decode_errors++;
                continue;
            }

            if (strncmp((char *)raw_data.data, AT_AUDPCM_PREFIX, prefix_len) != 0) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Invalid AT cmd prefix");
                g_voice_stats.downlink_decode_errors++;
                continue;
            }

            if (strncmp((char *)raw_data.data + raw_data.data_len - suffix_len,
                        AT_AUDPCM_SUFFIX, suffix_len) != 0) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Invalid AT cmd suffix");
                g_voice_stats.downlink_decode_errors++;
                continue;
            }

            /* Extract base64 data */
            char *base64_data = (char *)raw_data.data + prefix_len;
            size_t base64_len = raw_data.data_len - prefix_len - suffix_len;

            SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Base64 extracted: %d bytes", base64_len);

            /* Base64 decode to AMRNB */
            size_t amrnb_len = 0;
            int ret = mbedtls_base64_decode(amrnb_data, sizeof(amrnb_data), &amrnb_len,
                                          (const unsigned char *)base64_data, base64_len);

            if (ret != 0 || amrnb_len == 0) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Base64 decode failed: %d", ret);
                g_voice_stats.downlink_decode_errors++;
                continue;
            }

            /* Calculate frame count (each AMR-NB frame is AMRNB_MAX_FRAME_SIZE bytes) */
            int frame_count = amrnb_len / AMRNB_MAX_FRAME_SIZE;
            if (amrnb_len % AMRNB_MAX_FRAME_SIZE != 0 || frame_count < 1 || frame_count > VOICE_MAX_FRAMES_PER_CMD) {
                SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                    "Invalid AMR data: %d bytes, not aligned to %d-byte frames",
                    amrnb_len, AMRNB_MAX_FRAME_SIZE);
                g_voice_stats.downlink_decode_errors++;
                continue;
            }

            /* Update uplink mode to match downlink pattern (auto-detect) */
            if (!g_frame_mode_manual && g_frames_per_cmd != frame_count) {
                SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                    "Frame mode auto-detected: %d -> %d frames/AT", g_frames_per_cmd, frame_count);
                g_frames_per_cmd = frame_count;
            }

            SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                "AMRNB decoded: %d bytes (%d frames)", amrnb_len, frame_count);

            /* Decode each AMR frame into combined PCM buffer */
            int total_pcm_len = 0;
            int decoded_frames = 0;

            for (int i = 0; i < frame_count; i++) {
                int pcm_len = audio_decode(&amrnb_data[i * AMRNB_MAX_FRAME_SIZE],
                                           AMRNB_MAX_FRAME_SIZE, pcm_data);

                if (pcm_len <= 0) {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                        "AMRNB decode failed for frame %d/%d", i + 1, frame_count);
                    g_voice_stats.downlink_decode_errors++;
                    continue;
                }

                SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                    "PCM decoded frame %d/%d: %d bytes", i + 1, frame_count, pcm_len);

                /* Append to combined buffer */
                memcpy(pcm_combined + total_pcm_len, pcm_data, pcm_len);
                total_pcm_len += pcm_len;
                decoded_frames++;
            }

            /* Send combined PCM via downlink callback (MUX: one send, loopback: split enqueue) */
            if (decoded_frames > 0) {
                if (g_voice_downlink_callback != NULL) {
                    g_voice_downlink_callback(pcm_combined, total_pcm_len,
                                             g_voice_downlink_user_data);
                    SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                        "[Tick #%u] %d/%d frames decoded (%d bytes PCM), sent via callback",
                        g_20ms_stats.total_ticks, decoded_frames, frame_count, total_pcm_len);
                } else {
                    SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                        "Downlink callback not registered, %d frames dropped", decoded_frames);
                    g_voice_stats.downlink_packets_dropped++;
                }
            }

        } else {
            /* ===== No data: skip this tick ===== */
            g_20ms_stats.data_skipped++;
            g_decode_empty_count++;

            /* Log only if abnormal delay for the current mode:
             * - 1-frame: warn if > 1 raw tick (should be 20ms)
             * - 3-frame: warn if > 3 raw ticks (should be 60ms)
             */
            int expected_gap = g_frames_per_cmd;
            int ms_per_tick = 20 * g_frames_per_cmd;

            /* Log periodically */
            if (g_decode_empty_count > expected_gap && g_decode_empty_count % 250 == 1) {
                SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                               "[Tick #%u] No data for %u raw ticks (%.0f ms idle, mode=%d, expected=%d)",
                               g_20ms_stats.total_ticks, g_decode_empty_count,
                               (float)(g_decode_empty_count * 20), g_frames_per_cmd, expected_gap);
            }

            /* Auto-stop: same actual time (200ms) regardless of mode */
            if (g_decode_empty_count >= EMPTY_COUNT_THRESHOLD) {
                if (xSemaphoreTake(g_decode_timer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (g_decode_empty_count >= EMPTY_COUNT_THRESHOLD && g_decode_timer_running) {
                        esp_err_t ret = esp_timer_stop(g_voice_20ms_timer);
                        if (ret == ESP_OK) {
                            g_decode_timer_running = false;
                            g_20ms_stats.timer_auto_stops++;
                            SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                                           "[AUTO] 20ms timer stopped (%u ticks, %.0f ms idle)",
                                           g_decode_empty_count, (float)(g_decode_empty_count * 20));
                        }
                    }
                    xSemaphoreGive(g_decode_timer_mutex);
                }
            }
        }

        /* Track queue depth */
        UBaseType_t current_depth = uxQueueMessagesWaiting(voice_raw_data_queue);
        if (current_depth > g_20ms_stats.queue_max_depth) {
            g_20ms_stats.queue_max_depth = current_depth;
        }
    }

    vTaskDelete(NULL);
}



/**
 * @brief Voice encode task (uplink: PCM -> AMRNB -> Base64 -> AT command)
 *
 * Runs on CPU0 for encoding operations, paced by 20ms timer
 */
static void voice_encode_task(void *pvParameters)
{
    (void)pvParameters;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice encode task started on CPU0 (20ms timer-paced)");

    /* Accumulator for multi-frame AT commands */
    int16_t pcm_accum[VOICE_MAX_FRAMES_PER_CMD][AUDIO_FRAME_SIZE];
    uint8_t amr_combined[VOICE_MAX_AMR_DATA_SIZE];
    char base64_buf[200];
    char at_cmd[200];
    int accumulated = 0;

    while (1) {
        /* Wait for 20ms timer tick */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Non-blocking receive from queue */
        if (xQueueReceive(voice_data_queue, pcm_accum[accumulated], 0) == pdPASS) {
            accumulated++;
            int target = g_frames_per_cmd;

            if (accumulated >= target) {
                /* Encode all accumulated frames into one AT command */
                int total_amr_len = 0;
                bool all_ok = true;

                for (int i = 0; i < accumulated; i++) {
                    /* AMRNB encode PCM data (160 samples for 20ms at 8kHz) */
                    int amrnb_len = audio_encode(&amr_combined[total_amr_len],
                                                  AUDIO_FRAME_SIZE / 2, pcm_accum[i]);

                    if (amrnb_len <= 0) {
                        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                            "AMRNB encode failed for frame %d/%d", i + 1, accumulated);
                        g_voice_stats.uplink_encode_errors++;
                        all_ok = false;
                        break;
                    }
                    total_amr_len += amrnb_len;
                }

                if (all_ok) {
                    /* Base64 encode combined AMR data */
                    int base64_len = base64_encode(amr_combined, total_amr_len,
                                                    base64_buf, sizeof(base64_buf));

                    if (base64_len > 0) {
                        /* Format as AT^AUDPCM command */
                        int at_len = snprintf(at_cmd, sizeof(at_cmd),
                                              "AT^AUDPCM=\"%s\"", base64_buf);

                        SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                            "Uplink: %d frames, %d bytes AMR -> AT cmd %d bytes",
                            accumulated, total_amr_len, at_len);

                        /* Send via output callback */
                        if (g_voice_output_callback != NULL) {
                            g_voice_output_callback((const uint8_t *)at_cmd, at_len,
                                                    g_voice_output_user_data);
                        } else {
                            SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                                "Output callback not registered, packet dropped");
                            g_voice_stats.uplink_packets_dropped++;
                        }
                        g_voice_stats.uplink_packets_total++;
                    } else {
                        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                            "Base64 encoding failed");
                        g_voice_stats.uplink_encode_errors++;
                    }
                }

                accumulated = 0;
            }
        } else {
            /* No data this tick - skip (normal when MUX has no data) */
        }

        /* 20ms pacing controlled by timer, no vTaskDelay needed */
    }

    vTaskDelete(NULL);
}



/* ===== Public API Implementation ===== */

int voice_packet_register_output_callback(voice_data_output_callback_t callback, void *user_data)
{
    if (callback == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Invalid callback (NULL)");
        return -1;
    }

    g_voice_output_callback = callback;
    g_voice_output_user_data = user_data;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Output callback registered");
    return 0;
}

int voice_packet_register_downlink_callback(voice_data_downlink_callback_t callback, void *user_data)
{
    if (callback == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Invalid callback (NULL)");
        return -1;
    }

    g_voice_downlink_callback = callback;
    g_voice_downlink_user_data = user_data;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Downlink callback registered");
    return 0;
}

TaskHandle_t voice_tasks_start(void)
{
    /* Check if already initialized */
    if (g_voice_state == VOICE_STATE_RUNNING) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice tasks already running");
        return g_encode_task_handle;
    }

    /* Create PCM data queue for uplink encoding */
    voice_data_queue = xQueueCreate(VOICE_QUEUE_SIZE, sizeof(int16_t) * AUDIO_FRAME_SIZE);
    if (voice_data_queue == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to create PCM queue");
        return NULL;
    }

    /* Create raw data queue for downlink decoding */
    voice_raw_data_queue = xQueueCreate(VOICE_QUEUE_SIZE, sizeof(voice_raw_data_t));
    if (voice_raw_data_queue == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to create raw data queue");
        vQueueDelete(voice_data_queue);
        voice_data_queue = NULL;
        return NULL;
    }

    /* Create mutex for decode timer control */
    g_decode_timer_mutex = xSemaphoreCreateMutex();
    if (g_decode_timer_mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to create decode timer mutex");
        vQueueDelete(voice_raw_data_queue);
        vQueueDelete(voice_data_queue);
        voice_raw_data_queue = NULL;
        voice_data_queue = NULL;
        return NULL;
    }

    /* Create mutex for encode timer control */
    g_encode_timer_mutex = xSemaphoreCreateMutex();
    if (g_encode_timer_mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to create encode timer mutex");
        vQueueDelete(voice_raw_data_queue);
        vQueueDelete(voice_data_queue);
        vSemaphoreDelete(g_decode_timer_mutex);
        voice_raw_data_queue = NULL;
        voice_data_queue = NULL;
        g_decode_timer_mutex = NULL;
        return NULL;
    }

    /* Create decode task on CPU1 */
    BaseType_t ret = xTaskCreatePinnedToCore(voice_decode_task, "voice_decode",
                                             VOICE_TASK_STACK_SIZE, NULL, VOICE_TASK_PRIORITY,
                                             &g_decode_task_handle, 1);
    if (ret != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to create decode task");
        vQueueDelete(voice_raw_data_queue);
        vQueueDelete(voice_data_queue);
        vSemaphoreDelete(g_decode_timer_mutex);
        voice_raw_data_queue = NULL;
        voice_data_queue = NULL;
        g_decode_timer_mutex = NULL;
        return NULL;
    }

    /* Create encode task on CPU0 */
    ret = xTaskCreatePinnedToCore(voice_encode_task, "voice_encode",
                                  VOICE_TASK_STACK_SIZE, NULL, VOICE_TASK_PRIORITY,
                                  &g_encode_task_handle, 0);
    if (ret != pdPASS) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Failed to create encode task");
        vTaskDelete(g_decode_task_handle);
        g_decode_task_handle = NULL;
        vQueueDelete(voice_raw_data_queue);
        vQueueDelete(voice_data_queue);
        vSemaphoreDelete(g_decode_timer_mutex);
        voice_raw_data_queue = NULL;
        voice_data_queue = NULL;
        g_decode_timer_mutex = NULL;
        return NULL;
    }

    /* ========== Create 20ms timer (but DON'T start it) ========== */
    esp_timer_create_args_t timer_args = {
        .callback = voice_20ms_timer_callback,
        .arg = g_decode_task_handle,  /* Pass task handle to callback */
        .name = "voice_20ms"
    };

    esp_err_t err = esp_timer_create(&timer_args, &g_voice_20ms_timer);
    if (err != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                       "Failed to create decode 20ms timer: %s", esp_err_to_name(err));
        /* Cleanup */
        vTaskDelete(g_encode_task_handle);
        vTaskDelete(g_decode_task_handle);
        vQueueDelete(voice_raw_data_queue);
        vQueueDelete(voice_data_queue);
        vSemaphoreDelete(g_decode_timer_mutex);
        vSemaphoreDelete(g_encode_timer_mutex);
        g_encode_task_handle = NULL;
        g_decode_task_handle = NULL;
        voice_raw_data_queue = NULL;
        voice_data_queue = NULL;
        g_decode_timer_mutex = NULL;
        g_encode_timer_mutex = NULL;
        return NULL;
    }

    /* ========== Create encode 20ms timer (but DON'T start it) ========== */
    esp_timer_create_args_t encode_timer_args = {
        .callback = voice_encode_20ms_timer_callback,
        .arg = g_encode_task_handle,  /* Pass task handle to callback */
        .name = "voice_encode_20ms"
    };

    err = esp_timer_create(&encode_timer_args, &g_encode_20ms_timer);
    if (err != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                       "Failed to create encode 20ms timer: %s", esp_err_to_name(err));
        /* Cleanup */
        esp_timer_delete(g_voice_20ms_timer);
        vTaskDelete(g_encode_task_handle);
        vTaskDelete(g_decode_task_handle);
        vQueueDelete(voice_raw_data_queue);
        vQueueDelete(voice_data_queue);
        vSemaphoreDelete(g_decode_timer_mutex);
        vSemaphoreDelete(g_encode_timer_mutex);
        g_voice_20ms_timer = NULL;
        g_encode_task_handle = NULL;
        g_decode_task_handle = NULL;
        voice_raw_data_queue = NULL;
        voice_data_queue = NULL;
        g_decode_timer_mutex = NULL;
        g_encode_timer_mutex = NULL;
        return NULL;
    }

    /* Initialize state and start timers */
    g_decode_timer_running = false;
    g_decode_empty_count = 0;
    g_encode_timer_running = false;
    g_encode_empty_count = 0;
    memset(&g_voice_stats, 0, sizeof(g_voice_stats));
    memset(&g_20ms_stats, 0, sizeof(g_20ms_stats));

    /* Start decode 20ms timer */
    err = esp_timer_start_periodic(g_voice_20ms_timer, 20000);  /* 20ms */
    if (err == ESP_OK) {
        g_decode_timer_running = true;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Decode 20ms timer started");
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                       "Failed to start decode timer: %s", esp_err_to_name(err));
    }

    /* Start encode 20ms timer */
    err = esp_timer_start_periodic(g_encode_20ms_timer, 20000);  /* 20ms */
    if (err == ESP_OK) {
        g_encode_timer_running = true;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Encode 20ms timer started");
    } else {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                       "Failed to start encode timer: %s", esp_err_to_name(err));
    }

    g_voice_state = VOICE_STATE_RUNNING;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                   "Voice tasks started (encode:CPU0, decode:CPU1, 20ms timers:running)");

    return g_encode_task_handle;
}

void voice_tasks_stop(void)
{
    if (g_voice_state != VOICE_STATE_RUNNING) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice tasks not running");
        return;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Stopping voice tasks...");

    /* ========== Force stop decode timer (fallback protection) ========== */
    if (g_voice_20ms_timer != NULL) {
        esp_timer_stop(g_voice_20ms_timer);
        esp_timer_delete(g_voice_20ms_timer);
        g_voice_20ms_timer = NULL;
        g_decode_timer_running = false;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Decode 20ms timer stopped (forced)");
    }

    /* ========== Force stop encode timer (fallback protection) ========== */
    if (g_encode_20ms_timer != NULL) {
        esp_timer_stop(g_encode_20ms_timer);
        esp_timer_delete(g_encode_20ms_timer);
        g_encode_20ms_timer = NULL;
        g_encode_timer_running = false;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Encode 20ms timer stopped (forced)");
    }

    /* Delete mutexes */
    if (g_decode_timer_mutex != NULL) {
        vSemaphoreDelete(g_decode_timer_mutex);
        g_decode_timer_mutex = NULL;
    }

    if (g_encode_timer_mutex != NULL) {
        vSemaphoreDelete(g_encode_timer_mutex);
        g_encode_timer_mutex = NULL;
    }

    /* Delete tasks */
    if (g_encode_task_handle != NULL) {
        vTaskDelete(g_encode_task_handle);
        g_encode_task_handle = NULL;
    }

    if (g_decode_task_handle != NULL) {
        vTaskDelete(g_decode_task_handle);
        g_decode_task_handle = NULL;
    }

    /* Delete queues */
    if (voice_data_queue != NULL) {
        vQueueDelete(voice_data_queue);
        voice_data_queue = NULL;
    }

    if (voice_raw_data_queue != NULL) {
        vQueueDelete(voice_raw_data_queue);
        voice_raw_data_queue = NULL;
    }

    /* Print statistics */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "========================================");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice Tasks Stopped - Statistics:");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "========================================");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "  20ms Timer Stats:");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Total ticks: %u (%.1f sec)",
                   g_20ms_stats.total_ticks, g_20ms_stats.total_ticks * 0.02f);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Data processed: %u (%.1f%%)",
                   g_20ms_stats.data_processed,
                   g_20ms_stats.total_ticks > 0 ?
                       (g_20ms_stats.data_processed * 100.0f / g_20ms_stats.total_ticks) : 0);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Data skipped: %u (%.1f%%)",
                   g_20ms_stats.data_skipped,
                   g_20ms_stats.total_ticks > 0 ?
                       (g_20ms_stats.data_skipped * 100.0f / g_20ms_stats.total_ticks) : 0);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Queue max depth: %u / %u",
                   g_20ms_stats.queue_max_depth, VOICE_QUEUE_SIZE);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Timer auto-starts: %u", g_20ms_stats.timer_auto_starts);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Timer auto-stops: %u", g_20ms_stats.timer_auto_stops);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "----------------------------------------");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "  Uplink Stats:");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Total: %u", g_voice_stats.uplink_packets_total);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Dropped: %u", g_voice_stats.uplink_packets_dropped);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Errors: %u", g_voice_stats.uplink_encode_errors);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "----------------------------------------");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "  Downlink Stats:");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Total: %u", g_voice_stats.downlink_packets_total);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Dropped: %u", g_voice_stats.downlink_packets_dropped);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "    Errors: %u", g_voice_stats.downlink_decode_errors);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "========================================");

    g_voice_state = VOICE_STATE_STOPPED;
}

int voice_downlink_enqueue(uint8_t *data, size_t data_len)
{
    /* Validate parameters */
    if (data == NULL || data_len == 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Invalid parameters");
        return -1;
    }

    /* Check if module is initialized */
    if (g_voice_state != VOICE_STATE_RUNNING || voice_raw_data_queue == NULL) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Voice tasks not running, packet dropped");
        return -1;
    }

    /* Check data length */
    if (data_len > MAX_RAW_DATA_SIZE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Data too long: %d > %d", data_len, MAX_RAW_DATA_SIZE);
        return -1;
    }

    /* Create packet */
    voice_raw_data_t raw_data = {0};
    memcpy(raw_data.data, data, data_len);
    raw_data.data_len = data_len;

    /* Send to queue (non-blocking) */
    if (xQueueSend(voice_raw_data_queue, &raw_data, 0) != pdPASS) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Queue full, packet dropped");
        g_voice_stats.downlink_packets_dropped++;
        return -1;
    }

    /* ===== Auto-start timer if not running ===== */
    if (!g_decode_timer_running) {
        if (xSemaphoreTake(g_decode_timer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (!g_decode_timer_running) {  /* Double-check */
                esp_err_t ret = esp_timer_start_periodic(g_voice_20ms_timer, 20000);  /* 20ms */
                if (ret == ESP_OK) {
                    g_decode_timer_running = true;
                    g_decode_empty_count = 0;  /* Reset empty counter */
                    g_20ms_stats.timer_auto_starts++;
                    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                                   "[AUTO] 20ms timer started (data arrived, start #%u)",
                                   g_20ms_stats.timer_auto_starts);
                } else {
                    SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
                                   "Failed to auto-start timer: %s", esp_err_to_name(ret));
                }
            }
            xSemaphoreGive(g_decode_timer_mutex);
        }
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG, "Downlink packet enqueued: %d bytes", data_len);
    return 0;
}

int voice_packet_set_frame_mode(uint8_t frames)
{
    if (frames != 1 && frames != 3) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
            "Invalid frame mode: %d (must be 1 or 3)", frames);
        return -1;
    }

    g_frames_per_cmd = frames;
    g_frame_mode_manual = true;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_VOICE_PACKET, TAG,
        "Uplink frame mode set to %d frames/AT (manual, auto-detect disabled)", frames);
    return 0;
}

uint8_t voice_packet_get_frame_mode(void)
{
    return g_frames_per_cmd;
}
