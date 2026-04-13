#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

#define TAG "PROFILE"
#define PROFILE_MAX_SAMPLES 64

typedef struct {
    char         tag[64];
    uint32_t     delta_us;
} profile_sample_t;

static profile_sample_t samples[PROFILE_MAX_SAMPLES];
static int sample_count = 0;

void machdep_profile_init(void)
{
    sample_count = 0;
    ESP_LOGI(TAG, "Profiler init (esp_timer_get_time)");
}

void machdep_profile_reset(void)
{
    sample_count = 0;
}

unsigned int machdep_profile_sample(void)
{
    /* esp_timer_get_time() 返回 int64_t us */
    return (unsigned int)esp_timer_get_time();
}

unsigned int machdep_profile_sample_and_log(unsigned int start, char s[])
{
    uint32_t end = (uint32_t)esp_timer_get_time();
    uint32_t delta = end - start;

    if (sample_count < PROFILE_MAX_SAMPLES) {
        strncpy(samples[sample_count].tag, s,
                sizeof(samples[sample_count].tag) - 1);
        samples[sample_count].tag[sizeof(samples[sample_count].tag) - 1] = '\0';
        samples[sample_count].delta_us = delta;
        sample_count++;
    }

    return end;
}

void machdep_profile_print_logged_samples(void)
{
    ESP_LOGI(TAG, "---- Profiling result ----");

    for (int i = 0; i < sample_count; i++) {
        ESP_LOGI(TAG, "%2d: %-20s %6u us",
                 i,
                 samples[i].tag,
                 samples[i].delta_us);
    }
}
