#include "audio/audiosvc.h"
#include "amrnb_encoder.h"
#include "amr_decoder.h"
#include "audio_pipeline.h"
#include "raw_stream.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "system/syslog.h"

#define TAG                     "AUDIO"
#define RING_BUFFER_SIZE         2048
#define AMRNB_HEADER_SIZE        6
#define AMRNB_HEADER            "#!AMR\n"
#define AMRNB_MAX_FRAME_SIZE     32
#define PCM_FRAME_SIZE          160
#define PCM_FRAME_BYTES         (PCM_FRAME_SIZE * sizeof(int16_t))
#define MUTEX_TIMEOUT_MS        1000

/* Codec state */
typedef enum {
    AMR_CODEC_STATE_UNINITIALIZED = 0,
    AMR_CODEC_STATE_INITIALIZED,
    AMR_CODEC_STATE_ERROR
} amr_codec_state_t;

typedef struct {
    audio_pipeline_handle_t enc_pipeline;
    audio_pipeline_handle_t dec_pipeline;

    audio_element_handle_t enc_raw_wr;
    audio_element_handle_t enc_amr;
    audio_element_handle_t enc_raw_rd;

    audio_element_handle_t dec_raw_wr;
    audio_element_handle_t dec_amr;
    audio_element_handle_t dec_raw_rd;

    bool first_decode;               // Track first decode (not static for thread safety)
} amr_nb_codec_t;

static amr_nb_codec_t g_amr_codec = {0};
static amr_codec_state_t g_amr_codec_state = AMR_CODEC_STATE_UNINITIALIZED;
static SemaphoreHandle_t g_amr_codec_mutex = NULL;

/* Statistics */
typedef struct {
    uint32_t encode_calls;
    uint32_t encode_errors;
    uint32_t decode_calls;
    uint32_t decode_errors;
    uint32_t mutex_timeouts;
} audio_stats_t;

static audio_stats_t g_audio_stats = {0};

/* Forward declarations */
static esp_err_t create_encode_pipeline(amr_nb_codec_t *codec);
static esp_err_t create_decode_pipeline(amr_nb_codec_t *codec);
static void cleanup_encode_pipeline(amr_nb_codec_t *codec);
static void cleanup_decode_pipeline(amr_nb_codec_t *codec);

/**
 * @brief Audio element event handler
 */
static esp_err_t audio_element_event_handler(audio_element_handle_t self,
                                              audio_event_iface_msg_t *event,
                                              void *ctx)
{
    (void)ctx;

    if (self == NULL || event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (event->cmd == AEL_MSG_CMD_REPORT_STATUS) {
        switch ((int)event->data) {
            case AEL_STATUS_STATE_RUNNING:
                SYS_LOGD_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Element %s: RUNNING",
                               audio_element_get_tag(self));
                break;
            case AEL_STATUS_STATE_STOPPED:
                SYS_LOGD_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Element %s: STOPPED",
                               audio_element_get_tag(self));
                break;
            case AEL_STATUS_STATE_FINISHED:
                SYS_LOGD_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Element %s: FINISHED",
                               audio_element_get_tag(self));
                break;
            case AEL_STATUS_ERROR_OPEN:
                SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Element %s: OPEN ERROR",
                               audio_element_get_tag(self));
                break;
            default:
                SYS_LOGD_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Element %s: event %d",
                               audio_element_get_tag(self), (int)event->data);
                break;
        }
    }
    return ESP_OK;
}

/**
 * @brief Create encode pipeline (PCM -> AMRNB)
 */
static esp_err_t create_encode_pipeline(amr_nb_codec_t *codec)
{
    if (codec == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();

    /* Initialize pipeline */
    codec->enc_pipeline = audio_pipeline_init(&pipeline_cfg);
    if (codec->enc_pipeline == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to init encode pipeline");
        return ESP_FAIL;
    }

    /* Create raw stream writer (PCM input) */
    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_WRITER | AUDIO_STREAM_READER;
    raw_cfg.out_rb_size = RING_BUFFER_SIZE;

    codec->enc_raw_wr = raw_stream_init(&raw_cfg);
    if (codec->enc_raw_wr == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to init encode raw writer");
        audio_pipeline_deinit(codec->enc_pipeline);
        codec->enc_pipeline = NULL;
        return ESP_FAIL;
    }

    /* Create AMRNB encoder */
    amrnb_encoder_cfg_t enc_cfg = {
        .out_rb_size = RING_BUFFER_SIZE,
        .bitrate_mode = AMRNB_ENC_BITRATE_MR122,
        .contain_amrnb_header = false,
    };

    codec->enc_amr = amrnb_encoder_init(&enc_cfg);
    if (codec->enc_amr == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to init AMRNB encoder");
        cleanup_encode_pipeline(codec);
        return ESP_FAIL;
    }

    audio_element_set_event_callback(codec->enc_amr, audio_element_event_handler, NULL);

    /* Create raw stream reader (AMR output) */
    raw_cfg.type = AUDIO_STREAM_READER | AUDIO_STREAM_WRITER;
    codec->enc_raw_rd = raw_stream_init(&raw_cfg);
    if (codec->enc_raw_rd == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to init encode raw reader");
        cleanup_encode_pipeline(codec);
        return ESP_FAIL;
    }

    /* Register and link pipeline */
    audio_pipeline_register(codec->enc_pipeline, codec->enc_raw_wr, "pcm");
    audio_pipeline_register(codec->enc_pipeline, codec->enc_amr, "amr");
    audio_pipeline_register(codec->enc_pipeline, codec->enc_raw_rd, "out");

    esp_err_t ret = audio_pipeline_link(codec->enc_pipeline,
                                        (const char *[]) { "pcm", "amr", "out" }, 3);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to link encode pipeline: %d", ret);
        cleanup_encode_pipeline(codec);
        return ret;
    }

    ret = audio_pipeline_run(codec->enc_pipeline);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to run encode pipeline: %d", ret);
        cleanup_encode_pipeline(codec);
        return ret;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Encode pipeline created successfully");
    return ESP_OK;
}

/**
 * @brief Create decode pipeline (AMRNB -> PCM)
 */
static esp_err_t create_decode_pipeline(amr_nb_codec_t *codec)
{
    if (codec == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();

    /* Initialize pipeline */
    codec->dec_pipeline = audio_pipeline_init(&pipeline_cfg);
    if (codec->dec_pipeline == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to init decode pipeline");
        return ESP_FAIL;
    }

    /* Create raw stream writer (AMR input) */
    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_WRITER | AUDIO_STREAM_READER;

    codec->dec_raw_wr = raw_stream_init(&raw_cfg);
    if (codec->dec_raw_wr == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to init decode raw writer");
        audio_pipeline_deinit(codec->dec_pipeline);
        codec->dec_pipeline = NULL;
        return ESP_FAIL;
    }

    /* Create AMRNB decoder */
    amr_decoder_cfg_t dec_cfg = {
        .out_rb_size = RING_BUFFER_SIZE,
    };

    codec->dec_amr = amr_decoder_init(&dec_cfg);
    if (codec->dec_amr == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to init AMR decoder");
        cleanup_decode_pipeline(codec);
        return ESP_FAIL;
    }

    audio_element_set_event_callback(codec->dec_amr, audio_element_event_handler, NULL);

    /* Create raw stream reader (PCM output) */
    codec->dec_raw_rd = raw_stream_init(&raw_cfg);
    if (codec->dec_raw_rd == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to init decode raw reader");
        cleanup_decode_pipeline(codec);
        return ESP_FAIL;
    }

    /* Register and link pipeline */
    audio_pipeline_register(codec->dec_pipeline, codec->dec_raw_wr, "amr");
    audio_pipeline_register(codec->dec_pipeline, codec->dec_amr, "pcm");
    audio_pipeline_register(codec->dec_pipeline, codec->dec_raw_rd, "out");

    esp_err_t ret = audio_pipeline_link(codec->dec_pipeline,
                                        (const char *[]) { "amr", "pcm", "out" }, 3);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to link decode pipeline: %d", ret);
        cleanup_decode_pipeline(codec);
        return ret;
    }

    ret = audio_pipeline_run(codec->dec_pipeline);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to run decode pipeline: %d", ret);
        cleanup_decode_pipeline(codec);
        return ret;
    }

    /* Initialize first decode flag */
    codec->first_decode = true;

    SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Decode pipeline created successfully");
    return ESP_OK;
}

/**
 * @brief Clean up encode pipeline resources
 *
 * Correct cleanup order: stop -> wait_for_stop -> unlink -> unregister -> deinit elements -> deinit pipeline
 */
static void cleanup_encode_pipeline(amr_nb_codec_t *codec)
{
    if (codec == NULL) {
        return;
    }

    if (codec->enc_pipeline != NULL) {
        // Step 1: Stop pipeline
        audio_pipeline_stop(codec->enc_pipeline);
        audio_pipeline_wait_for_stop(codec->enc_pipeline);

        // Step 2: Unlink pipeline (must be done before unregister)
        audio_pipeline_unlink(codec->enc_pipeline);

        // Step 3: Unregister elements from pipeline
        if (codec->enc_raw_wr != NULL) {
            audio_pipeline_unregister(codec->enc_pipeline, codec->enc_raw_wr);
        }
        if (codec->enc_amr != NULL) {
            audio_pipeline_unregister(codec->enc_pipeline, codec->enc_amr);
        }
        if (codec->enc_raw_rd != NULL) {
            audio_pipeline_unregister(codec->enc_pipeline, codec->enc_raw_rd);
        }

        // Step 4: Deinitialize pipeline
        audio_pipeline_deinit(codec->enc_pipeline);
        codec->enc_pipeline = NULL;
    }

    // Step 5: Deinitialize elements (after pipeline cleanup)
    if (codec->enc_raw_wr != NULL) {
        audio_element_deinit(codec->enc_raw_wr);
        codec->enc_raw_wr = NULL;
    }

    if (codec->enc_amr != NULL) {
        audio_element_deinit(codec->enc_amr);
        codec->enc_amr = NULL;
    }

    if (codec->enc_raw_rd != NULL) {
        audio_element_deinit(codec->enc_raw_rd);
        codec->enc_raw_rd = NULL;
    }
}

/**
 * @brief Clean up decode pipeline resources
 *
 * Correct cleanup order: stop -> wait_for_stop -> unlink -> unregister -> deinit elements -> deinit pipeline
 */
static void cleanup_decode_pipeline(amr_nb_codec_t *codec)
{
    if (codec == NULL) {
        return;
    }

    if (codec->dec_pipeline != NULL) {
        // Step 1: Stop pipeline
        audio_pipeline_stop(codec->dec_pipeline);
        audio_pipeline_wait_for_stop(codec->dec_pipeline);

        // Step 2: Unlink pipeline (must be done before unregister)
        audio_pipeline_unlink(codec->dec_pipeline);

        // Step 3: Unregister elements from pipeline
        if (codec->dec_raw_wr != NULL) {
            audio_pipeline_unregister(codec->dec_pipeline, codec->dec_raw_wr);
        }
        if (codec->dec_amr != NULL) {
            audio_pipeline_unregister(codec->dec_pipeline, codec->dec_amr);
        }
        if (codec->dec_raw_rd != NULL) {
            audio_pipeline_unregister(codec->dec_pipeline, codec->dec_raw_rd);
        }

        // Step 4: Deinitialize pipeline
        audio_pipeline_deinit(codec->dec_pipeline);
        codec->dec_pipeline = NULL;
    }

    // Step 5: Deinitialize elements (after pipeline cleanup)
    if (codec->dec_raw_wr != NULL) {
        audio_element_deinit(codec->dec_raw_wr);
        codec->dec_raw_wr = NULL;
    }

    if (codec->dec_amr != NULL) {
        audio_element_deinit(codec->dec_amr);
        codec->dec_amr = NULL;
    }

    if (codec->dec_raw_rd != NULL) {
        audio_element_deinit(codec->dec_raw_rd);
        codec->dec_raw_rd = NULL;
    }
}

/**
 * @brief Initialize audio service
 */
int audio_svc_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Check if already initialized */
    if (g_amr_codec_state == AMR_CODEC_STATE_INITIALIZED) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Audio service already initialized");
        return 0;
    }

    /* Clear codec structure */
    memset(&g_amr_codec, 0, sizeof(g_amr_codec));

    /* Create mutex first (for thread safety during init) */
    g_amr_codec_mutex = xSemaphoreCreateMutex();
    if (g_amr_codec_mutex == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to create codec mutex");
        g_amr_codec_state = AMR_CODEC_STATE_ERROR;
        return -1;
    }

    /* Create encode pipeline */
    esp_err_t ret = create_encode_pipeline(&g_amr_codec);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to create encode pipeline: %d", ret);
        vSemaphoreDelete(g_amr_codec_mutex);
        g_amr_codec_mutex = NULL;
        g_amr_codec_state = AMR_CODEC_STATE_ERROR;
        return -1;
    }

    /* Create decode pipeline */
    ret = create_decode_pipeline(&g_amr_codec);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to create decode pipeline: %d", ret);
        cleanup_encode_pipeline(&g_amr_codec);
        vSemaphoreDelete(g_amr_codec_mutex);
        g_amr_codec_mutex = NULL;
        g_amr_codec_state = AMR_CODEC_STATE_ERROR;
        return -1;
    }

    /* Clear statistics */
    memset(&g_audio_stats, 0, sizeof(g_audio_stats));

    g_amr_codec_state = AMR_CODEC_STATE_INITIALIZED;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Audio service initialized successfully");

    return 0;
}

/**
 * @brief Destroy audio service
 */
void audio_destroy(void)
{
    if (g_amr_codec_state != AMR_CODEC_STATE_INITIALIZED) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Audio service not initialized");
        return;
    }

    SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Destroying audio service...");

    /* Print statistics */
    SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Statistics:");
    SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "  Encode: calls=%u, errors=%u",
                   g_audio_stats.encode_calls, g_audio_stats.encode_errors);
    SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "  Decode: calls=%u, errors=%u, timeouts=%u",
                   g_audio_stats.decode_calls, g_audio_stats.decode_errors,
                   g_audio_stats.mutex_timeouts);

    /* Clean up pipelines */
    cleanup_encode_pipeline(&g_amr_codec);
    cleanup_decode_pipeline(&g_amr_codec);

    /* Destroy mutex */
    if (g_amr_codec_mutex != NULL) {
        vSemaphoreDelete(g_amr_codec_mutex);
        g_amr_codec_mutex = NULL;
    }

    /* Clear codec structure */
    memset(&g_amr_codec, 0, sizeof(g_amr_codec));

    g_amr_codec_state = AMR_CODEC_STATE_UNINITIALIZED;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Audio service destroyed");
}

/**
 * @brief Encode PCM frame to AMRNB
 */
static int amr_nb_encode_frame(amr_nb_codec_t *codec,
                                const int16_t *pcm20ms,
                                uint8_t *out_amr,
                                size_t out_size)
{
    /* Validate parameters */
    if (codec == NULL || pcm20ms == NULL || out_amr == NULL) {
        return -1;
    }

    if (codec->enc_raw_wr == NULL || codec->enc_raw_rd == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Encode pipeline not ready");
        return -1;
    }

    if (out_size < AMRNB_MAX_FRAME_SIZE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Output buffer too small: %d < %d",
                       out_size, AMRNB_MAX_FRAME_SIZE);
        return -1;
    }

    /* Write PCM data to encoder */
    int wr = raw_stream_write(codec->enc_raw_wr, (char *)pcm20ms, PCM_FRAME_BYTES);
    if (wr <= 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to write PCM data: %d", wr);
        return -2;
    }

    /* Read encoded AMRNB data */
    int rd = raw_stream_read(codec->enc_raw_rd, (char *)out_amr, AMRNB_MAX_FRAME_SIZE);
    if (rd <= 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to read AMR data: %d", rd);
        return -2;
    }

    return rd;  // MR122 ≈ 32 bytes
}

/**
 * @brief Decode AMRNB frame to PCM
 */
static int amr_nb_decode_frame(amr_nb_codec_t *codec,
                                const uint8_t *in_amr,
                                size_t in_len,
                                int16_t *out_pcm,
                                size_t pcm_max)
{
    /* Validate parameters */
    if (codec == NULL || in_amr == NULL || out_pcm == NULL) {
        return -1;
    }

    if (codec->dec_raw_wr == NULL || codec->dec_raw_rd == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Decode pipeline not ready");
        return -1;
    }

    if (pcm_max < PCM_FRAME_BYTES) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "PCM buffer too small: %d < %d",
                       pcm_max, PCM_FRAME_BYTES);
        return -1;
    }

    if (in_len > AMRNB_MAX_FRAME_SIZE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "AMR data too large: %d > %d",
                       in_len, AMRNB_MAX_FRAME_SIZE);
        return -1;
    }

    /* First decode: add AMRNB header */
    if (codec->first_decode) {
        uint8_t amr_with_header[AMRNB_HEADER_SIZE + AMRNB_MAX_FRAME_SIZE];
        memcpy(amr_with_header, AMRNB_HEADER, AMRNB_HEADER_SIZE);
        memcpy(amr_with_header + AMRNB_HEADER_SIZE, in_amr, in_len);

        int wr = raw_stream_write(codec->dec_raw_wr, (char *)amr_with_header,
                                  in_len + AMRNB_HEADER_SIZE);
        if (wr <= 0) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to write first AMR frame: %d", wr);
            return -2;
        }

        codec->first_decode = false;
        SYS_LOGD_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "First decode with AMR header");
    } else {
        /* Subsequent decodes: write AMR data directly */
        int wr = raw_stream_write(codec->dec_raw_wr, (char *)in_amr, in_len);
        if (wr <= 0) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to write AMR data: %d", wr);
            return -2;
        }
    }

    /* Read decoded PCM data */
    int rd = raw_stream_read(codec->dec_raw_rd, (char *)out_pcm, PCM_FRAME_BYTES);
    if (rd <= 0) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Failed to read PCM data: %d", rd);
        return -2;
    }

    return rd;  // Usually = 160 * 2 = 320 bytes
}

/**
 * @brief Public API: Encode PCM to AMRNB
 */
int audio_encode(unsigned char *bits, int len, short speech[])
{
    /* Validate state */
    if (g_amr_codec_state != AMR_CODEC_STATE_INITIALIZED) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Audio service not initialized");
        return -1;
    }

    /* Validate parameters */
    if (bits == NULL || speech == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Invalid parameters (NULL)");
        return -1;
    }

    if (len < AMRNB_MAX_FRAME_SIZE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Output buffer too small: %d < %d",
                       len, AMRNB_MAX_FRAME_SIZE);
        return -1;
    }

    /* Take mutex with timeout */
    if (xSemaphoreTake(g_amr_codec_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Mutex timeout (encode)");
        g_audio_stats.mutex_timeouts++;
        return -2;
    }

    g_audio_stats.encode_calls++;

    int ret = amr_nb_encode_frame(&g_amr_codec, speech, bits, len);

    if (ret < 0) {
        g_audio_stats.encode_errors++;
    }

    xSemaphoreGive(g_amr_codec_mutex);
    return ret;
}

/**
 * @brief Public API: Decode AMRNB to PCM
 */
int audio_decode(unsigned char *bits, int len, short speech[])
{
    /* Validate state */
    if (g_amr_codec_state != AMR_CODEC_STATE_INITIALIZED) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Audio service not initialized");
        return -1;
    }

    /* Validate parameters */
    if (bits == NULL || speech == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Invalid parameters (NULL)");
        return -1;
    }

    if (len <= 0 || len > AMRNB_MAX_FRAME_SIZE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Invalid AMR data length: %d", len);
        return -1;
    }

    /* Take mutex with timeout */
    if (xSemaphoreTake(g_amr_codec_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Mutex timeout (decode)");
        g_audio_stats.mutex_timeouts++;
        return -2;
    }

    g_audio_stats.decode_calls++;

    int ret = amr_nb_decode_frame(&g_amr_codec, bits, len, speech, AUDIO_FRAME_SIZE);

    if (ret < 0) {
        g_audio_stats.decode_errors++;
    }

    xSemaphoreGive(g_amr_codec_mutex);
    return ret;
}

/**
 * @brief Reset decoder state (call when starting a new voice session)
 *
 * This function resets the first_decode flag, which is needed when
 * starting a new AMR decoding session. The first frame requires
 * the AMR header "#!AMR\n" to be prepended.
 */
void audio_decode_reset(void)
{
    if (g_amr_codec_state != AMR_CODEC_STATE_INITIALIZED) {
        return;
    }

    if (xSemaphoreTake(g_amr_codec_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        g_amr_codec.first_decode = true;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_AUDIO_PROC, TAG, "Decoder state reset (first_decode=true)");
        xSemaphoreGive(g_amr_codec_mutex);
    }
}
