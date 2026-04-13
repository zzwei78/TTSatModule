#ifndef __AUDIOSVC_H__
#define __AUDIOSVC_H__

/* Audio frame size: 20ms @ 8kHz, 16-bit = 320 bytes */
#define AUDIO_FRAME_SIZE        320

/**
 * @brief Initialize audio codec service
 * @return 0 on success, -1 on error
 */
int audio_svc_init(void);

/**
 * @brief Destroy audio codec service
 */
void audio_destroy(void);

/**
 * @brief Encode PCM audio to AMRNB
 * @param bits Output buffer for AMRNB data (min 32 bytes)
 * @param len Size of output buffer
 * @param speech Input PCM audio data (160 samples @ 8kHz, 16-bit)
 * @return Number of bytes encoded, or negative on error
 */
int audio_encode(unsigned char *bits, int len, short speech[]);

/**
 * @brief Decode AMRNB audio to PCM
 * @param bits Input AMRNB data
 * @param len Length of AMRNB data (max 32 bytes)
 * @param speech Output buffer for PCM data (min 320 bytes)
 * @return Number of bytes decoded, or negative on error
 */
int audio_decode(unsigned char *bits, int len, short speech[]);

/**
 * @brief Reset decoder state for new voice session
 *
 * Call this function when starting a new voice decoding session.
 * It resets the first_decode flag so the next decode call will
 * prepend the required AMR header "#!AMR\n".
 */
void audio_decode_reset(void);

#endif // __AUDIOSVC_H__