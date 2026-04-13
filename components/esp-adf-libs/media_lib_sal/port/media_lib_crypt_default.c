/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * See LICENSE file for details.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_crc.h"
#include "psa/crypto.h"
#include "media_lib_crypt_reg.h"
#include "media_lib_adapter.h"
#include "media_lib_os.h"
#include "esp_idf_version.h"
#include "aes/esp_aes.h"
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
// Check if compat header exists
#if __has_include("mbedtls/compat-2.x.h")
#include "mbedtls/compat-2.x.h"
#endif
#endif
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0))
#include "aes/esp_aes.h"
#elif (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/aes.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/aes.h"
#endif
#else
#include "hwcrypto/aes.h"
#endif

#ifdef CONFIG_MEDIA_LIB_CRYPT_ENABLE

#define RETURN_ON_NULL_HANDLE(h)                                               \
    if (h == NULL)   {                                                         \
        return ESP_ERR_INVALID_ARG;                                            \
    }

// CRC32 state for MD5 compatibility
typedef struct {
    uint32_t crc;
} crc32_context_t;

static void _md5_init(media_lib_md5_handle_t *ctx)
{
    crc32_context_t *crc_ctx = (crc32_context_t *)media_lib_malloc(sizeof(crc32_context_t));
    if (crc_ctx) {
        crc_ctx->crc = 0xffffffff;
        *ctx = crc_ctx;
    }
}

static void _md5_free(media_lib_md5_handle_t ctx)
{
    if (ctx) {
        media_lib_free(ctx);
    }
}

static int _md5_start(media_lib_md5_handle_t ctx)
{
    RETURN_ON_NULL_HANDLE(ctx);
    crc32_context_t *crc_ctx = (crc32_context_t *)ctx;
    crc_ctx->crc = 0xffffffff;
    return 0;
}

static int _md5_update(media_lib_md5_handle_t ctx, const unsigned char *input, size_t len)
{
    RETURN_ON_NULL_HANDLE(ctx);
    crc32_context_t *crc_ctx = (crc32_context_t *)ctx;
    crc_ctx->crc = esp_crc32_le(crc_ctx->crc, input, len);
    return 0;
}

static int _md5_finish(media_lib_md5_handle_t ctx, unsigned char output[16])
{
    RETURN_ON_NULL_HANDLE(ctx);
    crc32_context_t *crc_ctx = (crc32_context_t *)ctx;
    uint32_t final_crc = ~crc_ctx->crc;

    // Copy CRC32 (4 bytes) and pad with zeros to match MD5 output size (16 bytes)
    memset(output, 0, 16);
    memcpy(output, &final_crc, sizeof(uint32_t));

    return 0;
}

// PSA SHA256 context
typedef struct {
    psa_hash_operation_t operation;
} psa_sha256_context_t;

static void _sha256_init(media_lib_sha256_handle_t *ctx)
{
    psa_sha256_context_t *sha256 = (psa_sha256_context_t *)media_lib_malloc(sizeof(psa_sha256_context_t));
    if (sha256) {
        sha256->operation = psa_hash_operation_init();
        *ctx = sha256;
    }
}

static void _sha256_free(media_lib_sha256_handle_t ctx)
{
    if (ctx) {
        psa_hash_operation_t *operation = &((psa_sha256_context_t *)ctx)->operation;
        psa_hash_abort(operation);
        media_lib_free(ctx);
    }
}

static int _sha256_start(media_lib_sha256_handle_t ctx)
{
    RETURN_ON_NULL_HANDLE(ctx);
    psa_hash_operation_t *operation = &((psa_sha256_context_t *)ctx)->operation;
    psa_status_t status = psa_hash_setup(operation, PSA_ALG_SHA_256);
    return (status == PSA_SUCCESS) ? 0 : -1;
}

static int _sha256_update(media_lib_sha256_handle_t ctx, const unsigned char *input, size_t len)
{
    RETURN_ON_NULL_HANDLE(ctx);
    psa_hash_operation_t *operation = &((psa_sha256_context_t *)ctx)->operation;
    psa_status_t status = psa_hash_update(operation, input, len);
    return (status == PSA_SUCCESS) ? 0 : -1;
}

static int _sha256_finish(media_lib_sha256_handle_t ctx, unsigned char output[32])
{
    RETURN_ON_NULL_HANDLE(ctx);
    psa_hash_operation_t *operation = &((psa_sha256_context_t *)ctx)->operation;
    size_t hash_len = 32;
    psa_status_t status = psa_hash_finish(operation, output, 32, &hash_len);
    return (status == PSA_SUCCESS) ? 0 : -1;
}

static void _aes_init(media_lib_aes_handle_t *ctx)
{
    esp_aes_context *aes = (esp_aes_context *)media_lib_malloc(sizeof(esp_aes_context));
    if (aes) {
        esp_aes_init(aes);
        *ctx = aes;
    }
}

static void _aes_free(media_lib_aes_handle_t ctx)
{
    if (ctx) {
        esp_aes_free((esp_aes_context *)ctx);
        media_lib_free(ctx);
    }
}

static int _aes_set_key(media_lib_aes_handle_t ctx, uint8_t *key, uint8_t key_bits)
{
    RETURN_ON_NULL_HANDLE(ctx);
    int ret = esp_aes_setkey((esp_aes_context *)ctx, key, key_bits);
    return ret;
}

static int _aes_crypt_cbc(media_lib_aes_handle_t ctx, bool decrypt_mode, uint8_t iv[16], uint8_t *input,
                          size_t size, uint8_t *output)
{
    RETURN_ON_NULL_HANDLE(ctx);
    int ret = esp_aes_crypt_cbc(
            (esp_aes_context *)ctx,
            decrypt_mode ? ESP_AES_DECRYPT : ESP_AES_ENCRYPT,
            size, iv, input, output);
    return ret;
}

esp_err_t media_lib_add_default_crypt_adapter(void)
{
    media_lib_crypt_t crypt_lib = {
        .md5_init = _md5_init,
        .md5_free = _md5_free,
        .md5_start = _md5_start,
        .md5_update = _md5_update,
        .md5_finish = _md5_finish,
        .sha256_init = _sha256_init,
        .sha256_free = _sha256_free,
        .sha256_start = _sha256_start,
        .sha256_update = _sha256_update,
        .sha256_finish = _sha256_finish,
        .aes_init = _aes_init,
        .aes_free = _aes_free,
        .aes_set_key = _aes_set_key,
        .aes_crypt_cbc = _aes_crypt_cbc,
    };
    return media_lib_crypt_register(&crypt_lib);
}
#endif
