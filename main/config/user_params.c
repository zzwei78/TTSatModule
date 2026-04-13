/*
 * user_params.c - User Parameters Storage Implementation
 *
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "user_params.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include "system/syslog.h"

static const char *TAG = "user_params";

/* 单例实例 */
static user_params_t s_user_params = {0};
static bool s_initialized = false;

/* ========== CRC32 Implementation ========== */

/**
 * @brief CRC32计算（Ethernet/PNG多项式：0xEDB88320）
 */
static uint32_t crc32_calculate(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFF;
}

/**
 * @brief 计算user_params_t结构的CRC32
 *
 * CRC计算范围：整个结构除了crc32字段本身
 */
static uint32_t user_params_calculate_crc32(const user_params_t *params)
{
    if (params == NULL) {
        return 0;
    }

    /* 计算从结构开头到crc32字段之前的CRC */
    size_t offset = offsetof(user_params_t, crc32);
    uint32_t crc_part1 = crc32_calculate((const uint8_t *)params, offset);

    /* 计算从crc32字段之后到结构结尾的CRC */
    size_t remaining = sizeof(user_params_t) - offset - 4;
    uint32_t crc_part2 = crc32_calculate(
        (const uint8_t *)params + offset + 4,
        remaining
    );

    /* 组合两个部分的CRC */
    /* 简单方法：对part2的起始值使用part1作为初始值 */
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *data = (const uint8_t *)params;

    /* 计算整个结构的CRC，跳过crc32字段 */
    for (size_t i = 0; i < sizeof(user_params_t); i++) {
        /* 跳过crc32字段（offset 8-11） */
        if (i >= offset && i < offset + 4) {
            continue;
        }
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFF;
}

/* ========== Public API ========== */

esp_err_t user_params_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    /* 尝试从NVS加载 */
    esp_err_t ret = user_params_load(&s_user_params);

    if (ret == ESP_ERR_NOT_FOUND) {
        /* 参数不存在，使用默认值初始化 */
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "User params not found in NVS, creating with defaults");
        user_params_init_default(&s_user_params);
        ret = user_params_save(&s_user_params);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to save default params: %s", esp_err_to_name(ret));
            return ret;
        }
    } else if (ret == ESP_FAIL) {
        /* 参数损坏，使用默认值并覆盖 */
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "User params corrupted, resetting to defaults");
        user_params_init_default(&s_user_params);
        ret = user_params_save(&s_user_params);
        if (ret != ESP_OK) {
            SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to save default params: %s", esp_err_to_name(ret));
            return ret;
        }
    } else if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to load params: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "User params initialized (flags: 0x%02X, low_batt_thresh: %umV)",
                    s_user_params.flags, s_user_params.low_battery_threshold_mv);
    return ESP_OK;
}

esp_err_t user_params_deinit(void)
{
    s_initialized = false;
    memset(&s_user_params, 0, sizeof(user_params_t));
    return ESP_OK;
}

void user_params_init_default(user_params_t *params)
{
    if (params == NULL) {
        return;
    }

    memset(params, 0, sizeof(user_params_t));

    /* 头部 */
    params->magic = USER_PARAMS_MAGIC;
    params->version = USER_PARAMS_VERSION;
    params->size = sizeof(user_params_t);
    params->crc32 = 0;  /* 保存时计算 */

    /* 基本参数 */
    params->flags = 0;  /* 所有标志位清零 */
    params->low_battery_threshold_mv = USER_PARAMS_DEFAULT_LOW_BATT_MV;
}

esp_err_t user_params_save(const user_params_t *params)
{
    if (params == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "NULL params pointer");
        return ESP_ERR_INVALID_ARG;
    }

    /* 验证结构大小 */
    if (params->size != sizeof(user_params_t)) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Invalid params size: %u (expected %u)",
                        params->size, sizeof(user_params_t));
        return ESP_ERR_INVALID_ARG;
    }

    /* 复制参数并计算CRC */
    user_params_t params_copy = *params;
    params_copy.crc32 = user_params_calculate_crc32(&params_copy);

    /* 打开NVS */
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(USER_PARAMS_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 保存数据 */
    ret = nvs_set_blob(handle, USER_PARAMS_NVS_KEY_DATA, &params_copy, sizeof(user_params_t));
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to set blob: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    /* 提交 */
    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }

    nvs_close(handle);
    return ret;
}

esp_err_t user_params_load(user_params_t *params)
{
    if (params == NULL) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "NULL params pointer");
        return ESP_ERR_INVALID_ARG;
    }

    /* 打开NVS */
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(USER_PARAMS_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "NVS namespace not found");
        return ESP_ERR_NOT_FOUND;
    } else if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /* 读取数据 */
    size_t size = sizeof(user_params_t);
    ret = nvs_get_blob(handle, USER_PARAMS_NVS_KEY_DATA, params, &size);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Key not found in NVS");
        nvs_close(handle);
        return ESP_ERR_NOT_FOUND;
    } else if (ret != ESP_OK) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Failed to get blob: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ESP_FAIL;
    }

    nvs_close(handle);

    /* 验证魔数 */
    if (params->magic != USER_PARAMS_MAGIC) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Invalid magic: 0x%08X (expected 0x%08X)",
                        params->magic, USER_PARAMS_MAGIC);
        return ESP_FAIL;
    }

    /* 验证大小 */
    if (params->size != sizeof(user_params_t)) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Invalid size: %u (expected %u)",
                        params->size, sizeof(user_params_t));
        return ESP_FAIL;
    }

    /* 验证版本 */
    if (params->version != USER_PARAMS_VERSION) {
        SYS_LOGW_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Version mismatch: 0x%04X (expected 0x%04X)",
                        params->version, USER_PARAMS_VERSION);
        /* 可以在这里做版本迁移，暂时返回错误 */
        return ESP_FAIL;
    }

    /* 验证CRC32 */
    uint32_t calculated_crc = user_params_calculate_crc32(params);
    if (params->crc32 != calculated_crc) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "CRC32 mismatch: stored=0x%08X, calculated=0x%08X",
                        params->crc32, calculated_crc);
        return ESP_FAIL;
    }

    SYS_LOGD_MODULE(SYS_LOG_MODULE_MAIN, TAG, "User params loaded and validated (flags: 0x%02X)", params->flags);
    return ESP_OK;
}

const user_params_t* user_params_get(void)
{
    if (!s_initialized) {
        return NULL;
    }
    return &s_user_params;
}

esp_err_t user_params_set_tt_manual_off(bool manual_off)
{
    if (!s_initialized) {
        SYS_LOGE_MODULE(SYS_LOG_MODULE_MAIN, TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (manual_off) {
        s_user_params.flags |= USER_PARAMS_FLAG_TT_USER_OFF;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "TT module user manual off flag SET");
    } else {
        s_user_params.flags &= ~USER_PARAMS_FLAG_TT_USER_OFF;
        SYS_LOGI_MODULE(SYS_LOG_MODULE_MAIN, TAG, "TT module user manual off flag CLEARED");
    }

    return user_params_save(&s_user_params);
}

bool user_params_is_tt_manual_off(void)
{
    if (!s_initialized) {
        return false;
    }
    return (s_user_params.flags & USER_PARAMS_FLAG_TT_USER_OFF) != 0;
}

uint16_t user_params_get_low_battery_threshold(void)
{
    if (!s_initialized) {
        return USER_PARAMS_DEFAULT_LOW_BATT_MV;
    }
    return s_user_params.low_battery_threshold_mv;
}

esp_err_t user_params_set_low_battery_threshold(uint16_t threshold_mv)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_user_params.low_battery_threshold_mv = threshold_mv;
    return user_params_save(&s_user_params);
}
