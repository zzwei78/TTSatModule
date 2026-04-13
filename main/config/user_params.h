/*
 * user_params.h - User Parameters Storage (NVS)
 *
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef USER_PARAMS_H
#define USER_PARAMS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Constants ========== */

#define USER_PARAMS_MAGIC        0x50524D00  /* "PRM\0" - Parameters */
#define USER_PARAMS_VERSION      0x0100      /* Version 1.0 */
#define USER_PARAMS_SIZE         256         /* Total size in bytes */

/* NVS Storage */
#define USER_PARAMS_NVS_NAMESPACE    "user_params"
#define USER_PARAMS_NVS_KEY_DATA     "params_data"

/* Default Values */
#define USER_PARAMS_DEFAULT_LOW_BATT_MV      3500     /* Default low battery threshold 3.5V */

/* ========== User Parameter Flags ========== */

#define USER_PARAMS_FLAG_TT_USER_OFF     (1 << 0)  /* bit0: 天通模块用户主动关闭 (NVS持久化) */
/* bit1-7: 预留 */

/* ========== Data Structures ========== */

/**
 * @brief 用户参数结构（256字节）
 *
 * 此结构存储在NVS中，用于持久化用户配置参数
 */
typedef struct {
    /* ===== 头部信息 (16字节) ===== */
    uint32_t magic;                  /* 魔数：0x50524D00，用于验证数据有效性 */
    uint16_t version;                /* 结构版本：0x0100 */
    uint16_t size;                   /* 结构大小：256字节 */
    uint32_t crc32;                  /* CRC32校验（除crc32字段外的整个结构） */
    uint32_t reserved0;              /* 预留 */

    /* ===== 基本参数 (32字节) ===== */
    uint8_t flags;                   /* 标志位 (见 USER_PARAMS_FLAG_*) */
    uint8_t reserved1[3];            /* 对齐预留 */

    uint16_t low_battery_threshold_mv; /* 低电保护阈值 (mV) */
    uint16_t reserved2;

    uint32_t reserved3[6];           /* 预留扩展 */

    /* ===== 预留扩展区域 (208字节) ===== */
    uint8_t reserved4[208];          /* 预留扩展 */

} user_params_t;

/* ========== API Functions ========== */

/**
 * @brief 初始化用户参数
 *
 * 此函数从NVS加载用户参数，如果不存在则使用默认值初始化并保存。
 * 应在系统启动时调用一次。
 *
 * @return esp_err_t
 *     - ESP_OK: 初始化成功
 *     - ESP_ERR_NO_MEM: 内存不足
 *     - ESP_FAIL: NVS操作失败
 */
esp_err_t user_params_init(void);

/**
 * @brief 反初始化用户参数
 *
 * @return esp_err_t
 */
esp_err_t user_params_deinit(void);

/**
 * @brief 保存用户参数到NVS
 *
 * @param params 用户参数结构指针
 * @return esp_err_t
 *     - ESP_OK: 保存成功
 *     - ESP_ERR_INVALID_ARG: 参数为NULL
 *     - ESP_FAIL: NVS操作失败
 */
esp_err_t user_params_save(const user_params_t *params);

/**
 * @brief 从NVS加载用户参数
 *
 * @param params 用户参数结构指针（输出）
 * @return esp_err_t
 *     - ESP_OK: 加载成功
 *     - ESP_ERR_INVALID_ARG: 参数为NULL
 *     - ESP_ERR_NOT_FOUND: 参数不存在
 *     - ESP_FAIL: NVS操作失败或数据损坏（魔数/CRC错误）
 */
esp_err_t user_params_load(user_params_t *params);

/**
 * @brief 使用默认值初始化用户参数
 *
 * @param params 用户参数结构指针（输出）
 */
void user_params_init_default(user_params_t *params);

/**
 * @brief 获取用户参数（单例）
 *
 * @return 用户参数结构指针，如果未初始化则返回NULL
 */
const user_params_t* user_params_get(void);

/**
 * @brief 设置天通模块用户手动关闭标志
 *
 * 设置此标志后，天通模块将保持关闭状态，即使设备重启也不会自动开启。
 * 必须通过 user_params_clear_tt_user_off() 清除标志才能重新启用。
 *
 * @param manual_off true=用户手动关闭, false=清除标志
 * @return esp_err_t
 *     - ESP_OK: 设置成功
 *     - ESP_ERR_INVALID_STATE: 未初始化
 *     - ESP_FAIL: 保存失败
 */
esp_err_t user_params_set_tt_manual_off(bool manual_off);

/**
 * @brief 获取天通模块用户手动关闭标志
 *
 * @return true=用户手动关闭, false=正常模式（或未初始化）
 */
bool user_params_is_tt_manual_off(void);

/**
 * @brief 获取低电保护阈值
 *
 * @return 低电阈值，如果未初始化则返回默认值
 */
uint16_t user_params_get_low_battery_threshold(void);

/**
 * @brief 设置低电保护阈值
 *
 * @param threshold_mv 阈值电压
 * @return esp_err_t
 */
esp_err_t user_params_set_low_battery_threshold(uint16_t threshold_mv);

#ifdef __cplusplus
}
#endif

#endif /* USER_PARAMS_H */
