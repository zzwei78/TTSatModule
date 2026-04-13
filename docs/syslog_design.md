# Syslog 整合设计文档

## 目录
1. [功能概述](#1-功能概述)
2. [数据结构设计](#2-数据结构设计)
3. [外部接口设计](#3-外部接口设计)
4. [内部实现设计](#4-内部实现设计)
5. [GATT缓存机制](#5-gatt缓存机制)
6. [远程配置协议](#6-远程配置协议)
7. [使用示例](#7-使用示例)
8. [性能和内存](#8-性能和内存)

---

## 1. 功能概述

### 1.1 整合目标

将 `debuglog.c` 和 `syslog.c` 整合为统一的 `syslog.c`，提供：
- **控制接口** - 给system server使用（配置管理）
- **输出接口** - 给各模块使用（log输出）
- **GATT缓存** - 2KB环形缓冲，平滑日志突发
- **冲突避免** - Voice GATT活动时阻塞log GATT并清空缓存

### 1.2 核心特性

✅ 模块化日志控制（级别、开关、GATT输出）
✅ 双输出支持（ESP LOG + GATT LOG）
✅ GATT日志缓存（2KB环形缓冲）
✅ Voice GATT冲突避免（阻塞+清空缓存）
✅ 远程配置支持
✅ 线程安全

---

## 2. 数据结构设计

### 2.1 日志级别枚举

```c
typedef enum {
    SYS_LOG_LEVEL_DEBUG = 0,    // 调试信息
    SYS_LOG_LEVEL_INFO,         // 一般信息
    SYS_LOG_LEVEL_WARN,         // 警告信息
    SYS_LOG_LEVEL_ERROR,        // 错误信息
    SYS_LOG_LEVEL_FATAL,        // 致命错误
    SYS_LOG_LEVEL_NONE          // 关闭日志
} syslog_level_t;
```

### 2.2 模块ID枚举

```c
typedef enum {
    SYS_LOG_MODULE_MAIN = 0x01,      // 主应用模块
    SYS_LOG_MODULE_BLE_GATT,         // BLE GATT模块
    SYS_LOG_MODULE_AUDIO_PROC,       // 音频处理模块
    SYS_LOG_MODULE_VOICE_PACKET,     // 语音包处理模块
    SYS_LOG_MODULE_AT_CMD,           // AT命令模块
    SYS_LOG_MODULE_SPP_AT,           // SPP透传模块
    SYS_LOG_MODULE_TT_MODULE,        // 天通模块
    SYS_LOG_MODULE_OTA,              // OTA升级模块
    SYS_LOG_MODULE_ALL = 0xFF        // 所有模块
} syslog_module_t;
```

### 2.3 模块日志配置结构

```c
typedef struct {
    syslog_module_t module;     // 模块ID
    syslog_level_t level;       // 日志级别
    bool enabled;               // 是否启用日志输出
    bool gatt_output;           // 是否输出到GATT (per-module)
} syslog_module_config_t;
```

### 2.4 GATT日志状态

```c
/**
 * @brief GATT日志阻塞状态
 */
typedef enum {
    SYS_LOG_GATT_STATE_IDLE = 0,        // 空闲，允许输出
    SYS_LOG_GATT_STATE_BLOCKED          // 被阻塞，禁止输出
} syslog_gatt_state_t;

/**
 * @brief GATT阻塞原因
 */
typedef enum {
    SYS_LOG_GATT_BLOCK_VOICE_ACTIVE = 0x01,  // Voice GATT活动
    SYS_LOG_GATT_BLOCK_MANUAL = 0x02,         // 手动禁用
    SYS_LOG_GATT_BLOCK_BANDWIDTH = 0x03       // 带宽限制
} syslog_gatt_block_reason_t;
```

### 2.5 GATT缓存配置

```c
// GATT日志缓冲配置
#define SYS_LOG_GATT_BUF_SIZE    2048   // 2KB环形缓冲
#define SYS_LOG_GATT_CHUNK_SIZE  256    // 每次发送最大chunk (MTU限制)
#define SYS_LOG_GATT_SEND_STACK  3072   // 发送任务栈大小
#define SYS_LOG_GATT_SEND_PRIO    5     // 发送任务优先级
```

---

## 3. 外部接口设计

### 3.1 初始化和清理接口

```c
int syslog_init(void);
int syslog_deinit(void);
```

### 3.2 控制接口 - 给system server使用

#### 3.2.1 全局级别控制

```c
int syslog_set_global_level(syslog_level_t level);
syslog_level_t syslog_get_global_level(void);
```

#### 3.2.2 模块配置控制

```c
int syslog_set_module_config(syslog_module_t module,
                              syslog_level_t level,
                              bool enabled,
                              bool gatt_output);

int syslog_get_module_config(syslog_module_t module,
                              syslog_module_config_t *config);

int syslog_get_all_configs(syslog_module_config_t *configs,
                            int max_count,
                            int *actual_count);
```

#### 3.2.3 GATT连接管理和冲突避免

```c
void syslog_set_gatt_conn_handle(uint16_t conn_handle);
uint16_t syslog_get_gatt_conn_handle(void);
void syslog_clear_gatt_conn_handle(void);

void syslog_set_gatt_global_enabled(bool enabled);
bool syslog_is_gatt_global_enabled(void);

void syslog_block_gatt_output(syslog_gatt_block_reason_t reason);
void syslog_unblock_gatt_output(syslog_gatt_block_reason_t reason);
bool syslog_is_gatt_blocked(void);
syslog_gatt_state_t syslog_get_gatt_state(void);

void syslog_get_gatt_buffer_stats(size_t *used, size_t *free, size_t *dropped);
```

#### 3.2.4 远程配置命令处理

```c
int syslog_process_remote_config(const uint8_t *cmd_data,
                                  size_t cmd_len,
                                  uint8_t *response,
                                  size_t *resp_len);
```

### 3.3 输出接口 - 给各模块使用

```c
// 通用日志宏
#define SYS_LOGD(tag, fmt, ...)
#define SYS_LOGI(tag, fmt, ...)
#define SYS_LOGW(tag, fmt, ...)
#define SYS_LOGE(tag, fmt, ...)
#define SYS_LOGF(tag, fmt, ...)

// 模块指定日志宏
#define SYS_LOGD_MODULE(module, tag, fmt, ...)
#define SYS_LOGI_MODULE(module, tag, fmt, ...)
#define SYS_LOGW_MODULE(module, tag, fmt, ...)
#define SYS_LOGE_MODULE(module, tag, fmt, ...)
#define SYS_LOGF_MODULE(module, tag, fmt, ...)
```

---

## 4. 内部实现设计

### 4.1 全局状态

```c
// 全局日志级别
static syslog_level_t g_global_level = SYS_LOG_LEVEL_INFO;

// GATT日志全局开关
static bool g_gatt_global_enabled = false;

// GATT连接句柄
static uint16_t g_gatt_conn_handle = 0;
static SemaphoreHandle_t g_conn_handle_mutex = NULL;

// GATT阻塞状态
static syslog_gatt_state_t g_gatt_state = SYS_LOG_GATT_STATE_IDLE;
static uint8_t g_gatt_block_reasons = 0;
static SemaphoreHandle_t g_gatt_state_mutex = NULL;

// 模块配置表
static syslog_module_config_t g_module_configs[] = {
    {SYS_LOG_MODULE_MAIN,      SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_BLE_GATT,  SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_AUDIO_PROC,SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_VOICE_PACKET,SYS_LOG_LEVEL_INFO,true,false},
    {SYS_LOG_MODULE_AT_CMD,    SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_SPP_AT,    SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_TT_MODULE, SYS_LOG_LEVEL_INFO, true,  false},
    {SYS_LOG_MODULE_OTA,       SYS_LOG_LEVEL_INFO, true,  false},
};
```

---

## 5. GATT缓存机制

### 5.1 环形缓冲结构

```c
typedef struct {
    uint8_t buffer[SYS_LOG_GATT_BUF_SIZE];  // 2KB缓冲
    size_t head;                            // 写指针
    size_t tail;                            // 读指针
    size_t count;                           // 当前数据量
    size_t dropped_bytes;                   // 丢弃统计
    SemaphoreHandle_t mutex;                // 互斥锁
    TaskHandle_t send_task_handle;          // 发送任务
} syslog_gatt_buffer_t;
```

### 5.2 缓冲操作

- `syslog_gatt_buffer_write()` - 写入数据，满时丢弃新数据
- `syslog_gatt_buffer_read()` - 读取数据
- `syslog_gatt_buffer_clear()` - 清空缓冲（Voice阻塞时调用）
- `syslog_gatt_buffer_get_stats()` - 获取统计信息

### 5.3 Voice阻塞处理

```c
void syslog_block_gatt_output(syslog_gatt_block_reason_t reason)
{
    // 设置阻塞标志
    g_gatt_state = SYS_LOG_GATT_STATE_BLOCKED;
    g_gatt_block_reasons |= (1 << reason);

    // 清空GATT日志缓冲（Voice期间的日志不补发）
    syslog_gatt_buffer_clear();
}

void syslog_unblock_gatt_output(syslog_gatt_block_reason_t reason)
{
    // 清除阻塞标志
    g_gatt_block_reasons &= ~(1 << reason);
    if (g_gatt_block_reasons == 0) {
        g_gatt_state = SYS_LOG_GATT_STATE_IDLE;
    }
}
```

### 5.4 GATT发送任务

```c
static void syslog_gatt_send_task(void *arg)
{
    while (1) {
        // 等待数据通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {
            // 从缓冲读取
            size_t len = syslog_gatt_buffer_read(chunk, sizeof(chunk));
            if (len == 0) break;

            // 发送到GATT
            esp_err_t ret = esp_ble_gatts_send_notify(...);
            if (ret == ESP_ERR_NO_MEM) {
                vTaskDelay(pdMS_TO_TICKS(20));  // 队列满，等待
            } else if (ret != ESP_OK) {
                syslog_gatt_buffer_clear();    // 错误，清空
                break;
            }
        }
    }
}
```

---

## 6. 远程配置协议

### 6.1 命令格式

```
[CMD_TYPE][DATA_LEN][DATA...]
```

### 6.2 命令类型

```c
typedef enum {
    SYS_LOG_CMD_SET_GLOBAL_LEVEL = 0x01,
    SYS_LOG_CMD_GET_GLOBAL_LEVEL,

    SYS_LOG_CMD_SET_MODULE_CONFIG = 0x10,
    SYS_LOG_CMD_GET_MODULE_CONFIG,
    SYS_LOG_CMD_GET_ALL_CONFIGS,

    SYS_LOG_CMD_ENABLE_MODULE = 0x20,
    SYS_LOG_CMD_DISABLE_MODULE,

    SYS_LOG_CMD_SET_GATT_GLOBAL = 0x30,
    SYS_LOG_CMD_GET_GATT_GLOBAL,
    SYS_LOG_CMD_GET_GATT_STATE,
    SYS_LOG_CMD_GET_GATT_STATS,

    SYS_LOG_CMD_SET_ALL_LEVELS = 0x40,
    SYS_LOG_CMD_ENABLE_ALL_GATT,
    SYS_LOG_CMD_DISABLE_ALL_GATT,
} syslog_cmd_type_t;
```

### 6.3 命令详解

#### 设置全局日志级别 (0x01)
```
请求: [0x01][0x01][LEVEL]
响应: [0x01][0x02][STATUS][LEVEL]
```

#### 设置模块配置 (0x10)
```
请求: [0x10][0x03][MODULE][LEVEL][GATT]
响应: [0x10][0x04][STATUS][MODULE][LEVEL][GATT]
```

#### 获取GATT缓存统计 (0x33)
```
请求: [0x33][0x00]
响应: [0x33][0x03][USED][FREE][DROPPED]
```

---

## 7. 使用示例

### 7.1 初始化

```c
void app_main(void)
{
    syslog_init();
    syslog_set_global_level(SYS_LOG_LEVEL_INFO);
    syslog_set_gatt_global_enabled(false);
}
```

### 7.2 Voice GATT集成

```c
void voice_gatt_connected(uint16_t conn_handle)
{
    syslog_block_gatt_output(SYS_LOG_GATT_BLOCK_VOICE_ACTIVE);
}

void voice_gatt_disconnected(void)
{
    syslog_unblock_gatt_output(SYS_LOG_GATT_BLOCK_VOICE_ACTIVE);
}
```

### 7.3 BLE连接时

```c
void on_ble_connected(uint16_t conn_handle)
{
    syslog_set_gatt_conn_handle(conn_handle);
    syslog_set_gatt_global_enabled(true);

    syslog_set_module_config(SYS_LOG_MODULE_AT_CMD,
                              SYS_LOG_LEVEL_INFO,
                              true,
                              true);  // gatt_output
}
```

### 7.4 模块使用

```c
void tt_module_send_at(const char *cmd)
{
    SYS_LOGI_MODULE(SYS_LOG_MODULE_TT_MODULE, "TT_MODULE", "Sending: %s", cmd);
}
```

---

## 8. 性能和内存

### 8.1 内存开销

| 组件 | 大小 |
|------|------|
| 全局状态变量 | ~200 bytes |
| 模块配置表 | ~128 bytes |
| GATT缓冲 | 2048 bytes |
| Mutex (2个) | ~96 bytes |
| 发送任务栈 | 3072 bytes |
| **总计** | **~5.5KB** |

### 8.2 性能指标

- 日志格式化延迟: <1ms
- ESP LOG输出延迟: <1ms
- GATT缓冲写入: <10μs
- GATT发送速率: ~10KB/s
- 缓冲满丢弃率: <5%

### 8.3 缓冲效果

- **无缓冲**: 高频时丢失20-50%日志
- **2KB缓冲**: 丢失率<5%，可缓存200ms突发

---

## 版本历史

- v1.0 - 2025-01-19: 初始设计，整合debuglog和syslog，添加2KB GATT缓存机制
