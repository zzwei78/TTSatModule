# 天通模块 OTA 升级流程 — V2.1 (Streaming)

> **版本**: V2.1
> **日期**: 2026-07-03
> **变更**: V2 基础上增加升级模式检测、UART 竞争修复、断连恢复、feed 错误处理

## 概述

天通模块固件 2-3MB，ESP32-S3 仅 4MB Flash 无法中转。采用**实时流式传输**：
BLE 收到的数据即时通过 XMODEM 发给 TT 模块，仅需 128 字节 RAM 缓冲。

## 架构

```
APP → BLE → RAM 缓冲(128B) → XMODEM → TT 模块
              ↑ 只需 128 字节
```

## 完整流程

### 阶段 1: APP 发起 OTA

```
APP 发 OTA_CMD_START_TT (0x02) 到 OTA Control 特征值 (0xABF9)
  ├─ 包含 total_size (固件大小)
  ├─ 设备检查电池电压 ≥ 3500mV
  └→ tt_module_ota_begin() → 启动后台 prep task → 返回 OK
```

### 阶段 2: 设备准备 TT 模块（后台任务，~40s）

```
prep task (ota_prep_task):

  Step 1: tt_module_stop()  ← 始终执行（停止 UART RX 任务，防止竞争）
  Step 2: 判断当前模式:
    ├─ 正常模式: AT+UPDATE\r\n → 等 3s → 硬件复位 → 等 "ready"
    └─ 升级模式: 跳过 AT+UPDATE → 硬件复位 → 等 "ready"
  Step 3: setbaud 921600 → 切波特率 → 等 "baud ok"
  Step 4: loadx → 等 'C' (XMODEM CRC 就绪)
  → ready_for_data = true → 回调 UPLOADING(5%)
  → 进入数据间隙看门狗（60s 无数据自动中止）
```

### 阶段 3: APP 发送固件数据

```
APP 收到 OTA_STATUS_WRITING(5%) 后开始发数据

每个 BLE 数据包:
  → CRC16 校验 + 序号验证
  → tt_module_ota_feed(payload, data_len)
  → 累积 128B → XMODEM 包 → UART 发送
  → 等 ACK (3s, 最多重试 10 次)
  → 回调 UPLOADING(5%~85%)

feed 失败 (XMODEM 错误):
  → tt_module_ota_cancel() 立即恢复（不用 finish，避免 120s 阻塞）
  → 通知 APP FAILED
```

### 阶段 4: 自动完成

```
feed() 检测 received_size >= total_size
  → ready_for_data = false
  → spawn finish_task

finish_task:
  → flush 剩余 → EOT 双重确认
  → 等 "done" (120s) 或 "md5 failed"
  → 成功/失败 → ota_recover_tt()
```

## 升级模式检测（V2.1 新增）

### 开机检测

```
tt_module_start() → 上电 TT → tt_mux_init_task:
  UART RX 任务持续监听:
    ├─ "^SIMST: 1" → g_simst_detected = true → 正常模式
    └─ "ready"     → g_upgrade_ready_detected = true → 升级模式

  mux_init_task 等待循环（每 5s 检查一次）:
    ├─ g_upgrade_ready_detected → UPGRADE_MODE
    │   → g_tt_module_powered = true
    │   → sleep_manager_notify_tt_powered_on()
    │   → BLE 推送 [0x08][0x07] 通知 APP
    │   → task 自删除
    └─ SIMST 信号量 → 正常 MUX 初始化 → WORKING
```

### APP 通知

```
检测到升级模式 → BLE 推送 [0x08][0x07]
APP 收到 → 弹窗 "天通模块需要完成固件升级"
APP 发 OTA_CMD_START_TT → prep task 检测到 UPGRADE_MODE → 跳过 AT+UPDATE
```

## XMODEM 协议

### 包格式（128 字节 + CRC-16）

```
┌─────┬──────┬───────┬──────────────────┬────────┬────────┐
│ SOH │ 包号 │ ~包号  │ 128 字节数据      │ CRC_HI │ CRC_LO │
│0x01 │ 1B   │ 1B    │ (不足补 0x1A)     │ 1B     │ 1B     │
└─────┴──────┴───────┴──────────────────┴────────┴────────┘
总计: 133 字节/包 | CRC-16 多项式 0x1021

包号回绕: 1,2,...,255,0,1,... (uint8_t 自然回绕)
```

### EOT 双重确认

```
发 EOT → 等 ACK
  ├─ 收到 ACK → 再发 EOT → 等 ACK → 成功
  └─ NAK/超时 → 重试（最多 3 次）
```

## 安全保护

### 电池检查
- 启动前电压 ≥ 3500mV
- 低电保护覆盖 WORKING + UPGRADE_MODE（< 3100mV 自动关 TT）

### 睡眠禁止
- `begin()` → `sleep_manager_set_inhibit(true)`
- 所有结束路径（成功/失败/取消/断连）→ `sleep_manager_set_inhibit(false)`

### 命令屏蔽
OTA 进行中拒绝：SYSTEM_REBOOT / REBOOT_MCU / REBOOT_TT / SET_TT_POWER / RESET_TT_HARDWARE / TT_FORCE_ON / TT_FORCE_OFF

AT 命令也屏蔽：`tt_module_send_at_cmd_gatt_internal()` 检查 `tt_module_ota_is_in_progress()` 返回 BUSY。

### 数据间隙看门狗
prep 完成后，prep task 转为看门狗，每秒检查 `last_feed_time`。
60s 无 BLE 数据 → 自动中止 + 恢复。

### BLE 断连恢复
断连 → `gatt_ota_server_cleanup_on_disconnect()` → `tt_module_ota_cancel()` → `ota_recover_tt()`

## UPGRADE_MODE 状态管理

### 状态转换

```
开机 → 检测 UART
  ├─ "^SIMST:1" → WORKING
  └─ "ready"    → UPGRADE_MODE → BLE 通知 APP
                    │
                APP 发 OTA_CMD_START_TT
                    │
                prep task (跳过 AT+UPDATE)
                    │
            ┌───────┴───────┐
            │               │
         "done"          失败/超时
            │               │
        复位 TT          复位 TT
            │               │
     ^SIMST:1          又收到 "ready"
            │               │
       WORKING         UPGRADE_MODE
                       → BLE 再通知
```

### 操作权限

| 操作 | UPGRADE_MODE | 说明 |
|------|-------------|------|
| 关 TT (user_power_off) | ✅ 允许 | 用户可推迟 OTA |
| 开 TT (user_power_on) | ❌ 拒绝 | 必须先完成 OTA |
| 发起 OTA | ✅ 简化流程 | 跳过 AT+UPDATE |
| force_on | ❌ 拒绝 | 不打断升级等待 |
| 低电关机 | ✅ < 3100mV | 保护电池 |
| AT 命令 | ❌ BUSY | TT 不支持正常 AT |

## 超时保护

| 阶段 | 超时 | 常量 |
|------|------|------|
| 等 "ready" | 30s | `OTA_TIMEOUT_READY` |
| 等 "baud ok" | 5s | `OTA_TIMEOUT_BAUD_OK` |
| 等 'C' | 30s | `OTA_TIMEOUT_CRC` |
| 单包 ACK | 3s × 10次 | `OTA_TIMEOUT_ACK` |
| BLE 数据间隙 | 60s | `OTA_TIMEOUT_DATA_GAP` |
| 等 "done" | 120s | `OTA_TIMEOUT_DONE` |

## 错误码

| 码 | 常量 | 含义 |
|----|------|------|
| 0 | OK | 成功 |
| 1 | BATTERY | 电池不足 |
| 2 | READY_TIMEOUT | "ready" 超时 |
| 3 | BAUD_TIMEOUT | "baud ok" 超时 |
| 4 | CRC_TIMEOUT | 'C' 超时 |
| 5 | XMODEM | XMODEM 重试耗尽 |
| 6 | CAN | TT 发 CAN 中止 |
| 7 | MD5 | TT 报告 MD5 失败 |
| 8 | DONE_TIMEOUT | "done"/数据间隙超时 |
| 9 | UART | UART 通信错误 |
| 10 | NOT_READY | prep 未完成时收到数据 |

## API

```c
esp_err_t tt_module_ota_init(void);
esp_err_t tt_module_ota_begin(size_t total_size, tt_ota_progress_cb_t progress_cb);
bool      tt_module_ota_is_ready(void);
esp_err_t tt_module_ota_feed(const uint8_t *data, size_t len);
esp_err_t tt_module_ota_finish(void);
esp_err_t tt_module_ota_cancel(void);
tt_ota_state_t  tt_module_ota_get_state(void);
tt_ota_result_t tt_module_ota_get_result(void);
bool      tt_module_ota_is_in_progress(void);
```

## 涉及文件

| 文件 | 说明 |
|------|------|
| `main/tt/tt_module_ota.c` | 流式 OTA + XMODEM + 看门狗 + 升级模式检测 |
| `main/tt/tt_module_ota.h` | API + 错误码 + 电池阈值 |
| `main/tt/tt_module.c` | UART RX 检测 "ready" + mux_init 升级模式分支 + 状态管理 |
| `main/ble/gatt_ota_server.c` | TT 数据路由 + 断连恢复 + 电池检查 |
| `main/ble/gatt_system_server.c` | OTA 命令屏蔽 + TT 状态推送 |

## V2 → V2.1 变更

| 项目 | V2 | V2.1 |
|------|-----|------|
| 升级模式检测 | ❌ 误判为硬件故障 | ✅ 检测 "ready" → UPGRADE_MODE |
| APP 主动通知 | ❌ 无 | ✅ BLE 推送 [0x08][0x07] |
| UART RX 竞争 | 🔴 两条路径不一致 | ✅ 统一调 tt_module_stop() |
| BLE 断连 | ❌ 60s 后才恢复 | ✅ 立即 cancel + 恢复 |
| feed 失败 | 🔴 调 finish 阻塞 120s | ✅ 调 cancel 立即恢复 |
| UPGRADE_MODE 权限 | ❌ 未定义 | ✅ 关 TT 允许，开 TT 拒绝 |
| 低电保护 | ❌ 不覆盖 UPGRADE_MODE | ✅ 覆盖 |
