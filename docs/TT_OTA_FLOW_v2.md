# 天通模块 OTA 升级流程 — V2 (Streaming)

> **版本**: V2 — 实时流式传输（BLE → XMODEM → TT）
> **日期**: 2026-07-02
> **变更**: V1 先存 Flash 再传（固件 2-3MB 存不下），V2 改为实时流式

## 概述

天通模块固件 2-3MB，ESP32-S3 仅 4MB Flash 无法中转。采用**实时流式传输**：
BLE 收到的数据即时通过 XMODEM 发给 TT 模块，仅需 128 字节 RAM 缓冲。

## 架构对比

```
V1（已废弃）:  APP → BLE → 存 Flash 分区 → 读分区 → XMODEM → TT
                        ↑ 2-3MB 存不下 4MB Flash

V2（当前）:    APP → BLE → RAM 缓冲(128B) → XMODEM → TT
                        ↑ 只需 128 字节
```

## 完整流程

### 阶段 1: APP 发起 OTA

```
APP 发 OTA_CMD_START_TT (0x02) 到 OTA Control 特征值 (0xABF9)
  ├─ 包含 total_size (固件大小)
  └─ 设备检查电池电压 ≥ 3500mV
```

### 阶段 2: 设备准备 TT 模块（后台任务，~40s）

```
gatt_ota_server.c → tt_module_ota_begin(total_size, progress_cb)
  → 返回 OK，启动后台 prep task

prep task (ota_prep_task):
  Step 1: tt_module_stop()  ← 停 MUX，不直接动 gsm0710
  Step 2: AT+UPDATE\r\n (原始 UART, 115200)
          → 等 3s（让 TT 处理命令）
  Step 3: 硬件复位 TT（power off → 1s → power on）
          → 等 "ready" (30s 超时)
  Step 4: setbaud 921600\r\n
          → 切本地 UART 到 921600
          → 等 "baud ok" (5s 超时)
  Step 5: loadx\r\n
          → 等 'C' (0x43) XMODEM CRC 模式就绪 (30s 超时)
  → ready_for_data = true
  → 回调 UPLOADING(5%)
  → 进入数据间隙看门狗（60s 无数据自动中止）
```

### 阶段 3: APP 发送固件数据

```
APP 收到 OTA_STATUS_WRITING(5%) 后开始发数据

每个 BLE 数据包:
  → gatt_ota_server: CRC16 校验 + 序号验证
  → tt_module_ota_feed(payload, data_len)
  → 累积到 128B → 组 XMODEM 包(SOH + seq + ~seq + data + CRC16)
  → UART 发送给 TT
  → 等 ACK (3s 超时, 最多重试 10 次)
  → 回调 UPLOADING(5%~85%)
```

### 阶段 4: 数据传输完成

```
feed() 检测到 received_size >= total_size
  → ready_for_data = false（停止接收 + 看门狗退出）
  → 启动后台 finish task

finish task:
  Step 1: flush 剩余缓冲（不足 128B 补 0x1A）
  Step 2: 发 EOT (0x04) × 2（XMODEM 双重确认）
  Step 3: 等 "done" (120s 超时)
          或 "md5 failed"（校验失败）
  Step 4: 成功 → 回调 SUCCESS(100%)
          失败 → 回调 FAILED(0%)
  Step 5: 复位 TT + 恢复 MUX + 解除睡眠禁止
```

## XMODEM 协议详情

### 包格式（128 字节数据 + CRC-16）

```
┌─────┬──────┬───────┬──────────────────┬────────┬────────┐
│ SOH │ 包号 │ ~包号  │ 128 字节数据      │ CRC_HI │ CRC_LO │
│0x01 │ 1B   │ 1B    │ (不足补 0x1A)     │ 1B     │ 1B     │
└─────┴──────┴───────┴──────────────────┴────────┴────────┘
总计: 133 字节/包

CRC-16: 多项式 0x1021, 初始值 0x0000
```

### 控制字符

| 字符 | 值 | 含义 |
|------|-----|------|
| SOH | 0x01 | 包头（128字节数据） |
| EOT | 0x04 | 传输结束 |
| ACK | 0x06 | 确认（包正确） |
| NAK | 0x15 | 否认（包错误，重传） |
| CAN | 0x18 | 取消（中止传输） |
| 'C' | 0x43 | CRC 模式请求（接收端发） |
| EOF | 0x1A | 填充字节（最后一包补齐） |

### 传输状态机

```
等 'C' → 收到 'C'
  → 发 SOH + 数据 + CRC16
  → 收 ACK → 下一包
  → 收 NAK → 重发（≤10次）
  → 收 CAN → 中止
  → 超时   → 重发（≤10次）
  → 全部发完 → 发 EOT → 等 ACK → 再发 EOT → 等 ACK
```

## 命令协议

### TT 模块升级命令序列

| 方向 | 命令 | 响应 |
|------|------|------|
| AP → TT | `AT+UPDATE\r\n` | TT 自动重启进入升级模式 |
| AP → TT | (硬件复位) | 确保进入升级模式（非 BOOTROM） |
| TT → AP | `ready\r\n` | TT 升级模式就绪 |
| AP → TT | `setbaud 921600\r\n` | 切换波特率 |
| TT → AP | `baud ok\r\n` | 波特率切换成功 |
| AP → TT | `loadx\r\n` | 启动 XMODEM 传输 |
| TT → AP | `C` (0x43) | XMODEM CRC 模式就绪 |
| TT → AP | `ACK`/`NAK`/`CAN` | XMODEM 包响应 |
| TT → AP | `done\r\n` | 烧写完成 |
| TT → AP | `md5 failed:0\r\n` | 内存校验失败 |
| TT → AP | `md5 failed:1\r\n` | Flash 校验失败 |

## 超时保护

| 阶段 | 超时 | 常量 |
|------|------|------|
| 等 "ready" | 30s | `OTA_TIMEOUT_READY` |
| 等 "baud ok" | 5s | `OTA_TIMEOUT_BAUD_OK` |
| 等 'C' | 30s | `OTA_TIMEOUT_CRC` |
| 单包 ACK | 3s × 10次 | `OTA_TIMEOUT_ACK` |
| BLE 数据间隙 | 60s | `OTA_TIMEOUT_DATA_GAP` |
| 等 "done" | 120s | `OTA_TIMEOUT_DONE` |

所有超时都触发 `ota_recover_tt()`（复位 TT + 恢复 MUX + 解除睡眠）。

## 安全保护

### 电池检查
- 启动前：电压 ≥ 3500mV，否则拒绝
- 检查位置：`gatt_ota_server.c` + `tt_module_ota_begin()`

### 睡眠禁止
- `begin()` 调用 `sleep_manager_set_inhibit(true)`
- 完成/失败后恢复 `sleep_manager_set_inhibit(false)`

### 命令屏蔽
OTA 期间以下系统命令被拒绝（返回 `SYS_RESP_ERROR`）：

| 命令码 | 命令 | 拒绝原因 |
|--------|------|---------|
| 0x20 | SYSTEM_REBOOT | 会断电中止 OTA |
| 0x22 | REBOOT_MCU | 会断电 TT + 重启 MCU |
| 0x23 | REBOOT_TT | 会中断 XMODEM |
| 0x24 | SET_TT_POWER | 会断电 TT |
| 0x25 | RESET_TT_HARDWARE | 会硬件复位 TT |
| 0x72 | TT_FORCE_ON | 会强制重启 TT |
| 0x73 | TT_FORCE_OFF | 会强制关 TT |

AT 命令也被屏蔽：`tt_module_send_at_cmd_gatt_internal()` 检查 `tt_module_ota_is_in_progress()` 返回 BUSY。

### 数据间隙看门狗
prep 完成后，后台 prep task 转为看门狗模式，每秒检查 `last_feed_time`。
如果 60 秒内没有收到 BLE 数据，自动中止并恢复。

## 进度上报

| 阶段 | 进度 | BLE 状态 |
|------|------|---------|
| 准备中（prep） | 0-5% | WRITING |
| XMODEM 传输 | 5-85% | WRITING |
| 等待 "done" | 85-95% | VERIFYING |
| 完成 | 100% | SUCCESS |
| 失败 | 0% | FAILED |

## 错误码

| 错误码 | 常量 | 含义 |
|--------|------|------|
| 0 | `TT_OTA_RESULT_OK` | 成功 |
| 1 | `TT_OTA_RESULT_ERROR_BATTERY` | 电池不足 |
| 2 | `TT_OTA_RESULT_ERROR_READY_TIMEOUT` | "ready" 超时 |
| 3 | `TT_OTA_RESULT_ERROR_BAUD_TIMEOUT` | "baud ok" 超时 |
| 4 | `TT_OTA_RESULT_ERROR_CRC_TIMEOUT` | 'C' 超时 |
| 5 | `TT_OTA_RESULT_ERROR_XMODEM` | XMODEM 重试耗尽 |
| 6 | `TT_OTA_RESULT_ERROR_CAN` | TT 发 CAN 中止 |
| 7 | `TT_OTA_RESULT_ERROR_MD5` | TT 报告 MD5 校验失败 |
| 8 | `TT_OTA_RESULT_ERROR_DONE_TIMEOUT` | "done" / 数据间隙超时 |
| 9 | `TT_OTA_RESULT_ERROR_UART` | UART 通信错误 |
| 10 | `TT_OTA_RESULT_ERROR_NOT_READY` | prep 未完成时收到数据 |
| 11 | `TT_OTA_RESULT_ERROR_PARTITION` | 分区读取错误（V1 遗留） |

## API 接口

```c
// 初始化（开机时调用一次）
esp_err_t tt_module_ota_init(void);

// 开始 OTA（启动后台 prep task）
esp_err_t tt_module_ota_begin(size_t total_size, tt_ota_progress_cb_t progress_cb);

// 投喂数据（BLE 数据包到达时调用）
esp_err_t tt_module_ota_feed(const uint8_t *data, size_t len);

// 结束传输（feed() 自动检测完成后启动）
esp_err_t tt_module_ota_finish(void);

// 查询状态
bool tt_module_ota_is_in_progress(void);
bool tt_module_ota_is_ready(void);
tt_ota_state_t tt_module_ota_get_state(void);
tt_ota_result_t tt_module_ota_get_result(void);

// 取消
esp_err_t tt_module_ota_cancel(void);
```

## 涉及文件

| 文件 | 改动说明 |
|------|---------|
| `main/tt/tt_module_ota.c` | 完整流式 OTA 实现（prep task + XMODEM + finish task + 看门狗） |
| `main/tt/tt_module_ota.h` | API 声明 + 错误码 + 电池阈值 |
| `main/ble/gatt_ota_server.c` | TT OTA 数据路由（begin/feed/finish）+ MCU OTA 电池检查 + 重启前关 TT |
| `main/ble/gatt_system_server.c` | OTA 期间屏蔽 TT/重启命令 |
| `main/tt/tt_module.c` | AT 路由加 OTA 进行中检查 |

## 不影响的部分

- MCU OTA 流程（partition write + finalize + reboot）不变
- `gsm0710_manager.c` 不直接操作（通过 `tt_module_stop/start`）
- 分区表不需要 `ota_tt` 分区
- `ota_partition.c` 不改

## APP 交互要点

1. APP 发 `OTA_CMD_START_TT` 后，等收到 `OTA_STATUS_WRITING(5%)` 才开始发数据
2. 数据包格式与 MCU OTA 相同：`[SEQ(2)][LEN(2)][data][CRC16(2)]`
3. 每包最大数据由 MTU 决定（通常 ~500 字节）
4. 发完所有数据后，设备自动检测完成并 finalize
5. APP 等待 `OTA_STATUS_SUCCESS(100%)` 或 `OTA_STATUS_FAILED`
6. 失败时可通过 `tt_module_ota_get_result()` 获取详细错误码（日志中可见）

## 测试场景

| 场景 | 操作 | 预期结果 |
|------|------|---------|
| 正常升级 | 发完整固件 | SUCCESS(100%) |
| 电池不足 | 电压 < 3500mV 发起 OTA | 拒绝 |
| APP 中途断连 | 数据传到一半断 BLE | 60s 数据间隙超时 → FAILED → 恢复 |
| TT 不回 ready | AT+UPDATE 后 TT 无响应 | 30s 超时 → FAILED → 恢复 |
| XMODEM NAK | TT 反复 NAK | 10 次重试 → FAILED → 恢复 |
| MD5 校验失败 | TT 报 "md5 failed" | FAILED → 恢复 |
| OTA 中发 REBOOT 命令 | APP 发 SYSTEM_REBOOT | 拒绝 (SYS_RESP_ERROR) |
| OTA 中发 AT 命令 | APP 发 AT+CSQ | 返回 BUSY |
| prep 期间发数据 | APP 提前发数据 | 返回 INVALID_STATE，APP 等待 |
