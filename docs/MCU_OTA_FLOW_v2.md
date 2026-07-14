# MCU 固件 OTA 升级流程 — V2

> **版本**: V2
> **日期**: 2026-07-02
> **变更**: 增加电池检查、重启前关 TT、OTA 期间命令屏蔽、CRC32 日志、超时优化

## 概述

ESP32-S3 自身固件升级。固件通过 BLE 下载到 Flash OTA 分区，校验后切换启动分区并重启。

与 TT OTA 不同，MCU OTA **先存 Flash 再重启**（固件 ~1MB，Flash 有分区空间）。

## 架构

```
APP → BLE → Flash ota_0/ota_1 分区 → esp_ota_set_boot_partition → esp_restart
         (CRC16 逐包校验)           (CRC32 日志记录)
```

## 完整流程

### 阶段 1: APP 发起 OTA

```
APP 发 OTA_CMD_START_MCU (0x01) 到 OTA Control 特征值 (0xABF9)
  ├─ 包含 total_size, crc32
  └─ 设备检查:
      ├─ 电池电压 ≥ 3500mV（新增 V2）
      ├─ ota_partition_init(OTA_PARTITION_MCU, ...)
      ├─ sleep_manager_set_inhibit(true)  ← 禁止睡眠
      └─ ota_timeout_start()  ← 启动超时定时器
```

### 阶段 2: APP 发送固件数据

```
每个 BLE 数据包:
  → handle_ota_data_write()
  → 解析: [SEQ(2)][LEN(2)][data][CRC16(2)]
  → 校验:
      ├─ 包长度匹配
      ├─ CRC16-MODBUS 校验
      └─ 序号连续性（0, 1, 2, ...）
  → ota_partition_write(payload, data_len)
      ├─ 写入 4KB 内部 SRAM 缓冲（不用 PSRAM，Flash 写入安全）
      ├─ 缓冲满 4KB → flush 到 Flash
      └─ 增量计算 CRC32（用于最终校验）
  → 每 5% 进度上报 OTA_STATUS_WRITING
  → 每包重置超时定时器 (30s)
  → 回复 ACK
```

### 阶段 3: 数据传输完成

```
检测: written_size >= total_size
  → ota_timeout_stop()
  → OTA_STATUS_VERIFYING(100%)
  → ota_partition_finalize():
      ├─ flush 剩余缓冲
      ├─ 校验 written_size == total_size
      ├─ CRC32 日志记录（当前只记录不拒绝，待 APP 算法确认后启用拒绝）
      ├─ esp_ota_end() — ESP-IDF 内部校验固件哈希
      └─ esp_ota_set_boot_partition() — 设置新分区为启动分区
```

### 阶段 4: 重启应用新固件

```
OTA_STATUS_SUCCESS(100%)
  → tt_module_user_power_off()  ← 先关 TT（新增 V2）
  → 等 1s
  → sleep_manager_set_inhibit(false)  ← 恢复睡眠
  → ota_cleanup()  ← 清理 OTA 状态
  → 等 2s（让 BLE 通知发出去）
  → esp_restart()  ← 重启，启动新固件
```

## 数据包格式

### OTA 控制包（OTA Control 0xABF9）

```
启动 MCU: [0x01][total_size(4LE)][crc32(4LE)]
启动 TT:  [0x02][total_size(4LE)][crc32(4LE)]
中止:     [0x03]
```

### OTA 数据包（OTA Data 0xABFA）

```
偏移  长度  字段     说明
0     2     seq      序列号（从 0 开始递增，LE）
2     2     data_len 数据长度（LE）
4     N     data     固件数据
4+N   2     crc16    CRC16-MODBUS（覆盖 seq+len+data，LE）
```

## 分区布局

```
factory:    1MB    ← 出厂固件（启动分区）
ota_0:      1MB    ← OTA slot A
ota_1:      1MB    ← OTA slot B
otadata:    8KB    ← ESP-IDF OTA 分区选择数据
```

`esp_ota_set_boot_partition()` 切换 ota_0 / ota_1 为启动分区。

## 缓冲策略

```
g_write_buffer[4096]  ← 内部 SRAM（不是 PSRAM）
  → 累积到 4KB → esp_ota_write() 写 Flash
  → 减少 Flash 擦写次数（Flash 页大小 4KB）

注意：V1 曾尝试放 PSRAM，V2 改回内部 SRAM
      原因：ESP32-S3 Flash 操作期间 PSRAM 访问可能不可靠
```

## 安全保护

### 电池检查（新增 V2）
- 启动前电压 ≥ 3500mV
- 位置：`gatt_ota_server.c` OTA_CMD_START_MCU 处理

### 睡眠禁止
- `ota_partition_init` 后 `sleep_manager_set_inhibit(true)`
- `ota_cleanup` 后 `sleep_manager_set_inhibit(false)`

### 超时保护
- 每包重置 30s 定时器
- 超时触发 `ota_cleanup()` → 中止 OTA

### 命令屏蔽（新增 V2）
MCU OTA 期间以下系统命令被拒绝：

| 命令码 | 命令 | 拒绝原因 |
|--------|------|---------|
| 0x20 | SYSTEM_REBOOT | 会断电中止 OTA |
| 0x22 | REBOOT_MCU | 会重启 MCU |
| 0x23 | REBOOT_TT | 会中断 TT 通信 |
| 0x24 | SET_TT_POWER | 会影响电源 |
| 0x25 | RESET_TT_HARDWARE | 硬件复位 |
| 0x72 | TT_FORCE_ON | 强制重启 TT |
| 0x73 | TT_FORCE_OFF | 强制关 TT |

### 断连清理
- BLE 断连时 `gatt_ota_server_cleanup_on_disconnect()` → `ota_cleanup()`
- 中止 Flash 写入，释放资源

### CRC 校验

| 校验层 | 算法 | 范围 | 状态 |
|--------|------|------|------|
| 包级 | CRC16-MODBUS | seq + len + data | ✅ 拒绝错误包 |
| 固件级 | CRC32 | 全部数据 | ⚠️ 仅日志（待 APP 算法确认后启用拒绝） |
| ESP-IDF | 哈希 | 固件镜像 | ✅ esp_ota_end() 内部校验 |

### 重启前关 TT（新增 V2）
```c
tt_module_user_power_off();  // 先关 TT
vTaskDelay(1000ms);          // 等 TT 关机
sleep_manager_set_inhibit(false);
ota_cleanup();
esp_restart();
```
避免 MCU 重启时 TT 模块处于异常状态。

## 超时参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `OTA_TIMEOUT_MS` | 30000 | 数据包间隔超时（V1 是 8s，V2 改为 30s） |
| 超时动作 | abort | `ota_cleanup()` → 中止 |

## 状态码

| 状态 | 值 | 含义 |
|------|-----|------|
| IDLE | 0x00 | 空闲 |
| WRITING | 0x01 | 数据传输中 |
| VERIFYING | 0x02 | 校验中 |
| SUCCESS | 0x03 | 成功 |
| FAILED | 0x04 | 失败 |

## 响应码

| 响应 | 值 | 含义 |
|------|-----|------|
| OK | 0x00 | 成功 |
| ERROR | 0x01 | 错误 |
| INVALID_STATE | 0x03 | 状态不允许 |
| SIZE_MISMATCH | 0x04 | 大小不匹配 |
| CRC_MISMATCH | 0x05 | CRC 不匹配 |
| INVALID_PACKET | 0x06 | 包格式错误 |
| SEQ_ERROR | 0x07 | 序号错误 |
| PACKET_CRC_ERROR | 0x08 | 包 CRC 错误 |

## 涉及文件

| 文件 | 改动说明 |
|------|---------|
| `main/ble/gatt_ota_server.c` | 电池检查 + 重启前关 TT + 超时 30s + 睡眠禁止 |
| `main/ble/gatt_ota_server.h` | 接口声明 |
| `main/system/ota_partition.c` | g_write_buffer 移回内部 SRAM + CRC32 增量计算 |
| `main/system/ota_partition.h` | 新增 calc_crc32 字段 |
| `main/ble/gatt_system_server.c` | OTA 期间命令屏蔽 |

## V1 → V2 变更记录

| 项目 | V1 | V2 |
|------|-----|-----|
| 电池检查 | ❌ 无 | ✅ ≥3500mV |
| 重启前关 TT | ❌ 直接 esp_restart | ✅ tt_module_user_power_off() |
| 睡眠禁止 | ❌ 无 | ✅ set_inhibit(true/false) |
| 命令屏蔽 | ❌ 无 | ✅ 屏蔽 TT/重启命令 |
| 超时 | 8s | 30s |
| g_write_buffer | 曾尝试 PSRAM | 内部 SRAM |
| CRC32 校验 | 无 | 增量计算（日志记录） |

## APP 交互要点

1. 发 `OTA_CMD_START_MCU`，包含 `total_size` 和 `crc32`
2. 收到 `OTA_RESP_OK` + `OTA_STATUS_WRITING(0%)` 后开始发数据
3. 每包等 ACK 后发下一包（流控）
4. 发完所有数据后设备自动检测 `written >= total`
5. 设备校验 → `OTA_STATUS_VERIFYING(100%)` → `OTA_STATUS_SUCCESS(100%)` → 重启
6. 失败时收到 `OTA_STATUS_FAILED`
7. BLE 断连会自动中止 OTA，APP 需重新发起

## 测试场景

| 场景 | 操作 | 预期结果 |
|------|------|---------|
| 正常升级 | 发完整固件 | SUCCESS → 重启 → 新固件运行 |
| 电池不足 | 电压 < 3500mV | 拒绝 |
| CRC16 包错误 | 发错 CRC 的数据包 | PACKET_CRC_ERROR → 中止 |
| 序号跳号 | 跳过序号 5 | SEQ_ERROR → 中止 |
| 数据不足 | total_size=1MB 但只发 0.9MB | 30s 超时 → FAILED |
| BLE 断连 | 传到一半断开 | cleanup → 中止 |
| OTA 中发 REBOOT | 发 SYSTEM_REBOOT | 拒绝 |
| 重启后回滚 | 新固件启动失败 | ESP-IDF 自动回滚到上一分区 |
