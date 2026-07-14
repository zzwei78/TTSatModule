# MCU 固件 OTA 升级流程 — V2.1

> **版本**: V2.1
> **日期**: 2026-07-03
> **变更**: V2 基础上增加命令屏蔽（MCU OTA 也保护）、重启前关 TT、OTA 期间保护

## 概述

ESP32-S3 自身固件升级。固件通过 BLE 下载到 Flash OTA 分区，校验后切换启动分区并重启。
与 TT OTA 不同，MCU OTA **先存 Flash 再重启**。

## 完整流程

### 阶段 1: APP 发起 OTA

```
APP 发 OTA_CMD_START_MCU (0x01) 到 OTA Control 特征值 (0xABF9)
  ├─ 包含 total_size, crc32
  ├─ 电池检查: 电压 ≥ 3500mV（V2 新增）
  ├─ ota_partition_init(OTA_PARTITION_MCU)
  ├─ sleep_manager_set_inhibit(true)
  └─ ota_timeout_start() (30s)
```

### 阶段 2: APP 发送固件数据

```
每个 BLE 数据包:
  → [SEQ(2)][LEN(2)][data][CRC16(2)]
  → 校验: 包长度 + CRC16-MODBUS + 序号连续性
  → ota_partition_write(payload, data_len)
      ├─ 写入 4KB 内部 SRAM 缓冲（不用 PSRAM）
      ├─ 满 4KB → flush 到 Flash
      └─ 增量计算 CRC32
  → 每 5% 上报进度
  → 重置 30s 超时
```

### 阶段 3: 数据传输完成

```
written_size >= total_size
  → ota_partition_finalize():
      ├─ flush 剩余
      ├─ 校验大小匹配
      ├─ CRC32 日志记录（待 APP 算法确认后启用拒绝）
      ├─ esp_ota_end() — 固件哈希校验
      └─ esp_ota_set_boot_partition()
```

### 阶段 4: 重启（V2.1 改进）

```
OTA_STATUS_SUCCESS(100%)
  → tt_module_user_power_off()  ← 先关 TT（V2.1 新增）
  → 等 1s
  → sleep_manager_set_inhibit(false)
  → ota_cleanup()
  → 等 2s
  → esp_restart()
```

## 安全保护

### 电池检查
启动前电压 ≥ 3500mV，否则拒绝。

### 睡眠禁止
OTA 期间 `sleep_manager_set_inhibit(true)`，结束恢复。

### 超时保护
每包重置 30s 定时器，超时 → `ota_cleanup()` 中止。

### 命令屏蔽（V2.1 新增）
MCU OTA 期间也拒绝 TT/重启命令：
```c
if (tt_module_ota_is_in_progress() || gatt_ota_is_in_progress()) {
    // 拒绝: REBOOT / REBOOT_MCU / REBOOT_TT / SET_TT_POWER
    //       / RESET_TT_HARDWARE / TT_FORCE_ON / TT_FORCE_OFF
}
```

### 重启前关 TT（V2.1 新增）
避免 MCU 重启时 TT 模块处于异常状态。

### CRC 校验

| 层级 | 算法 | 状态 |
|------|------|------|
| 包级 | CRC16-MODBUS | ✅ 拒绝错误包 |
| 固件级 | CRC32 | ⚠️ 仅日志（待确认） |
| ESP-IDF | 哈希 | ✅ esp_ota_end() 内部校验 |

### 断连清理
BLE 断连 → `gatt_ota_server_cleanup_on_disconnect()` → `ota_cleanup()`

## 分区布局

```
factory:    1MB    ← 出厂固件
ota_0:      1MB    ← OTA slot A
ota_1:      1MB    ← OTA slot B
otadata:    8KB    ← ESP-IDF 分区选择
```

## V2 → V2.1 变更

| 项目 | V2 | V2.1 |
|------|-----|------|
| 命令屏蔽 | 仅 TT OTA 屏蔽 | MCU OTA 也屏蔽 |
| 重启前关 TT | ❌ 直接 esp_restart | ✅ 先关 TT |
| 重启前清理 | ❌ | ✅ sleep_inhibit(false) + ota_cleanup() |

## 测试场景

| 场景 | 预期 |
|------|------|
| 正常升级 | SUCCESS → 关 TT → 重启 → 新固件 |
| 电池 < 3500mV | 拒绝 |
| CRC 错误 | 中止 |
| BLE 断连 | 中止 |
| OTA 中发 REBOOT | 拒绝 |
| 超时 30s 无数据 | 中止 |
