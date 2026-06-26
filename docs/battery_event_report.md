# 电池事件主动上报 — 开发文档

## 概述

功能：APP 开启后，设备每 10 秒检测一次电池状态，在事件发生时主动推送通知。

支持的事件：
- 电量百分比 (SOC) 变化
- 充电开始 / 充电停止 / 充满
- 高温 / 低温报警（带回滞）

同时保留原有的 APP 主动查询功能（`GET_BATTERY_INFO` 0x01 / `GET_CHARGE_STATUS` 0x02），两种模式并存。

## GATT 通道

| 项目 | 值 |
|------|-----|
| Service UUID | `0xABFC` |
| Characteristic UUID (Write/Notify) | `0xABFD` |
| 命令下发 | Write to 0xABFD |
| 响应 + 事件推送 | Notify from 0xABFD |

## 命令定义

| 命令 | 码 | 方向 | 说明 |
|------|----|------|------|
| `ENABLE_BATTERY_REPORT` | 0x7E | APP → 设备 | 开启电池事件推送 |
| `DISABLE_BATTERY_REPORT` | 0x7F | APP → 设备 | 关闭电池事件推送 |

## 命令详情

### ENABLE_BATTERY_REPORT (0x7E)

#### 请求

```
[seq][0x7E][param_len][flags][soc_threshold][crc16]
```

| 参数 | 偏移 | 长度 | 必填 | 默认值 | 说明 |
|------|------|------|------|--------|------|
| flags | 0 | 1 | 否 | 0xFF (全部) | 事件订阅位掩码 |
| soc_threshold | 1 | 1 | 否 | 1 (%) | SOC 变化触发阈值 |

`param_len=0` 时使用全部默认值。

#### flags 位掩码

| bit | 常量 | 事件 |
|-----|------|------|
| 0 | `BATTERY_REPORT_FLAG_SOC` | SOC 变化 |
| 1 | `BATTERY_REPORT_FLAG_CHARGE` | 充电状态变化 |
| 2 | `BATTERY_REPORT_FLAG_TEMP` | 温度异常 |
| 3-7 | 保留 | — |
| — | `BATTERY_REPORT_FLAG_ALL` (0xFF) | 订阅全部 |

#### 响应

```
[seq][0x7E][resp_code][data_len=2][flags][soc_threshold][crc16]
```

### DISABLE_BATTERY_REPORT (0x7F)

#### 请求

```
[seq][0x7F][0x00][crc16]
```

#### 响应

```
[seq][0x7F][resp_code=0x00][data_len=1][0x00][crc16]
```

## 事件推送格式（设备 → APP）

事件推送是**设备主动发送**的 Notify（无需 APP 请求），固定 7 字节：

```
偏移  字段          类型     说明
0     0x7E         uint8    事件推送标识（APP 据此区分事件 vs 命令响应）
1     event_type   uint8    事件类型码（见下表）
2     soc          uint8    当前电量百分比 (0-100)
3     voltage_lo   uint8    当前电压 mV 低字节
4     voltage_hi   uint8    当前电压 mV 高字节
5     temp_lo      uint8    当前温度 0.1°K 低字节
6     temp_hi      uint8    当前温度 0.1°K 高字节
```

> 温度转换：`°C = temp_0_1k / 10 - 273`

### APP 区分事件推送与命令响应

| 来源 | 第 1 字节 | 说明 |
|------|----------|------|
| 命令响应 | seq (0-255) | APP 发送命令后的回显序号 |
| 传感器推送 | 0x74 | `SYS_CMD_GET_SENSOR_STATUS` |
| 电池事件推送 | 0x7E | `SYS_CMD_ENABLE_BATTERY_REPORT` |

> 当 APP 未发送命令时收到 Notify，根据第 1 字节判断推送类型。

## 事件类型

| 码 | 常量 | 触发条件 | 回滞 |
|----|------|---------|------|
| 0x01 | `BATTERY_EVENT_SOC_CHANGE` | SOC 变化量 ≥ soc_threshold | — |
| 0x02 | `BATTERY_EVENT_CHARGE_START` | 电流 < -200mA（开始充电） | — |
| 0x03 | `BATTERY_EVENT_CHARGE_STOP` | 充电停止且 SOC < 99% | — |
| 0x04 | `BATTERY_EVENT_CHARGE_FULL` | 充电停止且 SOC ≥ 99% | — |
| 0x05 | `BATTERY_EVENT_TEMP_HIGH` | 温度 ≥ 55°C | ≤ 52°C 解除 |
| 0x06 | `BATTERY_EVENT_TEMP_LOW` | 温度 ≤ -10°C | ≥ -7°C 解除 |

### 充电状态判断逻辑

```
统一 API 电流方向: 负 = 充电, 正 = 放电

charging = (current < -200mA)

状态转换:
  放电 → 充电               : 推送 CHARGE_START (0x02)
  充电 → 放电 (SOC ≥ 99%)   : 推送 CHARGE_FULL (0x04)
  充电 → 放电 (SOC < 99%)   : 推送 CHARGE_STOP (0x03)
```

### 温度回滞机制

```
高温:
  温度 ≥ 55°C 且 !active → 推送 TEMP_HIGH, active=true
  温度 ≤ 52°C             → active=false (允许下次再报)

低温:
  温度 ≤ -10°C 且 !active → 推送 TEMP_LOW, active=true
  温度 ≥ -7°C              → active=false (允许下次再报)
```

回滞 3°C，避免在阈值附近频繁推送。

## 检测周期

```
fuel_gauge_monitor_task (60 秒大循环):
  │
  ├── 每次循环开始: 读取电池数据 + 日志 + 低电保护 (每 60 秒)
  │
  └── 等待循环 (1 秒粒度):
       ├── 第 10 秒: battery_event_check() ← 事件检测
       ├── 第 20 秒: battery_event_check()
       ├── 第 30 秒: battery_event_check()
       ├── 第 40 秒: battery_event_check()
       ├── 第 50 秒: battery_event_check()
       └── 第 60 秒: 回到大循环开始
```

事件检测每 **10 秒**执行一次。首次调用时初始化基线（不推送事件）。

## 配置常量

| 常量 | 值 | 位置 |
|------|----|------|
| 检测周期 | 10 秒 | power_manage.c (monitor task 等待循环) |
| 默认 SOC 阈值 | 1% | gatt_system_server.h `BATTERY_REPORT_DEFAULT_SOC_THRESHOLD` |
| 充电判断电流阈值 | 200 mA | power_manage.c `BATT_CHARGE_CURRENT_MA` |
| 高温报警 | 55°C | power_manage.c `BATT_TEMP_HIGH_THRESHOLD` |
| 高温解除 | 52°C | power_manage.c `BATT_TEMP_HIGH_RECOVERY` |
| 低温报警 | -10°C | power_manage.c `BATT_TEMP_LOW_THRESHOLD` |
| 低温解除 | -7°C | power_manage.c `BATT_TEMP_LOW_RECOVERY` |

## 示例

### 示例 1: 开启全部事件推送（默认参数）

请求:
```
seq:       0x01
cmd:       0x7E
param_len: 0x00
crc16:     XX XX
```

响应:
```
seq:       0x01
cmd:       0x7E
resp_code: 0x00 (OK)
data_len:  0x02
data:      [0xFF, 0x01]    flags=ALL, soc_threshold=1%
crc16:     XX XX
```

### 示例 2: 只订阅充电事件，SOC 阈值 5%

请求:
```
cmd:       0x7E
param_len: 0x02
params:    [0x02, 0x05]    flags=CHARGE_ONLY, threshold=5%
```

### 示例 3: 收到电量变化事件

设备推送 (Notify):
```
[0x7E][0x01][0x57][0x4C 0x10][0xB7 0x09]

解析:
  0x7E          电池事件标识
  0x01          SOC_CHANGE
  0x57 = 87     SOC=87%
  0x104C = 4172 电压=4172mV
  0x09B7 = 2487 温度=248.7°K = -24.3°C → 室温约 24.3°C (2487/10-273=-24.3?)
```

> 温度计算修正: `2487 / 10 = 248.7`, `248.7 - 273 = -24.3°C`。实际室温约 25°C 对应 `temp_0_1k ≈ 2980` → `0xBA 0x0B`。

### 示例 4: 收到充电开始事件

设备推送:
```
[0x7E][0x02][0x35][0x0C 0x10][0xBA 0x0B]

解析:
  0x7E     电池事件
  0x02     CHARGE_START
  0x35=53  SOC=53%
  0x100C   4108mV
  0x0BBA   3002 → 300.2-273=27.2°C... 实际约 25°C
```

### 示例 5: 收到充满通知

设备推送:
```
[0x7E][0x04][0x63][0x68 0x10][0xBB 0x0B]

解析:
  0x7E     电池事件
  0x04     CHARGE_FULL
  0x63=99  SOC=99%
  0x1068   4200mV (满充电压)
```

### 示例 6: 关闭推送

请求:
```
cmd:       0x7F
param_len: 0x00
```

## CRC16 计算 (Python)

```python
def crc16_ccitt(data: bytes, init=0x0000) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

def build_command(seq: int, cmd: int, params: bytes = b"") -> bytes:
    packet = bytes([seq, cmd, len(params)]) + params
    crc = crc16_ccitt(packet)
    return packet + crc.to_bytes(2, 'little')

# 开启全部事件推送
print(build_command(1, 0x7E).hex())

# 只订阅充电事件, SOC阈值5%
print(build_command(1, 0x7E, bytes([0x02, 0x05])).hex())

# 关闭推送
print(build_command(1, 0x7F).hex())

# 解析事件推送
def parse_battery_event(raw: bytes) -> dict:
    assert raw[0] == 0x7E, "Not a battery event"
    event_type = raw[1]
    soc = raw[2]
    voltage_mv = raw[3] | (raw[4] << 8)
    temp_0_1k = raw[5] | (raw[6] << 8)
    temp_c = temp_0_1k / 10 - 273
    event_names = {
        0x01: "SOC_CHANGE",
        0x02: "CHARGE_START",
        0x03: "CHARGE_STOP",
        0x04: "CHARGE_FULL",
        0x05: "TEMP_HIGH",
        0x06: "TEMP_LOW",
    }
    return {
        "event": event_names.get(event_type, f"UNKNOWN(0x{event_type:02X})"),
        "soc": soc,
        "voltage_mv": voltage_mv,
        "temp_c": temp_c,
    }
```

## 涉及文件

| 文件 | 改动 |
|------|------|
| `main/ble/gatt_system_server.h` | 命令码 0x7E/0x7F + 事件类型常量 + `send_battery_event()` 声明 |
| `main/ble/gatt_system_server.c` | `send_battery_event()` 实现 + 命令处理 case + debug 白名单 |
| `main/system/power_manage.h` | `set_battery_report()` / `disable_battery_report()` 声明 |
| `main/system/power_manage.c` | 事件检测逻辑 + 状态管理 + 每 10s 挂载到 monitor task |

## 与现有功能的关系

| 现有功能 | 关系 |
|---------|------|
| `GET_BATTERY_INFO` (0x01) | **不变**。APP 随时可查询完整电池信息 |
| `GET_CHARGE_STATUS` (0x02) | **不变**。APP 随时可查询充电状态 |
| 传感器推送 (0x75/0x76) | **独立**。传感器和电池各有独立的 enable/disable |
| `fuel_gauge_monitor_task` | **复用**。60s 大循环不变，内嵌 10s 事件检测 |

## 限制

- 事件推送需要 BLE 连接（无连接时事件丢弃，不缓存）
- 检测周期 10 秒，短于 10 秒的事件可能遗漏（如快速插拔 USB）
- 充电/放电判断基于电流阈值 (200mA)，极小电流充电可能被判定为放电
- 断开重连后需重新 ENABLE（状态不持久化）

## 测试场景

| 场景 | 操作 | 预期结果 |
|------|------|---------|
| 开启推送 | APP 发 ENABLE(0xFF, 1) | 响应 OK，flags=0xFF, threshold=1 |
| SOC 变化 | 等待电池放电 ≥1% | 收到 SOC_CHANGE 事件 |
| 开始充电 | 插入 USB | ≤10s 内收到 CHARGE_START |
| 充满 | 充电至 100% | 收到 CHARGE_FULL (SOC≥99 且停止充电) |
| 停止充电 | 拔出 USB (SOC<99%) | 收到 CHARGE_STOP |
| 高温 | 加热电池至 ≥55°C | 收到 TEMP_HIGH |
| 高温解除 | 降温至 ≤52°C | 不重复推送；再加热至 55°C 再次推送 |
| 低温 | 冷却至 ≤-10°C | 收到 TEMP_LOW |
| 关闭推送 | APP 发 DISABLE | 响应 OK，后续不再收到事件 |
| 无连接时事件 | 断开 BLE，触发事件 | 事件丢弃，不缓存，重连后不补发 |
| 主动查询 | 推送开启时发 GET_BATTERY_INFO | 正常响应（查询和推送独立） |
| 只订阅充电 | ENABLE(0x02, 5) | 只收到 CHARGE_* 事件，不收到 SOC/温度 |
