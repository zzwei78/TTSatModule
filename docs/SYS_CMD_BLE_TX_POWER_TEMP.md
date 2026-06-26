# BLE TX Power 临时设置命令 — 开发文档

## 概述

命令 ID: `0x7B` (`SYS_CMD_BLE_TX_POWER_TEMP`)

功能：设置 BLE 发射功率，支持超时自动恢复。用于模拟测试不同信号强度场景。

- 设置功率后启动倒计时，到期自动恢复为默认值 (+12 dBm)
- 超时传 0 表示永久修改，不自动恢复
- 重复调用会重置倒计时

## GATT 通道

| 项目 | 值 |
|------|-----|
| Service UUID | `0xABFC` |
| Characteristic UUID (Write/Notify) | `0xABFD` |
| 写入方式 | Write Without Response |
| 响应方式 | Notification |

## 通信协议

### 数据包格式

所有命令和响应都通过 `0xABFD` characteristic 传输，使用统一的包结构 + CRC16 校验。

#### 命令包 (App → Device)

```
偏移  长度  字段         说明
0     1     seq         序列号 (0-255, 循环)
1     1     cmd         命令码 (= 0x7B)
2     1     param_len   参数长度
3     N     params[]    参数数据 (N = param_len)
3+N   2     crc16       CRC16-CCITT 校验 (多项式 0x1021, 初始值 0x0000)
```

CRC 计算范围: 从 `seq` 到 `params[]` 的最后一个字节 (不含 `crc16` 字段本身)。

#### 响应包 (Device → App)

```
偏移  长度  字段         说明
0     1     seq         序列号 (回显命令的 seq)
1     1     cmd         命令码 (回显 0x7B)
2     1     resp_code   响应码 (见下表)
3     1     data_len    数据长度
4     N     data[]      响应数据
4+N   2     crc16       CRC16-CCITT 校验
```

#### 响应码

| 值 | 常量 | 含义 |
|----|------|------|
| 0x00 | `SYS_RESP_OK` | 成功 |
| 0x01 | `SYS_RESP_ERROR` | 内部错误 |
| 0x03 | `SYS_RESP_INVALID_PARAM` | 参数无效 (功率值不支持) |

## 命令 0x7B 详情

### 请求参数

| 参数 | 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|------|
| power | 0 | 1 | int8_t | 发射功率 (dBm)，见下方支持列表 |
| timeout | 1 | 4 | uint32_t LE | 超时时间 (秒)，0 = 永久 |

`param_len` = 1 时只传 power，timeout 默认为 0 (永久)。
`param_len` = 5 时传 power + timeout。

#### 支持的功率值 (dBm)

ESP32-S3 支持以下 16 档:

| dBm | 枚举值 | 典型用途 |
|-----|--------|---------|
| -24 | `ESP_PWR_LVL_N24` | 极弱信号模拟 |
| -21 | `ESP_PWR_LVL_N21` | |
| -18 | `ESP_PWR_LVL_N18` | |
| -15 | `ESP_PWR_LVL_N15` | |
| -12 | `ESP_PWR_LVL_N12` | |
| -9 | `ESP_PWR_LVL_N9` | |
| -6 | `ESP_PWR_LVL_N6` | |
| -3 | `ESP_PWR_LVL_N3` | |
| 0 | `ESP_PWR_LVL_N0` | |
| +3 | `ESP_PWR_LVL_P3` | |
| +6 | `ESP_PWR_LVL_P6` | |
| +9 | `ESP_PWR_LVL_P9` | |
| **+12** | **`ESP_PWR_LVL_P12`** | **默认值 (出厂设置)** |
| +15 | `ESP_PWR_LVL_P15` | |
| +18 | `ESP_PWR_LVL_P18` | |
| +20 | `ESP_PWR_LVL_P20` | |

传入不支持的 dBm 值会返回 `SYS_RESP_INVALID_PARAM (0x03)`。

### 响应数据 (成功时, data_len=6)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 1 | int8_t | 设置后的实际功率 (dBm) |
| 1 | 4 | uint32_t LE | 超时秒数 (回显) |
| 5 | 1 | int8_t | 原始/默认功率 (dBm)，超时后会恢复到这个值 |

## 交互流程

```
App                              Device
  |                                 |
  |--- Write (0x7B, params) ------>|
  |                                 |  设置功率, 启动定时器
  |<-- Notify (response) ----------|
  |                                 |
  |         ...时间流逝...            |
  |                                 |  定时器到期, 自动恢复默认功率
  |                                 |
```

## 示例

### 示例 1: 模拟信号差，设为 -12 dBm，60 秒后恢复

请求:
```
seq:       0x01
cmd:       0x7B
param_len: 0x05
params:    [0xF4, 0x3C, 0x00, 0x00, 0x00]
            │      └────── timeout = 60 (LE) ──────┘
            └ power = -12 (int8_t 0xF4)
crc16:     XX XX
```

响应:
```
seq:       0x01
cmd:       0x7B
resp_code: 0x00 (OK)
data_len:  0x06
data:      [0xF4, 0x3C, 0x00, 0x00, 0x00, 0x0C]
            │      └────── timeout = 60 ──────┘   │
            └ current = -12 dBm                   └ original = +12 dBm
crc16:     XX XX
```

### 示例 2: 设为 0 dBm，30 秒后恢复

请求:
```
params: [0x00, 0x1E, 0x00, 0x00, 0x00]
          │      └─ timeout = 30 ─┘
          └ power = 0 dBm
```

### 示例 3: 设为 -24 dBm (极弱信号)，10 秒后恢复

请求:
```
params: [0xE8, 0x0A, 0x00, 0x00, 0x00]
          │      └─ timeout = 10 ─┘
          └ power = -24 (int8_t 0xE8)
```

### 示例 4: 永久设为 +6 dBm

请求:
```
params: [0x06]
          └ power = +6, 无 timeout 字段 (param_len=1, 默认 timeout=0)
```

### 示例 5: 恢复到默认功率

请求:
```
params: [0x0C]
          └ power = +12 (默认值), timeout=0 (永久)
```

## CRC16 计算示例 (Python)

```python
def crc16_ccitt(data: bytes, init=0x0000) -> int:
    """CRC16-CCITT: polynomial=0x1021, initial=0x0000"""
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

def build_command(seq: int, cmd: int, params: bytes) -> bytes:
    """构建带 CRC 的命令包"""
    packet = bytes([seq, cmd, len(params)]) + params
    crc = crc16_ccitt(packet)
    return packet + crc.to_bytes(2, 'little')

# 示例: 设为 -12 dBm, 60 秒超时
power = -12
timeout = 60
params = bytes([power & 0xFF]) + timeout.to_bytes(4, 'little')
cmd_packet = build_command(seq=1, cmd=0x7B, params=params)
print(cmd_packet.hex())
# 输出: 01 7b 05 f4 3c 00 00 00 XX XX
```

## 测试场景

| 场景 | 操作 | 预期结果 |
|------|------|---------|
| 信号差模拟 | 设 -12 dBm, 60s 超时 | 60s 内 BLE 连接不稳定/断开，60s 后功率恢复 +12 |
| 极弱信号 | 设 -24 dBm, 30s | 连接几乎断开，30s 后恢复 |
| 正常范围边界 | 设 0 dBm, 120s | 缩短有效距离，120s 后恢复 |
| 永久改功率 | 设 +6 dBm, timeout=0 | 功率永久改为 +6，不自动恢复 |
| 恢复默认 | 设 +12 dBm, timeout=0 | 恢复到出厂默认 |
| 非法值 | 设 5 dBm | 返回 `resp_code=0x03` (INVALID_PARAM) |
| 超时前再次设置 | 第一次设 -12dBm/60s，5s 后设 -6dBm/30s | 功率更新为 -6，定时器重置为 30s |
| 不传超时 | param_len=1, 只传功率 | 永久修改，等价于 timeout=0 |
