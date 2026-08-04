# 放电保护（对外供电低电量关断）— 开发文档

> 版本: 1.0
> 更新: 2026-08-04
> 适用固件: TTSatModule v1.3.0+

## 概述

设备可作为充电宝对外放电（两条路径）：
- **WPC**：无线充输出（给外设无线充电）
- **VBUS out**：USB 对外输出（boost 模式给外设充电）

为防止设备把自身电量耗尽给外设，当电池 SOC 低于阈值时，**自动关断对外输出**；电量恢复后自动重新开启。

特性：
- 默认**开启**，阈值默认 **20%**
- 滞回：≤阈值关断，≥阈值+5% 才恢复（防抖）
- 阈值与开关可通过 BLE 配置，per-device 存 NVS（持久）
- 关断 / 恢复时主动推事件给 APP

> 该功能与"电池充电"（VBUS/WPC 输入给本机充电）无关，只影响**对外放电输出**。

---

## 1. GATT 通道

复用 System Service：

| 项目 | UUID | 用途 |
|------|------|------|
| Service | `0xABFC` | 系统服务 |
| Control 特征值 | `0xABFD` | 下发命令 / 接收响应 |

命令包格式（Write 到 0xABFD）：
```
[seq][cmd][param_len][params...][crc16_lo][crc16_hi]
```
响应包格式（Notify from 0xABFD）：
```
[seq][cmd][resp_code][data_len][data...]
```
- `resp_code`：0x00 = OK

---

## 2. 命令：SET_DISCHARGE_LIMIT (0x80)

配置 / 查询放电保护。一条命令兼顾设置与查询。

### 2.1 设置

#### 请求

```
[seq][0x80][param_len=1~2][enable][threshold?][crc16]
```

| 字段 | 值 | 说明 |
|------|----|------|
| cmd | 0x80 | |
| param[0] | enable | 0 = 关闭保护（不限制对外输出），1 = 开启保护 |
| param[1] | threshold（**可选**） | 关断阈值 %（范围 5~95）。省略则只改 enable，保留原阈值 |

#### 响应

```
[seq][0x80][resp_code=0x00][data_len=3][enable][threshold][output_off]
```

| 字段 | 说明 |
|------|------|
| data[0] enable | 当前保护开关（0/1） |
| data[1] threshold | 当前阈值 % |
| data[2] output_off | 当前对外输出是否已被关断（0=正常输出，1=已因低电量关断） |

#### 示例

```
开启保护，阈值 20%:
请求: seq=0x01, cmd=0x80, param_len=0x02, param=[0x01, 0x14]  (+crc16)
响应: seq=0x01, cmd=0x80, resp_code=0x00, data_len=0x03, data=[0x01, 0x14, 0x00]

只关闭保护（保留原阈值）:
请求: seq=0x02, cmd=0x80, param_len=0x01, param=[0x00]  (+crc16)
响应: seq=0x02, cmd=0x80, resp_code=0x00, data_len=0x03, data=[0x00, 0x14, 0x00]
```

### 2.2 查询（不带参数）

#### 请求

```
[seq][0x80][param_len=0][crc16]
```

#### 响应

同上 `[enable][threshold][output_off]`，仅返回当前状态、不修改。

```
查询:
请求: seq=0x03, cmd=0x80, param_len=0x00  (+crc16)
响应: seq=0x03, cmd=0x80, resp_code=0x00, data_len=0x03, data=[0x01, 0x14, 0x00]
      → 保护已开启，阈值 20%，当前输出正常（未关断）
```

> threshold 取值范围 5~95，越界会被忽略（保留原值）。默认 `enable=1, threshold=20`。

---

## 3. 事件推送（设备 → APP）

当对外输出被关断 / 恢复时，设备主动推送 Notify（复用电池事件通道，APP 订阅 `ENABLE_BATTERY_REPORT(0x7E)` 即可收到）。

### 事件类型

| 事件 | 码 | 触发条件 |
|------|----|---------|
| `DISCHARGE_CUT` | **0x08** | SOC ≤ 阈值，对外输出已关断（WPC + VBUS out 关） |
| `DISCHARGE_RESTORE` | **0x09** | SOC ≥ 阈值+5%（滞回），对外输出已恢复 |

### 事件负载（8 字节，与电池事件同格式）

```
偏移  字段          类型    说明
0     0x7E         uint8   事件推送标识
1     event_type   uint8   0x08 = 关断，0x09 = 恢复
2     soc          uint8   当前电量 % (0-100)
3     voltage_lo   uint8   当前电压 mV 低字节
4     voltage_hi   uint8   当前电压 mV 高字节
5     temp_lo      uint8   当前温度 0.1°K 低字节
6     temp_hi      uint8   当前温度 0.1°K 高字节
7     charging     uint8   是否正在充电（0/1）
```

### APP 处理建议

- 收到 **0x08**：提示用户"电量过低，已停止对外供电"；可显示恢复所需电量（阈值+5%）
- 收到 **0x09**：提示"电量恢复，对外供电已开启"
- APP 也可通过查询 0x80 的 `output_off` 字段主动获取当前输出状态

---

## 4. 默认值与行为

| 项 | 默认值 |
|----|--------|
| 保护开关 | **开启** |
| 关断阈值 | **20%** |
| 恢复阈值 | 25%（阈值 + 5% 滞回） |
| 关断动作 | 关闭 WPC + VBUS out |
| 配置存储 | NVS（per-device，持久，重启不丢） |

### 关断/恢复时序

```
电量下降: SOC 25% → 20% → 关断 WPC+VBUS out，推 0x08
电量充电: SOC 20% → 25% → 恢复 WPC+VBUS out，推 0x09
（20%~25% 之间保持关断，防抖）
```

---

## 5. APP 集成流程

1. 连接设备，订阅 Control 特征值 0xABFD 的 Notify
2. （可选）订阅电池事件：发 `ENABLE_BATTERY_REPORT(0x7E, flags=0xFF)`
3. （可选）查询当前放电保护配置：发 `0x80`（不带参数）→ 读 `[enable][threshold][output_off]`
4. 提供设置 UI：
   - 开关：发 `0x80 [enable]`（仅开关）
   - 阈值滑条（5~95）：发 `0x80 [enable] [threshold]`
5. 监听事件 0x08/0x09，更新 UI 提示

---

## 6. 注意事项

1. **不影响本机充电**：该功能只控制对外放电（WPC TX + VBUS out），不影响本机被充电（VBUS/WPC 输入）。
2. **滞回 5%**：关断后需充到 阈值+5% 才恢复，避免在阈值附近频繁切换。
3. **事件依赖订阅**：0x08/0x09 通过电池事件通道推送，APP 需先发 `ENABLE_BATTERY_REPORT`。未订阅时不推（但保护动作仍执行，可查询 `output_off`）。
4. **per-device**：配置存各自设备 NVS，每台独立。
5. **阈值范围**：5~95%，越界忽略。
6. **与 TT/升压无关**：不影响卫星模块供电和 MCU 低电量升压保护（独立逻辑）。

---

## 7. 字段速查

| 命令/事件 | 码 | 方向 | 说明 |
|-----------|----|------|------|
| SET_DISCHARGE_LIMIT | 0x80 | APP↔设备 | 配置/查询放电保护（enable + threshold） |
| DISCHARGE_CUT | 0x08 | 设备→APP | 对外输出已关断（事件） |
| DISCHARGE_RESTORE | 0x09 | 设备→APP | 对外输出已恢复（事件） |
