# 传感器 BLE 命令接口文档

> ⚠️ **DEPRECATED（已废弃）**
>
> 本文档描述的是 v2.0 旧版上报机制（500ms、14 字节带 0x74 前缀、走 Control 特征值 0xABFD、断开不自动停止）。
> 该机制已被 **v3.0 高频主动上报**取代。
>
> 👉 **新功能请使用：[`sensor_active_report.md`](./sensor_active_report.md)（v3.0）**
> - 新版：50ms 高频推送（通话时 500ms）、独立 Sensor 特征值 **0xABF2**、13 字节无前缀、30s 会话超时、断开自动关闭、未开启时休眠省电
> - 单次查询命令 `GET_SENSOR_STATUS(0x74)` **保持不变**，仍可参考本文档第 1 节
> - 旧版与新版差异详见 `sensor_active_report.md` 第 11 章
>
> 保留本文档仅供旧版 APP 参考与历史记录，不再更新。

---

> 版本: 2.0
> 更新: 2026-06-16

## 概述

设备支持 DA228EC 三轴加速度计和 MMC5603NJ 三轴磁传感器，通过 BLE System Service (UUID: 0xABFC) 的 Control Characteristic (UUID: 0xABFD) 进行交互。

支持两种获取方式：
1. **主动查询** — App 发送命令，设备返回当前传感器原始数据
2. **主动上报** — 设备定期推送 Notification（默认关闭，可配置开关）

## 命令列表

| 命令 | Code | 方向 | 说明 |
|------|------|------|------|
| GET_SENSOR_DATA | 0x74 | App → Device → App | 获取传感器原始数据（加速度+磁场） |
| SET_SENSOR_REPORT | 0x75 | App → Device → App | 配置主动上报开关（param: 0=关 1=开） |
| DISABLE_SENSOR_REPORT | 0x76 | App → Device → App | 关闭上报（兼容旧版，等效 0x75 param=0） |

---

## 1. GET_SENSOR_DATA (0x74)

获取当前传感器原始数据。

### 请求

| 字段 | 值 | 说明 |
|------|------|------|
| cmd | 0x74 | 命令码 |
| param_len | 0 | 无参数 |

### 响应数据（13 bytes）

| 偏移 | 长度 | 类型 | 名称 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | flags | 数据有效标志位 |
| 1-2 | 2 | int16_t LE | accel_x | X 轴加速度，单位: **mg** |
| 3-4 | 2 | int16_t LE | accel_y | Y 轴加速度，单位: **mg** |
| 5-6 | 2 | int16_t LE | accel_z | Z 轴加速度，单位: **mg** |
| 7-8 | 2 | int16_t LE | mag_x | X 轴磁场，单位: **mG** (毫高斯) |
| 9-10 | 2 | int16_t LE | mag_y | Y 轴磁场，单位: **mG** |
| 11-12 | 2 | int16_t LE | mag_z | Z 轴磁场，单位: **mG** |

#### flags 位定义

| Bit | 名称 | 说明 |
|-----|------|------|
| 0 | ACCEL_VALID | 1 = 加速度计数据有效 |
| 1 | MAG_VALID | 1 = 磁传感器数据有效 |
| 7-2 | 保留 | 固定为 0 |

### 交互示例

```
请求: seq=0x01, cmd=0x74, param_len=0x00
响应: seq=0x01, cmd=0x74, resp_code=0x00, data_len=0x0D
      data=[0x03, ax_lo, ax_hi, ay_lo, ay_hi, az_lo, az_hi,
            mx_lo, mx_hi, my_lo, my_hi, mz_lo, mz_hi]
```

---

## 2. SET_SENSOR_REPORT (0x75)

配置传感器数据主动上报开关。默认**关闭**。

开启后，设备每 **500ms** 通过 Control Characteristic 发送一次 Notification，数据格式与 GET_SENSOR_DATA 响应一致（增加 report_id 前缀）。

### 请求

| 字段 | 值 | 说明 |
|------|------|------|
| cmd | 0x75 | 命令码 |
| param_len | 1 | 参数长度 |
| param[0] | 0 或 1 | 0 = 关闭上报，1 = 开启上报（省略时默认开启） |

### 响应

| 字段 | 说明 |
|------|------|
| resp_code | 0x00 = OK |
| data[0] | 当前上报状态：0 = 已关闭，1 = 已开启 |

### 交互示例

```
开启上报:
请求: seq=0x02, cmd=0x75, param_len=0x01, param=[0x01]
响应: seq=0x02, cmd=0x75, resp_code=0x00, data_len=0x01, data=[0x01]

关闭上报:
请求: seq=0x03, cmd=0x75, param_len=0x01, param=[0x00]
响应: seq=0x03, cmd=0x75, resp_code=0x00, data_len=0x01, data=[0x00]
```

---

## 3. DISABLE_SENSOR_REPORT (0x76)

关闭传感器数据主动上报。**兼容旧版命令**，等效于 `0x75 param=0`。

### 请求

| 字段 | 值 | 说明 |
|------|------|------|
| cmd | 0x76 | 命令码 |
| param_len | 0 | 无参数 |

### 响应

| 字段 | 说明 |
|------|------|
| resp_code | 0x00 = OK |
| data[0] | 固定 0x00（已关闭） |

---

## 4. 主动上报 Notification 格式

当上报开启后，设备每 **500ms** 通过 Control Characteristic (0xABFD) 发送 Notification。

### Notification 负载（14 bytes）

| 偏移 | 长度 | 类型 | 名称 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | report_id | 固定 0x74（上报标识，用于区分命令响应） |
| 1 | 1 | uint8 | flags | 数据有效标志 |
| 2-3 | 2 | int16_t LE | accel_x | X 轴加速度，单位: **mg** |
| 4-5 | 2 | int16_t LE | accel_y | Y 轴加速度，单位: **mg** |
| 6-7 | 2 | int16_t LE | accel_z | Z 轴加速度，单位: **mg** |
| 8-9 | 2 | int16_t LE | mag_x | X 轴磁场，单位: **mG** |
| 10-11 | 2 | int16_t LE | mag_y | Y 轴磁场，单位: **mG** |
| 12-13 | 2 | int16_t LE | mag_z | Z 轴磁场，单位: **mG** |

> **与 GET_SENSOR_DATA 的区别**：Notification 多 1 字节 report_id 前缀（偏移 0），其余字段完全一致。App 端可通过第一个字节区分：收到 0xABFD 的数据后，若为完整命令响应包（含 seq/cmd/resp_code），则为查询响应；若首字节为 0x74 且后续正好 13 字节，则为主动上报。

### flags 位定义

| Bit | 名称 | 说明 |
|-----|------|------|
| 0 | ACCEL_VALID | 1 = 加速度计数据有效 |
| 1 | MAG_VALID | 1 = 磁传感器数据有效 |
| 7-2 | 保留 | 固定为 0 |

### 单位说明

| 传感器 | 单位 | 说明 |
|--------|------|------|
| 加速度计 | mg | 毫 g，1g = 1000mg。静止时 Z 轴约 ±1000mg |
| 磁传感器 | mG | 毫高斯，1 Gauss = 1000mG。地磁场约 300~500mG |

### 数据范围

| 传感器 | 范围 | 分辨率 |
|--------|------|--------|
| 加速度计 (±2g) | ±2000 mg | ~1 mg |
| 磁传感器 (±30G) | ±30000 mG | ~1 mG |

---

## 5. App 端计算示例

### 5.1 俯仰角 (Pitch)

```c
float ax_g = accel_x / 1000.0f;  // mg -> g
float ay_g = accel_y / 1000.0f;
float az_g = accel_z / 1000.0f;

float pitch = atan2(-ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / M_PI;
```

### 5.2 横滚角 (Roll)

```c
float roll = atan2(ax_g, az_g) * 180.0 / M_PI;
```

### 5.3 倾斜补偿航向角 (Heading)

```c
float mx_gauss = mag_x / 1000.0f;  // mG -> Gauss
float my_gauss = mag_y / 1000.0f;
float mz_gauss = mag_z / 1000.0f;

// 倾斜补偿
float X_h = mx_gauss * cos(pitch) + mz_gauss * sin(pitch);
float Y_h = mx_gauss * sin(roll) * sin(pitch)
          + my_gauss * cos(roll)
          - mz_gauss * sin(roll) * cos(pitch);

// 航向角
float heading = atan2(Y_h, X_h) * 180.0 / M_PI;
if (heading < 0) heading += 360.0;

// 方向判断
// 0°=北, 90°=东, 180°=南, 270°=西
```

### 5.4 方向文字映射

| 角度范围 | 方向 |
|----------|------|
| 337.5° ~ 22.5° | N (北) |
| 22.5° ~ 67.5° | NE (东北) |
| 67.5° ~ 112.5° | E (东) |
| 112.5° ~ 157.5° | SE (东南) |
| 157.5° ~ 202.5° | S (南) |
| 202.5° ~ 247.5° | SW (西南) |
| 247.5° ~ 292.5° | W (西) |
| 292.5° ~ 337.5° | NW (西北) |

---

## 6. 注意事项

1. **flags 判断**：收到数据后先检查 flags，只处理有效字段（bit0=加速度，bit1=磁场）
2. **上报默认关闭**：开机后上报状态为关闭，需 App 发送 `0x75 param=1` 开启
3. **BLE 断开**：连接断开后上报不会自动停止，App 重新连接后会继续收到上报
4. **上报频率**：500ms 一次，App 端可根据需要降频使用
5. **坐标系**：加速度计和磁传感器坐标系一致，具体轴向参考硬件安装方向
6. **校准**：磁传感器建议首次使用前做 8 字校准，获取硬铁偏移值
7. **Debug 连接**：三个 sensor 命令（0x74/0x75/0x76）均允许 Debug 连接使用
