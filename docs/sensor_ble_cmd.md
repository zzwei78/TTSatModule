# 传感器 BLE 命令接口文档

> 版本: 1.0
> 更新: 2026-06-03

## 概述

设备支持 DA228EC 三轴加速度计和 MMC5603NJ 三轴磁传感器，通过 BLE System Service (UUID: 0xABFC) 的 Control Characteristic (UUID: 0xABFD) 进行交互。

## 命令列表

| 命令 | Code | 方向 | 说明 |
|------|------|------|------|
| GET_SENSOR_STATUS | 0x74 | App → Device → App | 查询传感器是否存在 |
| ENABLE_SENSOR_REPORT | 0x75 | App → Device | 开启传感器数据主动上报 |
| DISABLE_SENSOR_REPORT | 0x76 | App → Device | 关闭传感器数据主动上报 |

---

## 1. GET_SENSOR_STATUS (0x74)

### 请求

| 字段 | 值 | 说明 |
|------|------|------|
| cmd | 0x74 | 命令码 |
| param_len | 0 | 无参数 |

### 响应

| 字段 | 偏移 | 长度 | 说明 |
|------|------|------|------|
| resp_code | - | 1 | 0x00 = OK |
| data[0] | 0 | 1 | 传感器存在标志位 |

#### flags 位定义

| Bit | 名称 | 说明 |
|-----|------|------|
| 0 | ACCEL_PRESENT | 1 = DA228EC 加速度计存在 |
| 1 | MAG_PRESENT | 1 = MMC5603 磁传感器存在 |
| 7-2 | 保留 | 固定为 0 |

#### 示例

- `0x03` = 两个传感器都存在
- `0x01` = 仅加速度计存在
- `0x02` = 仅磁传感器存在
- `0x00` = 无传感器

### 交互示例

```
请求: seq=0x01, cmd=0x74, param_len=0x00
响应: seq=0x01, cmd=0x74, resp_code=0x00, data_len=0x01, data=[0x03]
```

---

## 2. ENABLE_SENSOR_REPORT (0x75)

开启传感器数据主动上报。设备将每 **500ms** 通过 Control Characteristic 发送一次 Notification，包含最新的加速度计和磁传感器数据。

### 请求

| 字段 | 值 | 说明 |
|------|------|------|
| cmd | 0x75 | 命令码 |
| param_len | 0 | 无参数 |

### 响应

| 字段 | 说明 |
|------|------|
| resp_code | 0x00 = OK |
| data[0] | 0x00 = OK |

### 交互示例

```
请求: seq=0x02, cmd=0x75, param_len=0x00
响应: seq=0x02, cmd=0x75, resp_code=0x00, data_len=0x01, data=[0x00]
```

开启后，设备每 500ms 主动发送 Notification（见下节数据格式）。

---

## 3. DISABLE_SENSOR_REPORT (0x76)

关闭传感器数据主动上报，停止发送 Notification。

### 请求

| 字段 | 值 | 说明 |
|------|------|------|
| cmd | 0x76 | 命令码 |
| param_len | 0 | 无参数 |

### 响应

| 字段 | 说明 |
|------|------|
| resp_code | 0x00 = OK |
| data[0] | 0x00 = OK |

### 交互示例

```
请求: seq=0x03, cmd=0x76, param_len=0x00
响应: seq=0x03, cmd=0x76, resp_code=0x00, data_len=0x01, data=[0x00]
```

---

## 4. 主动上报数据格式

当 ENABLE_SENSOR_REPORT 开启后，设备每 **500ms** 通过 Control Characteristic (0xABFD) 发送 Notification，数据格式如下：

### Notification 负载 (14 bytes)

| 偏移 | 长度 | 类型 | 名称 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | report_id | 固定 0x74（上报标识） |
| 1 | 1 | uint8 | flags | 数据有效标志 |
| 2-3 | 2 | int16_t LE | accel_x | X 轴加速度，单位: **mg** |
| 4-5 | 2 | int16_t LE | accel_y | Y 轴加速度，单位: **mg** |
| 6-7 | 2 | int16_t LE | accel_z | Z 轴加速度，单位: **mg** |
| 8-9 | 2 | int16_t LE | mag_x | X 轴磁场，单位: **mG** (毫高斯) |
| 10-11 | 2 | int16_t LE | mag_y | Y 轴磁场，单位: **mG** |
| 12-13 | 2 | int16_t LE | mag_z | Z 轴磁场，单位: **mG** |

### flags 位定义

| Bit | 名称 | 说明 |
|-----|------|------|
| 0 | ACCEL_VALID | 1 = 加速度计数据有效 |
| 1 | MAG_VALID | 1 = 磁传感器数据有效 |
| 7-2 | 保留 | 固定为 0 |

### 单位说明

| 传感器 | 单位 | 说明 |
|--------|------|------|
| 加速度计 | mg | 毫 g，1g = 1000mg。静止时 Z 轴约 1000mg |
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

1. **上报前先查询状态**: 建议先发送 GET_SENSOR_STATUS 确认哪些传感器存在
2. **flags 判断**: 收到 Notification 后先检查 flags，只处理有效数据
3. **BLE 断开自动停止**: BLE 连接断开后上报自动停止，App 重新连接后需再次发送 0x75
4. **上报频率**: 500ms 一次，App 端可根据需要降频使用
5. **坐标系**: 加速度计和磁传感器坐标系一致，具体轴向参考硬件安装方向
6. **校准**: 磁传感器建议首次使用前做 8 字校准，获取硬铁偏移值
