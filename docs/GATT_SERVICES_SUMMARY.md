# GATT服务总结

## 当前系统GATT服务列表

### 1. **SPP Service (0xABF0)** - 核心服务
- **UUID**: 0xABF0
- **特征值**: 0xABF1 (读写通知)
- **功能**: 语音数据透传
- **状态**: 始终启用（核心服务，不可停止）
- **文件**: `ble_gatt_server.c`

### 2. **SPP AT Service (0xABF2)** - AT命令透传
- **UUID**: 0xABF2
- **特征值**: 0xABF3 (读写通知)
- **功能**: AT命令透传，支持动态开启/关闭
- **状态**: 默认启用，可通过System服务控制
- **文件**: `spp_at_server.c`

### 3. **Log Service (0xABF4)** - 日志服务
- **UUID**: 0xABF4
- **特征值**: 0xABF5 (读写通知)
- **功能**: 系统日志上报
- **状态**: 默认启用，可通过System服务控制
- **文件**: `gatt_log_server.c`

### 4. **OTA Service (0xABF8)** - 固件更新服务
- **UUID**: 0xABF8
- **特征值**:
  - 0xABF9: Control (控制命令)
  - 0xABFA: Data (固件数据)
  - 0xABFB: Status (状态通知)
- **功能**: MCU和天通模块固件OTA更新
- **状态**: 默认关闭，可通过System服务启动
- **文件**: `gatt_ota_server.c`

### 5. **System Service (0xABFC)** - 系统控制服务 ⭐ 新增
- **UUID**: 0xABFC
- **特征值**:
  - 0xABFD: Control (系统命令)
  - 0xABFE: Info (系统信息读取)
  - 0xABFF: Status (状态通知)
- **功能**: 系统级控制和信息获取
- **状态**: 始终启用（系统管理服务）
- **文件**: `gatt_system_server.c`

## System Service 功能

### 系统信息获取命令

| 命令 | 值 | 功能 | 响应数据 |
|------|-----|------|----------|
| `SYS_CMD_GET_BATTERY_INFO` | 0x01 | 获取电池信息 | `system_battery_info_t` |
| `SYS_CMD_GET_CHARGE_STATUS` | 0x02 | 获取充电状态 | `system_charge_status_t` |
| `SYS_CMD_GET_TT_SIGNAL` | 0x03 | 获取天通模块信号 | `system_tt_signal_t` |
| `SYS_CMD_GET_BLE_TX_POWER` | 0x04 | 获取蓝牙发射功率 | `int8_t` |
| `SYS_CMD_GET_SYSTEM_INFO` | 0x30 | 获取系统信息 | `system_info_t` |

### 系统控制命令

| 命令 | 值 | 功能 | 参数 |
|------|-----|------|------|
| `SYS_CMD_SET_BLE_TX_POWER` | 0x05 | 设置蓝牙发射功率 | `[power:1字节]` |
| `SYS_CMD_SERVICE_START` | 0x10 | 启动服务 | `[service_id:1字节]` |
| `SYS_CMD_SERVICE_STOP` | 0x11 | 停止服务 | `[service_id:1字节]` |
| `SYS_CMD_SERVICE_STATUS` | 0x12 | 查询服务状态 | `[service_id:1字节]` |
| `SYS_CMD_SYSTEM_REBOOT` | 0x20 | 系统重启 | 无 |
| `SYS_CMD_SYSTEM_RESET` | 0x21 | 恢复出厂设置 | 无 |

### 服务ID定义

| 服务ID | 值 | 服务名称 |
|--------|-----|----------|
| `SYS_SERVICE_ID_OTA` | 0x01 | OTA服务 |
| `SYS_SERVICE_ID_LOG` | 0x02 | Log服务 |
| `SYS_SERVICE_ID_AT` | 0x03 | AT服务 |
| `SYS_SERVICE_ID_SPP` | 0x04 | SPP服务（不可停止） |

### 响应码定义

| 响应码 | 值 | 说明 |
|--------|-----|------|
| `SYS_RESP_OK` | 0x00 | 成功 |
| `SYS_RESP_ERROR` | 0x01 | 错误 |
| `SYS_RESP_INVALID_CMD` | 0x02 | 无效命令 |
| `SYS_RESP_INVALID_PARAM` | 0x03 | 无效参数 |
| `SYS_RESP_SERVICE_NOT_FOUND` | 0x04 | 服务未找到 |
| `SYS_RESP_SERVICE_ALREADY_RUNNING` | 0x05 | 服务已在运行 |
| `SYS_RESP_SERVICE_NOT_RUNNING` | 0x06 | 服务未运行 |

## 数据结构

### system_battery_info_t
```c
typedef struct {
    uint16_t voltage_mv;              // 电池电压 (mV)
    int16_t current_ma;               // 电池电流 (mA, 正=充电, 负=放电)
    uint16_t soc_percent;             // 电量百分比 (0-100%)
    uint16_t soh_percent;             // 健康度百分比 (0-100%)
    uint16_t temperature_0_1k;        // 温度 (0.1°K)
    uint16_t full_charge_capacity_mah; // 满充容量 (mAh)
    uint16_t remaining_capacity_mah;   // 剩余容量 (mAh)
    uint8_t charging;                  // 是否充电 (1=是, 0=否)
    uint8_t full_charged;              // 是否充满 (1=是, 0=否)
} system_battery_info_t;
```

### system_charge_status_t
```c
typedef struct {
    uint8_t is_charging;          // 是否充电
    uint8_t is_full;              // 是否充满
    uint16_t charge_voltage_mv;   // 充电电压 (mV)
    uint16_t charge_current_ma;   // 充电电流 (mA)
    uint16_t time_to_full_min;    // 充满时间 (分钟)
} system_charge_status_t;
```

### system_tt_signal_t
```c
typedef struct {
    int8_t rssi;          // 信号强度 (dBm)
    uint8_t reg_status;   // 注册状态
    uint8_t sim_status;   // SIM状态
    uint8_t network_type; // 网络类型
} system_tt_signal_t;
```

### system_info_t
```c
typedef struct {
    uint32_t uptime_sec;      // 运行时间 (秒)
    uint32_t free_heap;       // 空闲堆内存 (字节)
    uint32_t min_free_heap;   // 最小空闲堆 (字节)
    uint8_t cpu_freq_mhz;     // CPU频率 (MHz)
    uint8_t service_status;   // 服务状态位掩码
} system_info_t;
```

## 使用示例

### 1. 启动OTA服务
```
发送到Control特征值: [0x10, 0x01]
响应: [0x00, 0x01]  // OK, OTA服务已启动
```

### 2. 获取电池信息
```
发送到Control特征值: [0x01]
响应: [0x00] + system_battery_info_t数据
```

### 3. 获取系统信息
```
发送到Control特征值: [0x30]
响应: [0x00] + system_info_t数据
```

### 4. 查询服务状态
```
发送到Control特征值: [0x12, 0x01]  // 查询OTA服务
响应: [0x00, 0x01, 0x01]  // OK, 服务ID, 状态(1=运行)
```

## 注意事项

1. **UUID冲突已修复**: OTA服务UUID从0xABF4改为0xABF8，避免与Log服务冲突
2. **服务动态管理**: OTA、Log、AT服务支持动态启动/停止
3. **SPP服务**: 核心服务，始终启用，不可停止
4. **System服务**: 系统管理服务，始终启用，负责管理其他服务
