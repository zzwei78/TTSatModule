# BLE GATT Server 架构说明

## 服务架构

本项目实现了5个自定义GATT服务，以System服务为核心进行统一管理。

### 服务分类

#### 1. 常驻服务（Always Running）

| 服务名 | UUID | 文件 | 功能描述 |
|--------|------|------|----------|
| SPP服务 | 0xABF0 | ble_gatt_server.c | 语音数据传输，双向通信 |
| AT服务 | 0xABF2 | spp_at_server.c | AT命令透传到天通模块 |
| SYSTEM服务 | 0xABFC | gatt_system_server.c | 系统信息查询和服务控制 |

#### 2. 按需服务（On-Demand）

| 服务名 | UUID | 文件 | 功能描述 |
|--------|------|------|----------|
| LOG服务 | 0xABF4 | gatt_log_server.c | 实时日志上报 |
| OTA服务 | 0xABF8 | gatt_ota_server.c | MCU和天通模块固件升级 |
| VOICE任务 | - | voice_packet_handler.c | 语音数据编解码处理 |

## 初始化流程

所有GATT服务在 `ble_gatt_server_init()` 中统一初始化，**仅初始化常驻服务**：

```
main.c: app_main()
    └── ble_gatt_server_init()
        └── gatt_svr_init()
            ├── ble_svc_gap_init()         // GAP服务
            ├── ble_svc_gatt_init()        // GATT服务
            ├── SPP服务 (0xABF0)           // 语音数据 - 常驻
            ├── AT服务 (0xABF2)            // AT命令透传 - 常驻
            └── SYSTEM服务 (0xABFC)        // 系统控制 - 常驻

按需服务（通过SYSTEM服务动态启动）:
    ├── LOG服务 (0xABF4)                  // SYS_CMD_SERVICE_START (0x10) + SYS_SERVICE_ID_LOG (0x02)
    ├── OTA服务 (0xABF8)                  // SYS_CMD_SERVICE_START (0x10) + SYS_SERVICE_ID_OTA (0x01)
    └── VOICE任务                         // SYS_CMD_SERVICE_START (0x10) + SYS_SERVICE_ID_VOICE (0x05)
```

### 服务启动说明

**常驻服务**：系统启动时自动初始化并运行

**按需服务**：需要通过SYSTEM服务的控制命令启动：
- 命令：`SYS_CMD_SERVICE_START (0x10)`
- 参数：服务ID（LOG=0x02, OTA=0x01, VOICE=0x05）

## AT服务架构

### 数据流向

```
BLE客户端
  ↓ AT命令写入
AT服务 (spp_at_server.c)
  ↓ tt_module_send_at_cmd()
天通模块 (tt_module.c)
  ↓ UART/MUX通信
天通模块硬件
  ↑ 响应数据
天通模块 (tt_module.c)
  ↑ spp_at_tt_data_callback()
AT服务 (spp_at_server.c)
  ↑ BLE Notification
BLE客户端
```

### 关键接口

| 函数 | 功能 |
|------|------|
| `tt_module_send_at_cmd()` | 发送AT命令到天通模块 |
| `tt_module_register_data_callback()` | 注册数据接收回调 |
| `spp_at_tt_data_callback()` | 接收天通模块响应 |
| `spp_at_server_send_response()` | 发送响应到BLE客户端 |

### AT服务特点

- **透传模式**：直接转发AT命令，不做解析
- **异步响应**：命令发送后，响应通过UART/MUX接收任务返回
- **类似jtag_serial**：数据流向设计参考jtag_serial架构

## 服务功能

### 1. SPP服务 (0xABF0)

语音数据传输服务，支持双向通信：
- **特征值**: 0xABF1
- **属性**: READ | WRITE | NOTIFY
- **功能**: 接收客户端发送的语音数据，向客户端发送语音数据
- **处理**: `voice_packet_handle()` 处理接收到的AT^AUDPCM命令
- **状态**: 常驻运行

### 2. AT服务 (0xABF2)

AT命令透传服务，用于设备配置和控制：
- **特征值**: 0xABF3
- **属性**: READ | WRITE | NOTIFY
- **功能**: 接收AT命令，转发给天通模块，返回响应
- **数据处理**: `spp_at_tt_data_callback()` 接收天通模块响应
- **接口**: `spp_at_server_send_response()` 发送响应
- **状态**: 常驻运行

### 3. LOG服务 (0xABF4)

实时日志上报服务：
- **特征值**: 0xABF5
- **属性**: NOTIFY
- **功能**: 通过BLE通知发送设备日志
- **接口**: `gatt_log_server_send_log()` 发送日志
- **状态**: 默认关闭，通过SYSTEM服务启动

### 4. OTA服务 (0xABF8)

固件升级服务：
- **控制特征值**: 0xABF9 (开始/中止OTA)
- **数据特征值**: 0xABFA (固件数据)
- **状态特征值**: 0xABFB (读取OTA状态)
- **功能**: 支持MCU和天通模块固件升级
- **状态**: 默认关闭，通过SYSTEM服务启动

### 5. SYSTEM服务 (0xABFC)

系统控制服务：
- **控制特征值**: 0xABFD (系统命令)
- **信息特征值**: 0xABFE (读取系统信息)
- **状态特征值**: 0xABFF (状态通知)

**支持的命令**:
- `0x01` - 获取电池信息
- `0x02` - 获取充电状态
- `0x03` - 获取天通模块信号
- `0x04/0x05` - 获取/设置BLE发射功率
- `0x10` - 启动服务（参数：LOG=0x02, OTA=0x01, VOICE=0x05）
- `0x11` - 停止服务
- `0x12` - 获取服务状态
- `0x20/0x21` - 系统重启/恢复出厂设置
- `0x30` - 获取系统信息

## 动态服务控制

### 启动服务

通过SYSTEM服务的控制特征值（0xABFD）发送命令：

```
[CMD] [SERVICE_ID]
 0x10   0x02     // 启动LOG服务
 0x10   0x01     // 启动OTA服务
 0x10   0x05     // 启动VOICE任务
```

### 停止服务

```
[CMD] [SERVICE_ID]
 0x11   0x02     // 停止LOG服务
 0x11   0x01     // 停止OTA服务
 0x11   0x05     // 停止VOICE任务
```

### 服务状态查询

```
[CMD]
 0x12            // 获取服务状态
响应: [SPP][AT][LOG][OTA][VOICE]
       1   1   0    0     0    (示例: SPP和AT运行，其他关闭)
```

## 相关模块

### voice_packet_handler.c

语音数据处理模块：
- 解析AT^AUDPCM命令
- AMRNB编解码
- Base64编码/解码
- 语音数据队列管理

### audiosvc.c

音频编解码服务：
- AMRNB编码/解码
- 20ms 8kHz 16bit音频帧处理

### tt_module.c

天通模块控制：
- UART通信
- AT命令发送
- 数据接收回调
- GSM0710 MUX支持

## 设计特点

1. **System为核心**：所有服务通过SYSTEM服务统一管理
2. **模块解耦**：每个服务独立实现，通过明确的接口交互
3. **按需启动**：LOG、OTA、VOICE服务默认关闭，节省资源
4. **防重复初始化**：所有动态服务都有防止重复初始化的保护机制
5. **异步架构**：AT服务采用异步响应模式，类似jtag_serial
