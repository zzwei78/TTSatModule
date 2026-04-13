# GATT服务架构设计文档

## 服务分类

### 1. 主服务
- **System系统服务** (UUID: 0xABFC)
  - 管理其他服务的启动/停止
  - 提供系统信息（电池、充电、信号、内存等）
  - 系统控制（重启、恢复出厂设置）

### 2. 常驻服务（默认启动）
- **AT命令服务** (UUID: 0xABF2)
  - AT命令透传到TT模块
  - 接收TT模块响应并转发到BLE
  - 通过回调机制与tt_module解耦

- **System系统服务** (UUID: 0xABFC)
  - 核心管理服务

- **SPP语音服务** (UUID: 0xABF0)
  - GATT服务常驻，但语音发送任务默认关闭
  - 接收AMRNB语音数据（AT^AUDPCM格式）

### 3. 默认关闭服务（通过System服务动态启动）
- **Log日志服务** (UUID: 0xABF4)
- **OTA升级服务** (UUID: 0xABF8)
- **Voice语音任务** (非GATT服务，是FreeRTOS任务)

## 服务控制接口

### System服务控制命令

| 命令 | 值 | 描述 |
|------|-----|------|
| SYS_CMD_SERVICE_START | 0x10 | 启动服务 |
| SYS_CMD_SERVICE_STOP | 0x11 | 停止服务 |
| SYS_CMD_SERVICE_STATUS | 0x12 | 查询服务状态 |

### 服务ID定义

| 服务ID | 服务名称 | 默认状态 | 可否停止 |
|--------|----------|----------|----------|
| 0x01 | OTA服务 | 关闭 | ✓ |
| 0x02 | Log服务 | 关闭 | ✓ |
| 0x03 | AT服务 | 运行 | ✗（核心服务） |
| 0x04 | SPP服务 | 运行 | ✗（核心服务） |
| 0x05 | Voice任务 | 关闭 | ✓ |

## 数据流向

### AT命令服务
```
BLE客户端 → spp_at_service_handler()
              ↓
          spp_at_cmd_task()
              ↓
          tt_module_send_at_cmd()
              ↓
          ┌─────────────────┐
          │  TT Module      │
          └─────────────────┘
              ↓
          tt_module UART/MUX RX
              ↓
          tt_module_forward_response_to_gatt()
              ↓
          g_data_callback (spp_at_tt_data_callback)
              ↓
          spp_at_server_send_response()
              ↓
BLE客户端 ← GATT Notification
```

### 语音服务
```
FLASH → audio_test_data_source_task()
           ↓
       voice_data_queue
           ↓
       voice_packet_send_task()
           ↓
       audio_encode() (AMRNB)
           ↓
       base64_encode()
           ↓
       AT^AUDPCM="..."
           ↓
BLE客户端 ← GATT Notification
```

## 模块解耦设计

### 1. AT命令服务与tt_module解耦
- **注册回调机制**：
  ```c
  // spp_at_server.c
  static void spp_at_tt_data_callback(const uint8_t *data, size_t len, void *user_data);

  // 初始化时注册
  tt_module_register_data_callback(spp_at_tt_data_callback, NULL);
  ```

### 2. 服务统一管理
- 所有可选服务通过System服务统一管理
- 服务状态 bitmask: `bit0=OTA, bit1=LOG, bit2=AT, bit3=SPP, bit4=VOICE`

### 3. 静态缓冲池
避免频繁malloc/free：
- `g_spp_at_cmd_buffer` (256字节)
- `g_ota_ctrl_buffer` / `g_ota_data_buffer` (256字节)
- `g_gatt_sys_cmd_buffer` (128字节)

## 关键文件修改

### spp_at_server.c
```c
// 添加回调函数
static void spp_at_tt_data_callback(const uint8_t *data, size_t len, void *user_data)
{
    if (g_at_active_conn_handle == 0) return;
    spp_at_server_send_response(g_at_active_conn_handle, data, len);
}

// 在初始化时注册
int spp_at_server_init(void)
{
    // ... GATT服务注册 ...

    // 注册数据回调到tt_module
    tt_module_register_data_callback(spp_at_tt_data_callback, NULL);
    return 0;
}
```

### gatt_system_server.c
```c
// Voice任务控制
case SYS_SERVICE_ID_VOICE:
    if (g_service_status.voice_enabled) return -1;
    g_voice_task_handle = voice_packet_send_task_start();
    if (g_voice_task_handle != NULL) {
        g_service_status.voice_enabled = true;
    }
    break;
```

### main.c
```c
// Voice任务默认不启动
// voice_packet_send_task_start(); // 已注释
```

## 编译验证
```bash
. ~/esp/esp-idf/export.sh
idf.py build
```

编译成功，仅有少量unused variable警告。

## 测试建议

1. **AT命令测试**
   - 通过BLE发送AT命令
   - 验证响应正确返回

2. **服务控制测试**
   - 通过System服务启动Log/OTA/Voice
   - 验证服务状态正确更新

3. **语音测试**
   - 通过System服务启动Voice任务
   - 验证语音数据正确发送
