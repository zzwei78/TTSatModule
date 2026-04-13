# 天通模块OTA升级流程说明

## 概述

本文档说明天通模块OTA升级的完整流程，从BLE GATT下载到XMODEM传输的完整实现。

## 整体架构

```
┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│ BLE Client  │─────▶│ ESP32 (MCU)  │─────▶│ TT Module   │
└─────────────┘      └──────────────┘      └─────────────┘
                            │
                            ├─ ota_tt 分区
                            └─ XMODEM 协议
```

## 完整流程

### 阶段1: BLE GATT 下载固件到 ota_tt 分区

1. **初始化OTA分区**
   ```
   BLE Client → ESP32: OTA_CMD_START_TT
   ESP32: ota_partition_init(OTA_PARTITION_TT, size, crc32)
   ```

2. **传输固件数据**
   ```
   BLE Client → ESP32: OTA_DATA (分块传输)
   ESP32: ota_partition_write(data, len)
   ```

3. **传输完成**
   ```
   ESP32 → BLE Client: OTA_STATUS_SUCCESS (100%)
   ```

### 阶段2: 自动触发天通模块固件更新

当GATT OTA下载完成后，系统会自动检测分区类型并执行相应操作：

```c
// 在 gatt_ota_server.c 中
if (part_type == OTA_PARTITION_TT || strcmp(partition_name, "ota_tt") == 0) {
    // 天通模块OTA - 触发XMODEM更新
    tt_module_ota_start_update(tt_ota_progress_callback);
} else {
    // MCU OTA - 重启应用新固件
    esp_restart();
}
```

### 阶段3: XMODEM固件传输

1. **准备阶段** (TT_OTA_STATE_PREPARING)
   - 停止MUX模式
   - 停止AT命令任务
   - 重置天通模块

2. **发送AT+UPDATE命令** (TT_OTA_STATE_WAITING_READY)
   ```
   ESP32 → TT: AT+UPDATE\r\n
   TT → ESP32: ready\r\n
   ```

3. **切换波特率**
   ```
   ESP32 → TT: setbaud 3000000\r\n
   TT → ESP32: baud ok\r\n
   ESP32: 修改UART波特率为3000000
   ```

4. **进入XMODEM模式**
   ```
   ESP32 → TT: loadx\r\n
   TT → ESP32: C (CRC模式)
   ```

5. **流式传输固件** (TT_OTA_STATE_UPLOADING)
   ```
   循环:
     1. 从 ota_tt 分区读取 128 字节
     2. 构造 XMODEM 数据包
     3. 发送到天通模块
     4. 等待 ACK
     5. 更新进度
     6. 重复直到传输完成
   ```

6. **传输完成** (TT_OTA_STATE_VERIFYING)
   ```
   ESP32 → TT: EOT (0x04)
   TT → ESP32: ACK + done\r\n
   ```

7. **恢复运行** (TT_OTA_STATE_COMPLETED)
   - 天通模块自动重启
   - 模块启动后默认进入AT命令模式
   - 检测到 `SIMST:1` 后自动切换到MUX模式
   - 无需手动恢复

## 关键代码文件

### 1. tt_module_ota.h / tt_module_ota.c
- 天通模块OTA核心实现
- XMODEM协议实现
- 从ota_tt分区流式读取固件
- 状态管理和进度报告

### 2. gatt_ota_server.c
- GATT OTA服务实现
- 接收BLE客户端的固件数据
- 写入ota_tt分区
- 下载完成后自动触发tt_module_ota_start_update()

### 3. ota_partition.c
- 分区操作实现
- 提供ota_partition_get_type()判断当前OTA类型
- 提供ota_partition_get_name()获取分区名称

## API使用

### 初始化

```c
// 在main函数中初始化
tt_module_ota_init();
```

### BLE客户端触发流程

```c
// 1. 开始天通模块OTA (通过BLE GATT)
// BLE Client发送命令: OTA_CMD_START_TT

// 2. 传输固件数据
// BLE Client分块发送固件数据

// 3. ESP32自动执行:
//    a. 写入ota_tt分区
//    b. 验证CRC32
//    c. 自动调用tt_module_ota_start_update()
//    d. 通过XMODEM传输给天通模块
```

### 进度回调

```c
void tt_ota_progress_callback(tt_ota_state_t state, int progress) {
    ESP_LOGI(TAG, "TT OTA: state=%d, progress=%d%%", state, progress);

    // 状态映射:
    // TT_OTA_STATE_UPLOADING → OTA_STATUS_WRITING
    // TT_OTA_STATE_VERIFYING → OTA_STATUS_WRITING
    // TT_OTA_STATE_COMPLETED → OTA_STATUS_SUCCESS
    // TT_OTA_STATE_FAILED → OTA_STATUS_FAILED
}
```

## 分区配置

需要在 partition table 中定义 ota_tt 分区：

```csv
# Name,    Type, SubType, Offset,  Length, Flags
ota_tt,    data, ota_tt,  ,        2048K,
ota_mcu,   data, ota,    ,        2048K,
```

## XMODEM协议细节

### 数据包格式

```
+-----+-------+-------+----------+----------+-----+
| SOH | NUM   | ~NUM  | DATA     | CRC_H    | CRC_L|
| 0x01|(1byte)|(1byte)|(128bytes)|(1byte)  |(1byte)|
+-----+-------+-------+----------+----------+-----+
```

### 控制字符

- `SOH` (0x01): 数据包开始
- `EOT` (0x04): 传输结束
- `ACK` (0x06): 确认
- `NAK` (0x15): 错误请求重传
- `CAN` (0x18): 取消传输
- `CRC` (0x43): CRC模式

### 传输参数

- 包大小: 128 字节
- 波特率: 3000000 (升级模式)
- 超时: 1000ms
- 最大重试: 10次
- CRC: CRC-16

## 状态机集成

### tt_module 状态更新

```c
typedef enum {
    TT_STATE_UNINITIALIZED = 0,
    TT_STATE_AT_COMMAND = 1,
    TT_STATE_WAITING_MUX_RESP = 2,
    TT_STATE_MUX_MODE = 3,
    TT_STATE_OTA_UPDATE = 4,       // OTA升级中
    TT_STATE_ERROR = 5
} tt_state_t;
```

### OTA期间的状态处理

OTA开始前，需要停止当前模式：
- **MUX模式**: 停止GSM0710 MUX管理器，切换到AT命令模式
- **AT命令模式**: 无需操作，直接准备OTA

OTA完成后，天通模块自动重启并走正常流程：
```
天通模块重启 → AT命令模式 → 检测SIMST:1 → 自动切换MUX模式
```

**ESP32端无需手动恢复状态**，系统会自动处理：
- SIMST检测任务监听 `SIMST:1` 通知
- 检测到后自动发送AT命令进入MUX模式
- 正常数据通信恢复

## 内存优化

使用流式读取避免一次性加载整个固件：

```c
// 每次只读取128字节
uint8_t *data_buffer = malloc(XMODEM_PACKET_SIZE);  // 128 bytes
esp_partition_read(partition, offset, data_buffer, chunk_size);
xmodem_send_packet(data_buffer, XMODEM_PACKET_SIZE, packet_num);
free(data_buffer);
```

## 错误处理

### 传输错误
- NAK 接收: 自动重传（最多10次）
- 超时: 重传当前包
- 达到最大重试次数: 放弃并返回错误

### 模块错误
- 模块无响应: 硬件复位
- CRC校验失败: 重新传输
- 固件损坏: 返回失败状态

## 日志输出

```
I (1234) TT_MODULE_OTA: Found ota_tt partition, size: 2097152 bytes
I (1245) TT_MODULE_OTA: Step 1: Preparing for update...
I (1246) TT_MODULE_OTA: Current TT module state: 3 (MUX_MODE)
I (1247) TT_MODULE_OTA: Stopping MUX mode for OTA...
I (1256) TT_MODULE_OTA: Step 2: Resetting modem...
I (1267) TT_MODULE_OTA: Step 3: Sending AT+UPDATE command...
I (1278) TT_MODULE_OTA: Step 4: Changing baudrate to 3000000...
I (1289) TT_MODULE_OTA: Step 6: Uploading firmware via XMODEM...
I (1290) TT_MODULE_OTA: Upload progress: 10% (209715/2097152 bytes)
I (1291) TT_MODULE_OTA: Upload progress: 20% (419430/2097152 bytes)
...
I (1310) TT_MODULE_OTA: Upload progress: 100% (2097152/2097152 bytes)
I (1311) TT_MODULE_OTA: Firmware upload completed, total bytes: 2097152
I (1312) TT_MODULE_OTA: OTA update completed successfully!
I (1313) TT_MODULE_OTA: TT module will reboot and go through normal boot process (AT mode -> SIMST -> MUX mode)
```

## 测试流程

### 1. 准备测试固件
- 准备天通模块固件 bin 文件
- 计算 CRC32 校验和

### 2. 通过BLE上传
- 连接BLE
- 发送 OTA_CMD_START_TT 命令
- 传输固件数据

### 3. 观察日志
- 监控 ESP32 日志输出
- 确认 XMODEM 传输进度
- 验证最终状态

### 4. 验证结果
- 天通模块自动重启
- 运行新固件
- 功能验证

## 与MCU OTA的区别

| 特性 | MCU OTA | TT Module OTA |
|------|---------|---------------|
| 分区 | ota_mcu | ota_tt |
| 应用方式 | ESP重启后生效 | XMODEM实时传输 |
| 协议 | BLE GATT | XMODEM over UART |
| 波特率 | - | 3000000 |
| 验证 | CRC32 | CRC16 + ACK |
| 状态恢复 | 自动重启 | 恢复MUX模式 |

## 后续优化

1. **断点续传**: 记录传输位置，支持从中断处继续
2. **多文件支持**: 支持打包多个文件一起传输
3. **版本管理**: 检查天通模块当前固件版本
4. **回滚机制**: 保留旧固件，支持回滚
5. **差分升级**: 仅传输差异部分
