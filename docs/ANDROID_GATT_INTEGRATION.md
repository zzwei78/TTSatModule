# Android BLE GATT 对接文档

## 1. 概述

本文档说明 Android 应用如何与 ESP32-S3 设备的 BLE GATT 服务进行通信。

**设备名称**: `TTCat`

**主要服务**:
- System Service (0xABFC) - 系统控制和管理（核心服务，常驻）
- AT Service (0xABF2) - AT 命令透传（核心服务，常驻）
- Voice Service (0xABF0) - 语音数据传输（可选服务，动态启动）
- LOG Service (0xABF4) - 日志服务（可选服务，动态启动）
- OTA Service (0xABF8) - 固件升级服务（可选服务，动态启动）

---

## 2. GATT 服务 UUID 列表

| 服务名称 | Service UUID | 说明 | 默认状态 |
|---------|-------------|------|---------|
| System Service | `0xABFC` | 系统控制服务 | ✅ 常驻 |
| AT Service | `0xABF2` | AT 命令透传 | ✅ 常驻 |
| Voice Service | `0xABF0` | 语音数据传输 | ⚪ 动态 |
| LOG Service | `0xABF4` | 日志服务 | ⚪ 动态 |
| OTA Service | `0xABF8` | 固件升级 | ⚪ 动态 |

**UUID 格式**: 16位 UUID 需要转换为 128位 UUID 格式
```
0000ABFC-0000-1000-8000-00805F9B34FB  (System Service)
```

---

## 3. 核心服务详解

### 3.1 System Service (0xABFC) - 系统控制服务

**特征值**:

| 特征名称 | Characteristic UUID | 属性 | 说明 |
|---------|-------------------|------|------|
| Control | `0xABFD` | Write | 发送系统控制命令 |
| Info | `0xABFE` | Read | 读取系统信息 |
| Status | `0xABFF` | Notify | 系统状态通知 |

#### 3.1.1 Control 特征值 (0xABFD) - Write

**命令格式**:

```
[CMD] [DATA...]
```

| 字节 | 偏移 | 说明 |
|-----|------|------|
| 1 | 0 | 命令码 (CMD) |
| N | 1+ | 命令参数 (DATA，可选) |

**命令列表**:

##### 1. 获取电池信息 (0x01)
```
CMD: 0x01
DATA: (无)
响应: 通过 Status 特征通知返回
```

**响应格式** (19字节):
```
[电压mV(2)] [电流mA(2)] [电量%(2)] [健康度%(2)] [温度0.1K(2)]
[满充容量mAh(2)] [剩余容量mAh(2)] [充电中(1)] [已充满(1)]
```

##### 2. 获取充电状态 (0x02)
```
CMD: 0x02
DATA: (无)
```

**响应格式** (8字节):
```
[充电中(1)] [已充满(1)] [充电电压mV(2)] [充电电流mA(2)] [充满时间分钟(2)]
```

##### 3. 获取天通信号 (0x03)
```
CMD: 0x03
DATA: (无)
```

**响应格式** (4字节):
```
[RSSI(1)] [注册状态(1)] [SIM状态(1)] [网络类型(1)]
```

##### 4. 获取 BLE 发射功率 (0x04)
```
CMD: 0x04
DATA: (无)
```

**响应**: 返回当前发射功率值 (dBm)

##### 5. 设置 BLE 发射功率 (0x05)
```
CMD: 0x05
DATA: [功率值(1)]
范围: -20 ~ +20 dBm
```

##### 6. 启动服务 (0x10)
```
CMD: 0x10
DATA: [服务ID(1)]

服务ID:
  0x01 - OTA Service
  0x02 - LOG Service
  0x03 - AT Service (常驻，不可控制)
  0x04 - System Service (常驻，不可控制)
  0x05 - Voice Service
```

##### 7. 停止服务 (0x11)
```
CMD: 0x11
DATA: [服务ID(1)]
```

##### 8. 获取服务状态 (0x12)
```
CMD: 0x12
DATA: [服务ID(1)]

响应: 0=停止, 1=运行
```

##### 9. 系统重启 (0x20)
```
CMD: 0x20
DATA: (无)
```

##### 10. 恢复出厂设置 (0x21)
```
CMD: 0x21
DATA: (无)
```

##### 11. 获取系统信息 (0x30)
```
CMD: 0x30
DATA: (无)
```

**响应格式** (14字节):
```
[运行时间秒(4)] [空闲堆内存(4)] [最小空闲堆(4)] [CPU频率MHz(1)] [服务状态位(1)]
```

服务状态位:
- bit 0: OTA Service
- bit 1: LOG Service
- bit 2: AT Service
- bit 3: System Service
- bit 4: Voice Service

#### 3.1.2 Info 特征值 (0xABFE) - Read

**响应格式**: 与 "获取系统信息" 命令相同

#### 3.1.3 Status 特征值 (0xABFF) - Notify

**用途**: 接收系统异步事件和命令响应

**启用通知**: 需要先调用 `setNotifyNotification(true)`

---

### 3.2 AT Service (0xABF2) - AT 命令透传服务

**特征值**:

| 特征名称 | Characteristic UUID | 属性 | 说明 |
|---------|-------------------|------|------|
| AT Data | `0xABF3` | Write + Notify | AT 命令发送和响应 |

#### 3.2.1 AT Data 特征值 (0xABF3) - Write

**发送 AT 命令**:

```
格式: "AT<命令>\r\n"
示例: "AT\r\n"
      "AT+CPIN?\r\n"
      "AT+COPS?\r\n"
```

**注意事项**:
- 字符串格式，以 `\r\n` 结尾
- 设备会通过 Notify 返回响应

#### 3.2.2 AT Data 特征值 (0xABF3) - Notify

**接收 AT 响应**:

```
格式: "<响应数据>\r\n"
示例: "OK\r\n"
      "+CPIN: READY\r\n"
```

**响应内容**: 天通模块返回的原始响应

---

### 3.3 Voice Service (0xABF0) - 语音数据传输服务

**特征值**:

| 特征名称 | Characteristic UUID | 属性 | 说明 |
|---------|-------------------|------|------|
| Voice Data | `0xABF1` | Write + Notify | 语音数据收发 |

#### 3.3.1 Voice Data 特征值 (0xABF1) - Write

**发送语音数据到设备 (下行)**:

```
格式: "AT^AUDPCM=\"<Base64数据>\""
```

**数据流程**:
1. Android 端采集 PCM 音频 (8kHz, 16bit, mono)
2. 每 20ms = 320 字节 PCM
3. AMR-NB 编码 → 约 32 字节
4. Base64 编码 → 约 43 字节
5. 包装成 `AT^AUDPCM` 命令发送

**Java 示例**:
```java
// 1. 获取 PCM 数据 (20ms)
byte[] pcmData = new byte[320]; // 8kHz * 16bit * 0.02s
// ... 填充音频数据 ...

// 2. AMR-NB 编码
byte[] amrnbData = amrnbEncode(pcmData);

// 3. Base64 编码
String base64Data = Base64.encodeToString(amrnbData, Base64.NO_WRAP);

// 4. 包装命令
String command = String.format("AT^AUDPCM=\"%s\"", base64Data);

// 5. 发送
characteristic.setValue(command.getBytes());
gatt.writeCharacteristic(characteristic);
```

#### 3.3.2 Voice Data 特征值 (0xABF1) - Notify

**接收设备语音数据 (上行)**:

```
格式: "AT^AUDPCM=\"<Base64数据>\""
```

**数据流程**:
1. 设备接收天通网络语音
2. AMR-NB 编码 → Base64 编码
3. 通过 Notify 发送到 Android
4. Android 解码并播放

**Java 示例**:
```java
@Override
public void onCharacteristicChanged(BluetoothGatt gatt,
                                   BluetoothGattCharacteristic characteristic) {
    String data = characteristic.getStringValue(0);

    // 解析 AT^AUDPCM 命令
    if (data.startsWith("AT^AUDPCM=\"")) {
        String base64Data = data.substring(11, data.length() - 1);

        // Base64 解码
        byte[] amrnbData = Base64.decode(base64Data, Base64.NO_WRAP);

        // AMR-NB 解码
        byte[] pcmData = amrnbDecode(amrnbData);

        // 播放音频
        audioTrack.write(pcmData, 0, pcmData.length);
    }
}
```

**AMR-NB 参数**:
- 采样率: 8000 Hz
- 声道: 单声道 (mono)
- 位深: 16 bit
- 帧长: 20 ms
- 帧大小: 320 字节 PCM → 32 字节 AMR-NB

---

### 3.4 LOG Service (0xABF4) - 日志服务

**特征值**:

| 特征名称 | Characteristic UUID | 属性 | 说明 |
|---------|-------------------|------|------|
| Log Data | `0xABF5` | Write + Notify | 日志配置和数据 |

#### 3.4.1 Log Data 特征值 (0xABF5) - Write

**配置日志**:
```
[CMD] [LEVEL] [MODULE_MASK...]

CMD:
  0x01 - 设置全局日志级别
  0x02 - 设置模块日志级别

LEVEL:
  0x00 - None
  0x01 - Error
  0x02 - Warning
  0x03 - Info
  0x04 - Debug
  0x05 - Verbose

MODULE_MASK (可选): 模块掩码 (bitmask)
```

**示例**:
```
设置全局为 Debug 级别: [01 04]
设置 WiFi 模块为 Error:    [02 01 00 00 00 10]
```

#### 3.4.2 Log Data 特征值 (0xABF5) - Notify

**接收日志**:
```
[TIMESTAMP(4)] [LEVEL(1)] [TAG长度(1)] [TAG字符串] [LOG字符串]
```

**日志级别**:
```
0x01 - E (Error)
0x02 - W (Warning)
0x03 - I (Info)
0x04 - D (Debug)
0x05 - V (Verbose)
```

---

### 3.5 OTA Service (0xABF8) - 固件升级服务

**特征值**:

| 特征名称 | Characteristic UUID | 属性 | 说明 |
|---------|-------------------|------|------|
| OTA Control | `0xABF9` | Write + Notify | OTA 控制命令 |
| OTA Data | `0xABFA` | Write | 固件数据传输 |
| OTA Status | `0xABFB` | Read + Notify | OTA 状态 |

#### 3.5.1 OTA Control 特征值 (0xABF9) - Write

**控制命令**:
```
[CMD] [RESERVED(3)] [TOTAL_SIZE(4)] [CRC32(4)]

CMD:
  0x01 - 开始 MCU 固件升级
  0x02 - 开始天通模块升级
  0x03 - 中断升级

TOTAL_SIZE: 固件总大小 (little-endian)
CRC32: 固件 CRC32 校验和 (little-endian)
```

**响应**: 通过 Status 特征通知返回

#### 3.5.2 OTA Data 特征值 (0xABFA) - Write

**传输固件**:
```
分块传输固件数据，建议每次 128-200 字节
```

#### 3.5.3 OTA Status 特征值 (0xABFB) - Read + Notify

**状态读取**:
```
[STATUS(1)] [PROGRESS(1)]

STATUS:
  0x00 - Idle
  0x01 - Writing
  0x02 - Verifying
  0x03 - Success
  0x04 - Failed

PROGRESS: 进度百分比 (0-100)
```

---

## 4. Android 实现指南

### 4.1 连接设备

```java
// UUID 定义
private static final UUID SYSTEM_SERVICE_UUID =
    UUID.fromString("0000ABFC-0000-1000-8000-00805F9B34FB");
private static final UUID AT_SERVICE_UUID =
    UUID.fromString("0000ABF2-0000-1000-8000-00805F9B34FB");
private static final UUID VOICE_SERVICE_UUID =
    UUID.fromString("0000ABF0-0000-1000-8000-00805F9B34FB");

// 扫描设备
@Override
public void onScanResult(int callbackType, ScanResult result) {
    BluetoothDevice device = result.getDevice();
    if ("TTCat".equals(device.getName())) {
        scanCallback.stop();
        connectDevice(device);
    }
}

// 连接设备
private void connectDevice(BluetoothDevice device) {
    bluetoothGatt = device.connectGatt(this, false, gattCallback);
}
```

### 4.2 发现服务

```java
BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
    @Override
    public void onConnectionStateChange(BluetoothGatt gatt, int status,
                                       int newState) {
        if (newState == BluetoothProfile.STATE_CONNECTED) {
            gatt.discoverServices();
        }
    }

    @Override
    public void onServicesDiscovered(BluetoothGatt gatt, int status) {
        if (status == BluetoothGatt.GATT_SUCCESS) {
            // 获取服务
            BluetoothGattService systemService =
                gatt.getService(SYSTEM_SERVICE_UUID);
            BluetoothGattService atService =
                gatt.getService(AT_SERVICE_UUID);

            // 启用通知
            enableNotifications(gatt);
        }
    }
};
```

### 4.3 启用通知

```java
private void enableNotifications(BluetoothGatt gatt) {
    // System Status 通知
    BluetoothGattCharacteristic systemStatus =
        gatt.getService(SYSTEM_SERVICE_UUID)
           .getCharacteristic(UUID.fromString("0000ABFF-0000-1000-8000-00805F9B34FB"));
    gatt.setCharacteristicNotification(systemStatus, true);

    // AT Data 通知
    BluetoothGattCharacteristic atData =
        gatt.getService(AT_SERVICE_UUID)
           .getCharacteristic(UUID.fromString("0000ABF3-0000-1000-8000-00805F9B34FB"));
    gatt.setCharacteristicNotification(atData, true);
}
```

### 4.4 启动 Voice Service

```java
private void startVoiceService(BluetoothGatt gatt) {
    // Control 特征值
    BluetoothGattCharacteristic control =
        gatt.getService(SYSTEM_SERVICE_UUID)
           .getCharacteristic(UUID.fromString("0000ABFD-0000-1000-8000-00805F9B34FB"));

    // 启动 Voice Service (CMD=0x10, ServiceID=0x05)
    byte[] cmd = {0x10, 0x05};
    control.setValue(cmd);
    gatt.writeCharacteristic(control);
}

@Override
public void onCharacteristicWrite(BluetoothGatt gatt,
                                 BluetoothGattCharacteristic characteristic,
                                 int status) {
    if (status == BluetoothGatt.GATT_SUCCESS) {
        // 写入成功，现在可以发现 Voice Service
        gatt.discoverServices();
    }
}
```

### 4.5 发送 AT 命令

```java
private void sendAtCommand(BluetoothGatt gatt, String command) {
    BluetoothGattCharacteristic atData =
        gatt.getService(AT_SERVICE_UUID)
           .getCharacteristic(UUID.fromString("0000ABF3-0000-1000-8000-00805F9B34FB"));

    // 添加 \r\n 结尾
    String cmd = command + "\r\n";
    atData.setValue(cmd.getBytes());
    gatt.writeCharacteristic(atData);
}
```

### 4.6 读取电池信息

```java
private void getBatteryInfo(BluetoothGatt gatt) {
    BluetoothGattCharacteristic control =
        gatt.getService(SYSTEM_SERVICE_UUID)
           .getCharacteristic(UUID.fromString("0000ABFD-0000-1000-8000-00805F9B34FB"));

    // CMD = 0x01
    control.setValue(new byte[]{0x01});
    gatt.writeCharacteristic(control);
}

// 在 onCharacteristicChanged 中接收响应
@Override
public void onCharacteristicChanged(BluetoothGatt gatt,
                                   BluetoothGattCharacteristic characteristic) {
    byte[] data = characteristic.getValue();

    // 解析电池信息
    int voltage = (data[0] & 0xFF) | ((data[1] & 0xFF) << 8);
    int current = (data[2] & 0xFF) | ((data[3] & 0xFF) << 8);
    int soc = (data[4] & 0xFF) | ((data[5] & 0xFF) << 8);
    boolean charging = data[16] != 0;

    Log.i(TAG, String.format("电压: %dmV, 电量: %d%%, 充电: %b",
                            voltage, soc, charging));
}
```

### 4.7 语音数据传输

```java
// 音频参数
private static final int SAMPLE_RATE = 8000;
private static final int PCM_FRAME_SIZE = 320; // 20ms @ 8kHz 16bit
private AudioRecord audioRecord;
private AudioTrack audioTrack;

// 初始化录音
private void initAudioRecorder() {
    int minBufferSize = AudioRecord.getMinBufferSize(
        SAMPLE_RATE,
        AudioFormat.CHANNEL_IN_MONO,
        AudioFormat.ENCODING_PCM_16BIT
    );

    audioRecord = new AudioRecord(
        MediaRecorder.AudioSource.MIC,
        SAMPLE_RATE,
        AudioFormat.CHANNEL_IN_MONO,
        AudioFormat.ENCODING_PCM_16BIT,
        minBufferSize * 2
    );
}

// 发送语音
private void sendVoiceData(BluetoothGatt gatt) {
    byte[] pcmBuffer = new byte[PCM_FRAME_SIZE];

    audioRecord.startRecording();

    while (isRecording) {
        // 读取 20ms PCM
        int read = audioRecord.read(pcmBuffer, 0, PCM_FRAME_SIZE);

        if (read == PCM_FRAME_SIZE) {
            // AMR-NB 编码
            byte[] amrnbData = amrnbEncoder.encode(pcmBuffer);

            // Base64 编码
            String base64 = Base64.encodeToString(amrnbData, Base64.NO_WRAP);

            // 包装命令
            String cmd = "AT^AUDPCM=\"" + base64 + "\"";

            // 发送
            BluetoothGattCharacteristic voiceData =
                gatt.getService(VOICE_SERVICE_UUID)
                   .getCharacteristic(UUID.fromString("0000ABF1-0000-1000-8000-00805F9B34FB"));
            voiceData.setValue(cmd.getBytes());
            gatt.writeCharacteristic(voiceData);
        }
    }
}

// 接收语音
@Override
public void onCharacteristicChanged(BluetoothGatt gatt,
                                   BluetoothGattCharacteristic characteristic) {
    String data = characteristic.getStringValue(0);

    if (data.startsWith("AT^AUDPCM=\"")) {
        String base64 = data.substring(11, data.length() - 1);
        byte[] amrnbData = Base64.decode(base64, Base64.NO_WRAP);

        // AMR-NB 解码
        byte[] pcmData = amrnbDecoder.decode(amrnbData);

        // 播放
        audioTrack.write(pcmData, 0, pcmData.length);
    }
}
```

---

## 5. 完整通信流程示例

### 5.1 启动流程

```
1. 扫描并连接 "TTCat"
2. 发现服务
   - System Service (0xABFC) - 自动发现
   - AT Service (0xABF2) - 自动发现
3. 启用通知
   - System Status (0xABFF)
   - AT Data (0xABF3)
4. 启动 Voice Service
   Write to 0xABFD: [0x10, 0x05]
5. 重新发现服务
   - Voice Service (0xABF0) - 现在可用
6. 启用 Voice Data 通知
   - Voice Data (0xABF1)
```

### 5.2 语音通话流程

```
Android 端                    ESP32 端                    天通网络
   |                              |                             |
   |--- AT^AUDPCM="..." ------->|                             |
   |  (下行语音)                 |                             |
   |                             |--- PCM数据 --------------->|
   |                             |  (MUX通道9)                 |
   |                             |                             |
   |                             |<-- PCM数据 ----------------|
   |                             |  (MUX通道9)                 |
   |<-- AT^AUDPCM="..." ---------|                             |
   |  (上行语音)                 |                             |
```

### 5.3 AT 命令流程

```
Android 端                    ESP32 端                    天通模块
   |                              |                             |
   |--- "AT+CPIN?\r\n" --------->|                             |
   |                             |--- "AT+CPIN?\r\n" --------->|
   |                             |                             |
   |                             |<-- "+CPIN: READY\r\n" ------|
   |<-- "+CPIN: READY\r\n" ------|                             |
   |                             |                             |
```

---

## 6. AMR-NB 编解码库

### 6.1 Android 端推荐库

**选项 1: 使用 Android 内置编解码器** (API 16+)

```java
// AMR-NB 编码器
MediaCodec amrEncoder = MediaCodec.createEncoderByType("audio/3gpp");
MediaFormat format = new MediaFormat();
format.setString(MediaFormat.KEY_MIME, "audio/3gpp");
format.setInteger(MediaFormat.KEY_SAMPLE_RATE, 8000);
format.setInteger(MediaFormat.KEY_CHANNEL_COUNT, 1);
format.setInteger(MediaFormat.KEY_BIT_RATE, 12200);
amrEncoder.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);

// AMR-NB 解码器
MediaCodec amrDecoder = MediaCodec.createDecoderByType("audio/3gpp");
```

**选项 2: 使用开源库**

```gradle
dependencies {
    implementation 'com.github.naoaki:AMR:1.0.0'
}
```

### 6.2 编码示例

```java
public class AmrnbEncoder {
    private MediaCodec encoder;

    public AmrnbEncoder() {
        try {
            encoder = MediaCodec.createEncoderByType("audio/3gpp");
            MediaFormat format = new MediaFormat();
            format.setString(MediaFormat.KEY_MIME, "audio/3gpp");
            format.setInteger(MediaFormat.KEY_SAMPLE_RATE, 8000);
            format.setInteger(MediaFormat.KEY_CHANNEL_COUNT, 1);
            format.setInteger(MediaFormat.KEY_BIT_RATE, 12200);
            format.setInteger(MediaFormat.KEY_MAX_INPUT_SIZE, 320);
            encoder.configure(format, null, null,
                            MediaCodec.CONFIGURE_FLAG_ENCODE);
            encoder.start();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public byte[] encode(byte[] pcmData) {
        // 输入 PCM
        ByteBuffer[] inputBuffers = encoder.getInputBuffers();
        int inputIndex = encoder.dequeueInputBuffer(0);
        if (inputIndex >= 0) {
            ByteBuffer inputBuffer = inputBuffers[inputIndex];
            inputBuffer.clear();
            inputBuffer.put(pcmData);
            encoder.queueInputBuffer(inputIndex, 0, pcmData.length, 0, 0);
        }

        // 输出 AMR-NB
        ByteBuffer[] outputBuffers = encoder.getOutputBuffers();
        MediaCodec.BufferInfo bufferInfo = new MediaCodec.BufferInfo();
        int outputIndex = encoder.dequeueOutputBuffer(bufferInfo, 0);

        if (outputIndex >= 0) {
            ByteBuffer outputBuffer = outputBuffers[outputIndex];
            byte[] amrData = new byte[bufferInfo.size];
            outputBuffer.position(bufferInfo.offset);
            outputBuffer.get(amrData);
            encoder.releaseOutputBuffer(outputIndex, false);
            return amrData;
        }

        return null;
    }
}
```

---

## 7. 错误处理

### 7.1 GATT 错误码

| 错误码 | 说明 | 处理方式 |
|-------|------|---------|
| 0x0000 | 成功 | - |
| 0x0101 | 系统错误 | 重试或重新连接 |
| 0x0102 | 无效命令 | 检查命令格式 |
| 0x0103 | 无效参数 | 检查参数范围 |
| 0x0201 | 服务未找到 | 确认服务已启动 |
| 0x0202 | 服务运行中 | 无需重复启动 |
| 0x0203 | 服务未运行 | 需先启动服务 |

### 7.2 常见问题

**Q: 发现不到 Voice Service**
- A: 需要先通过 System Service 的 `0x10` 命令启动

**Q: 写入特征值失败**
- A: 检查 MTU 大小，数据不能超过 MTU (默认 23 字节)

**Q: 收不到通知**
- A: 确保已调用 `setNotifyNotification(true)`

**Q: 连接断开**
- A: Android 系统会自动断开空闲连接，需要定期发送保活数据

---

## 8. 权限配置

**AndroidManifest.xml**:

```xml
<!-- BLE 权限 -->
<uses-permission android:name="android.permission.BLUETOOTH" />
<uses-permission android:name="android.permission.BLUETOOTH_ADMIN" />
<uses-permission android:name="android.permission.BLUETOOTH_SCAN" />
<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />

<!-- Android 12+ 需要 -->
<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />

<!-- 录音权限 (语音通话) -->
<uses-permission android:name="android.permission.RECORD_AUDIO" />
```

**动态权限请求**:

```java
// Android 12+
if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN)
    != PackageManager.PERMISSION_GRANTED) {
    ActivityCompat.requestPermissions(this,
        new String[]{
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.BLUETOOTH_CONNECT,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.RECORD_AUDIO
        }, 100);
}
```

---

## 9. 测试工具

### 9.1 nRF Connect (调试工具)

1. 安装 nRF Connect 应用
2. 扫描并连接 "TTCat"
3. 查看服务和特征值
4. 测试读写操作

### 9.2 日志查看

启用 LOG Service 后，可以在 nRF Connect 中查看实时日志

---

## 10. 参考资料

- [Android BLE 官方文档](https://developer.android.com/guide/topics/connectivity/bluetooth/connect/gatt-server)
- [AMR 编解码标准](https://www.etsi.org/deliver/etsi_ts/126000_126099/126073/14.00.00_60/ts_126073v140000p.pdf)
- [BLE GATT 规范](https://www.bluetooth.com/specifications/bluetooth-core-specification/)

---

## 11. 附录: UUID 速查表

```java
// System Service
public static final UUID SYSTEM_SERVICE_UUID =
    UUID.fromString("0000ABFC-0000-1000-8000-00805F9B34FB");
public static final UUID SYSTEM_CONTROL_UUID =
    UUID.fromString("0000ABFD-0000-1000-8000-00805F9B34FB");
public static final UUID SYSTEM_INFO_UUID =
    UUID.fromString("0000ABFE-0000-1000-8000-00805F9B34FB");
public static final UUID SYSTEM_STATUS_UUID =
    UUID.fromString("0000ABFF-0000-1000-8000-00805F9B34FB");

// AT Service
public static final UUID AT_SERVICE_UUID =
    UUID.fromString("0000ABF2-0000-1000-8000-00805F9B34FB");
public static final UUID AT_DATA_UUID =
    UUID.fromString("0000ABF3-0000-1000-8000-00805F9B34FB");

// Voice Service
public static final UUID VOICE_SERVICE_UUID =
    UUID.fromString("0000ABF0-0000-1000-8000-00805F9B34FB");
public static final UUID VOICE_DATA_UUID =
    UUID.fromString("0000ABF1-0000-1000-8000-00805F9B34FB");

// LOG Service
public static final UUID LOG_SERVICE_UUID =
    UUID.fromString("0000ABF4-0000-1000-8000-00805F9B34FB");
public static final UUID LOG_DATA_UUID =
    UUID.fromString("0000ABF5-0000-1000-8000-00805F9B34FB");

// OTA Service
public static final UUID OTA_SERVICE_UUID =
    UUID.fromString("0000ABF8-0000-1000-8000-00805F9B34FB");
public static final UUID OTA_CONTROL_UUID =
    UUID.fromString("0000ABF9-0000-1000-8000-00805F9B34FB");
public static final UUID OTA_DATA_UUID =
    UUID.fromString("0000ABFA-0000-1000-8000-00805F9B34FB");
public static final UUID OTA_STATUS_UUID =
    UUID.fromString("0000ABFB-0000-1000-8000-00805F9B34FB");
```

---

**文档版本**: v1.0
**更新日期**: 2025-01-16
**适用固件**: TTSatModule v1.0+
