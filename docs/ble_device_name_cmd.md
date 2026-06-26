# BLE 设备名设置命令 — 开发文档

## 概述

命令 ID: `0x7C` (`SYS_CMD_SET_DEVICE_NAME`) / `0x7D` (`SYS_CMD_GET_DEVICE_NAME`)

功能：APP 动态修改 BLE 广播设备名（支持中文/UTF-8），持久化到 NVS，重启后仍生效。

- 默认设备名: `TTCat`
- 最大长度: 29 字节（ASCII 29 字符；UTF-8 中文约 9 个汉字）
- 空参数恢复默认名 `TTCat` 并擦除 NVS 记录
- 修改后需 **断开重连 / 重新扫描** 才能看到新名字
- NVS 在普通烧录下保留；`erase-flash` 才会清空

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
1     1     cmd         命令码 (0x7C 或 0x7D)
2     1     param_len   参数长度
3     N     params[]    参数数据 (N = param_len)
3+N   2     crc16       CRC16-CCITT 校验 (多项式 0x1021, 初始值 0x0000)
```

CRC 计算范围: 从 `seq` 到 `params[]` 的最后一个字节 (不含 `crc16` 字段本身)。

#### 响应包 (Device → App)

```
偏移  长度  字段         说明
0     1     seq         序列号 (回显命令的 seq)
1     1     cmd         命令码 (回显 0x7C 或 0x7D)
2     1     resp_code   响应码 (见下表)
3     1     data_len    数据长度
4     N     data[]      响应数据
4+N   2     crc16       CRC16-CCITT 校验
```

#### 响应码

| 值 | 常量 | 含义 |
|----|------|------|
| 0x00 | `SYS_RESP_OK` | 成功 |
| 0x01 | `SYS_RESP_ERROR` | 内部错误 (NVS 写入失败、GAP 设置失败等) |
| 0x03 | `SYS_RESP_INVALID_PARAM` | 参数无效 (名字过长 > 29 字节，或包含内嵌 NUL) |

## NVS 存储

| 项目 | 值 |
|------|-----|
| Namespace | `ble_config` |
| Key | `dev_name` |
| 类型 | blob (原始 UTF-8 字节，不含终止符) |
| 恢复默认 | 删除该 key (`nvs_erase_key`) |

> 独立于 `user_params` 结构，不影响其他配置。

## 命令 0x7C 详情 (SET_DEVICE_NAME)

### 请求参数

| param_len | 含义 |
|-----------|------|
| 0 | 恢复默认名 `TTCat`，并擦除 NVS 记录 |
| 1 ~ 29 | 设置自定义名 (UTF-8 字节) |
| > 29 | 返回 `SYS_RESP_INVALID_PARAM` |

`params[]` 即 UTF-8 编码的设备名字节，不需要 null 终止符。

> 字节序列中不允许内嵌 `0x00` (否则返回 `INVALID_PARAM`)；UTF-8 多字节字符 (≥0x80) 是合法的。

### 响应数据 (成功时)

| 偏移 | 长度 | 说明 |
|------|------|------|
| 0 | 1 | 当前设备名字节长度 `name_len` |
| 1 | name_len | 当前设备名 UTF-8 字节 (应用后的实际值) |

无论请求是"设置新名"还是"恢复默认"，响应都返回**应用后**的当前设备名。

## 命令 0x7D 详情 (GET_DEVICE_NAME)

### 请求参数

无 (param_len = 0)。

### 响应数据 (成功时)

| 偏移 | 长度 | 说明 |
|------|------|------|
| 0 | 1 | 当前设备名字节长度 `name_len` |
| 1 | name_len | 当前设备名 UTF-8 字节 |

## 交互流程

### 设置新名字

```
App                              Device
  |                                 |
  |--- Write (0x7C, "天通") ------>|
  |                                 |  校验 → 写 NVS → 更新 GAP 名
  |                                 |  → 停广播 → 重启广播
  |<-- Notify (response) ----------|
  |                                 |
  |   (断开当前连接)                  |
  |   (重新扫描)                      |
  |--- 发现新名 "天通" ------------>|
  |                                 |
```

### 查询当前名字

```
App                              Device
  |                                 |
  |--- Write (0x7D, 无参数) ------->|
  |<-- Notify (response) ----------|
  |                                 |
```

## 示例

> 字符串 → UTF-8 字节示例：
> - `"TTCat"` = `54 54 43 61 74` (5 字节)
> - `"天通"` = `E5 A4 A9 E9 80 9A` (6 字节，每字 3 字节)
> - `"卫星电话"` = `E5 8D AB E6 98 9F E7 94 B5 E8 AF 9D` (12 字节)

### 示例 1: 设置为 "天通"

请求:
```
seq:       0x01
cmd:       0x7C
param_len: 0x06
params:    [0xE5, 0xA4, 0xA9, 0xE9, 0x80, 0x9A]    "天通" UTF-8
crc16:     XX XX
```

响应:
```
seq:       0x01
cmd:       0x7C
resp_code: 0x00 (OK)
data_len:  0x07
data:      [0x06, 0xE5, 0xA4, 0xA9, 0xE9, 0x80, 0x9A]
            │len  └────── "天通" (6 字节) ──────┘
crc16:     XX XX
```

### 示例 2: 恢复默认名 "TTCat" (传空参数)

请求:
```
seq:       0x02
cmd:       0x7C
param_len: 0x00
params:    []
crc16:     XX XX
```

响应:
```
seq:       0x02
cmd:       0x7C
resp_code: 0x00 (OK)
data_len:  0x06
data:      [0x05, 0x54, 0x54, 0x43, 0x61, 0x74]
            │len  └── "TTCat" (5 字节) ──┘
crc16:     XX XX
```

### 示例 3: 查询当前名字

请求:
```
seq:       0x03
cmd:       0x7D
param_len: 0x00
params:    []
crc16:     XX XX
```

响应 (假设当前名为 "天通"):
```
seq:       0x03
cmd:       0x7D
resp_code: 0x00 (OK)
data_len:  0x07
data:      [0x06, 0xE5, 0xA4, 0xA9, 0xE9, 0x80, 0x9A]
crc16:     XX XX
```

### 示例 4: 名字过长 (> 29 字节)

请求 `params` 为 30 字节的 ASCII:
```
cmd:       0x7C
param_len: 0x1E  (30)
params:    [30 个 'A']
```

响应:
```
resp_code: 0x03 (INVALID_PARAM)
data_len:  0x00
```

### 示例 5: 含内嵌 NUL 的非法输入

请求 `params = [0xE5, 0xA4, 0x00, 0xA9]` (中间出现 0x00):
```
resp_code: 0x03 (INVALID_PARAM)
data_len:  0x00
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

# 示例: 设置设备名为 "天通"
name = "天通".encode("utf-8")     # b'\xe5\xa4\xa9\xe9\x80\x9a'
cmd_packet = build_command(seq=1, cmd=0x7C, params=name)
print(cmd_packet.hex())
# 输出: 01 7c 06 e5 a4 a9 e9 80 9a XX XX

# 示例: 恢复默认名 (空参数)
cmd_packet = build_command(seq=2, cmd=0x7C, params=b"")
print(cmd_packet.hex())
# 输出: 02 7c 00 XX XX

# 示例: 查询当前名 (0x7D, 空参数)
cmd_packet = build_command(seq=3, cmd=0x7D, params=b"")
print(cmd_packet.hex())
# 输出: 03 7d 00 XX XX

# 响应解析
def parse_response(raw: bytes) -> dict:
    seq, cmd, resp_code, data_len = raw[:4]
    data = raw[4:4 + data_len]
    name_len = data[0]
    name = data[1:1 + name_len].decode("utf-8")
    return {"seq": seq, "cmd": cmd, "code": resp_code, "name": name}
```

## 实现细节

### 设备端 API

声明在 `main/ble/ble_gatt_server.h`:

```c
#define BLE_DEVICE_NAME_DEFAULT    "TTCat"
#define BLE_DEVICE_NAME_MAX_LEN    29

void ble_device_name_init(void);                              // 开机从 NVS 读取
esp_err_t ble_device_name_set(const char *name, uint8_t len); // 设置 + 存 NVS + 重启广播
const char *ble_device_name_get(void);                        // 获取当前名字
```

### 调用位置

| 函数 | 调用位置 | 说明 |
|------|---------|------|
| `ble_device_name_init()` | `ble_gatt_server_init()` (替代原 `ble_svc_gap_device_name_set("TTCat")`) | 开机加载持久化名字 |
| `ble_device_name_set()` | `gatt_system_server.c` 中 `SYS_CMD_SET_DEVICE_NAME` 分支 | 处理 0x7C 命令 |
| `ble_device_name_get()` | `gatt_system_server.c` 中 `SYS_CMD_GET_DEVICE_NAME` 分支 | 处理 0x7D 命令 |

### 广播重启

`ble_device_name_set()` 在写入 NVS 和更新 GAP 名后，调用：

1. `ble_gap_adv_stop()` — 停止当前广播 (未在广播时返回 `BLE_HS_EALREADY` / `BLE_HS_ENOENT`，已忽略)
2. `ble_spp_server_advertise()` — 用新名字重新开始广播

> NimBLE 的 `ble_spp_server_advertise()` 在内部读取 `ble_svc_gap_device_name()` 作为广播名，因此 GAP 名更新后即可生效。

### 权限

两个命令均在 `DEBUG_ALLOWED_COMMANDS` 白名单内，调试连接也可执行。命令为非破坏性 (可逆)，未列入 `is_dangerous_command()`。

## 测试场景

| 场景 | 操作 | 预期结果 |
|------|------|---------|
| 默认名 | 烧录后首次扫描 | 设备显示为 `TTCat` |
| 设置中文名 | SET "天通" → 断开重连 | 显示为 "天通" |
| 持久化 | SET "天通" → 重启设备 | 仍显示 "天通" (NVS 保留) |
| 恢复默认 | SET "" (空参数) → 断开重连 | 恢复为 `TTCat` |
| 查询名字 | GET | 返回当前名字 |
| 长度边界 (上限) | SET 29 字节 ASCII | 成功 |
| 长度超限 | SET 30 字节 | `resp_code=0x03` |
| 中文边界 | SET 9 个汉字 (27 字节) | 成功 |
| 中文超限 | SET 10 个汉字 (30 字节) | `resp_code=0x03` |
| 内嵌 NUL | SET `[0xE5, 0x00, 0xA9]` | `resp_code=0x03` |
| 恢复默认后查询 | SET "" → GET | 返回 `TTCat` (5 字节) |
| 普通烧录不擦除 | SET "TestName" → 重新烧录固件 | 名字仍为 "TestName" |
| erase-flash | SET "TestName" → `idf.py erase-flash` → 烧录 | 恢复为 `TTCat` |
