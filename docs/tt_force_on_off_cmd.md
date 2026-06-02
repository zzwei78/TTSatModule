# 天通模块强制开关命令 (通话专用)

## 概述

拨打电话时，天通模块必须在任何状态下（包括低电量自动关机）都能强制开启。通话结束后恢复正常低电量保护。

## 新增命令

### SYS_CMD_TT_FORCE_ON (0x72) — 强制开启天通模块

拨打电话前发送，忽略低电量保护，任何状态下尝试启动天通模块。

| 项目 | 说明 |
|------|------|
| 命令码 | `0x72` |
| 参数 | 无 |
| 响应码 | `0x00`=成功, `0x01`=失败(OTA中或启动失败) |
| 响应数据 | `[state][voltage_lo][voltage_hi]` (3 bytes) |

**state 值说明：**

| 值 | 状态 | 说明 |
|----|------|------|
| 0 | HARDWARE_FAULT | 硬件故障 |
| 1 | INITIALIZING | 初始化中 |
| 2 | WAITING_MUX_RESP | 等待MUX响应 |
| 3 | LOW_BATTERY_OFF | 低电量自动关机 |
| 4 | USER_OFF | 用户手动关机 |
| 5 | WORKING | 正常工作 |
| 6 | UPDATING | OTA升级中 |

**调用后 state 变化：**

| 调用前状态 | 调用后 state | 说明 |
|-----------|-------------|------|
| WORKING(5) | WORKING(5) | 已在工作，无需操作 |
| INITIALIZING(1) | INITIALIZING(1) | 正在初始化，等待即可 |
| LOW_BATTERY_OFF(3) | INITIALIZING(1)→WORKING(5) | 重新启动 |
| USER_OFF(4) | INITIALIZING(1)→WORKING(5) | 重新启动 |
| HARDWARE_FAULT(0) | INITIALIZING(1)或FAULT(0) | 尝试恢复 |
| UPDATING(6) | UPDATING(6) | 拒绝，返回错误 |

---

### SYS_CMD_TT_FORCE_OFF (0x73) — 取消强制，恢复低电量保护

挂断电话后发送。取消强制开启状态，恢复 3.1V 低电量自动关机保护。

| 项目 | 说明 |
|------|------|
| 命令码 | `0x73` |
| 参数 | 无 |
| 响应码 | `0x00`=成功 |
| 响应数据 | `[state][voltage_lo][voltage_hi]` (3 bytes) |

**调用后行为：**

| 当前电压 | 行为 | 最终 state |
|---------|------|-----------|
| >= 3.1V | 保持运行 | WORKING(5) |
| < 3.1V | 立即关机 | LOW_BATTERY_OFF(3) |

---

## 状态查询变更

### SYS_CMD_GET_TT_STATUS (0x60) — 响应格式变更

响应 byte4 从 `reserved(0x00)` 变为 `flags`：

```
Byte 0: state          (天通模块状态)
Byte 1: voltage_lo     (电池电压低字节)
Byte 2: voltage_hi     (电池电压高字节)
Byte 3: error_code     (错误码, state=0时有效)
Byte 4: flags          (标志位)  ← 变更
Byte 5: reserved       (0x00)
```

**flags 位定义：**

| Bit | 含义 |
|-----|------|
| 0 | `force_on` — 1=强制开启中(通话模式), 0=正常模式 |
| 1-7 | 预留 |

---

## App 端显示建议

### 天通模块状态显示

建议 App 根据 state + flags 组合显示：

| state | flags.bit0 | 显示建议 |
|-------|-----------|---------|
| 5 (WORKING) | 0 | "天通模块: 在线" |
| 5 (WORKING) | 1 | "天通模块: 通话中" |
| 3 (LOW_BATTERY_OFF) | 0 | "天通模块: 低电量已关闭" |
| 4 (USER_OFF) | 0 | "天通模块: 已关闭" |
| 0 (HARDWARE_FAULT) | 0 | "天通模块: 硬件异常" |
| 1 (INITIALIZING) | 0/1 | "天通模块: 启动中..." |
| 6 (UPDATING) | 0 | "天通模块: 升级中" |

---

## 通话流程

```
App 拨打电话:
  1. 发送 0x72 (TT_FORCE_ON)
     → 响应: resp_code=0x00, data=[state, v_lo, v_hi]
     → 等待 state=5 (WORKING) 后拨号
  2. 通话中...
     → 可轮询 0x60 (GET_TT_STATUS) 检查状态
     → flags.bit0=1 表示强制模式激活中

App 挂断电话:
  1. 发送 0x73 (TT_FORCE_OFF)
     → 响应: data=[state, v_lo, v_hi]
     → 如果 state=3 (LOW_BATTERY_OFF), 提示用户电量不足
  2. 正常模式恢复

异常: BLE 断开
  → 设备端自动取消 force_on (等同于发送 0x73)
  → App 重连后查询 0x60 确认状态
```

## 注意事项

1. `0x72` 调用后模块可能需要 3-5 秒才能变为 WORKING(5)，App 应轮询等待
2. OTA 升级期间(UPDATING) 无法强制开启，resp_code=0x01
3. BLE 断开时设备端自动取消强制模式，App 无需额外处理
4. `0x60` 响应长度不变(6 bytes)，byte4 含义变更，需适配解析
