# 电话功能全场景测试用例

## 文档版本 & 覆盖范围

覆盖模拟通话 (sat_call_sim)、语音服务 (spp_voice_server)、BLE 连接管理、TT 模块通话控制、电源管理等所有电话相关功能。

---

## 一、测试环境准备

### 1.1 前置条件

- 设备固件已烧录，包含模拟通话模块 (`ENABLE_SAT_CALL_SIM=1`)
- 手机安装测试 APK，支持 BLE 连接、AT 命令发送、系统命令
- 设备与手机正常 BLE 配对连接
- （可选）TT 模块可用（真实通话测试）

### 1.2 系统命令速查

| 命令 | ID | 参数 | 用途 |
|------|----|------|------|
| SERVICE_START | 0x10 | `[service_id]` | 启用服务 (VOICE=5, SPP=4) |
| SERVICE_STOP | 0x11 | `[service_id]` | 停用服务 |
| SIM_CTRL | 0x77 | `[enable, scenario]` | 开启/关闭模拟 (scenario: 0=NORMAL, 1=BUSY, 2=NO_ANSWER, 3=NET_DROP, 5=REJECT) |
| SIM_INCOMING | 0x78 | `[delay_ms_le32]` | 触发模拟来电 (0=立即) |
| SIM_SET_NET | 0x79 | `[csq, creg]` | 设置模拟网络参数 |
| SIM_GET_STATE | 0x7A | 无 | 查询模拟状态 |
| SET_VOICE_FRAME_MODE | 0x70 | `[mode]` | 设置语音帧模式 (1=Android, 3=iPhone) |
| GET_VOICE_FRAME_MODE | 0x71 | 无 | 查询帧模式 |
| TT_FORCE_ON | 0x72 | 无 | 强制开启 TT 模块 |
| TT_FORCE_OFF | 0x73 | 无 | 取消强制开启 |
| BLE_TX_POWER_TEMP | 0x7B | `[power_dbm, timeout_sec_le32]` | 临时设置 BLE 发射功率 |

### 1.3 模拟通话状态

| 状态 | 值 | 含义 |
|------|---|------|
| IDLE | 0 | 空闲 |
| DIALING | 1 | 拨号中 |
| ALERTING | 2 | 对方振铃 |
| ACTIVE | 3 | 通话中 |
| INCOMING | 4 | 来电振铃 |
| DISCONNECTING | 5 | 挂断中 |

### 1.4 模拟场景

| 场景 | 值 | 行为 |
|------|---|------|
| NORMAL | 0 | ATD → 3s DIALING → 5s ALERTING → 5s ACTIVE → 持续通话 |
| BUSY | 1 | ATD → 3s DIALING → 5s ALERTING → ^CEND(103,17) 忙音 |
| NO_ANSWER | 2 | ATD → 3s DIALING → 5s ALERTING → 30s 无应答 → ^CEND(102,19) |
| NETWORK_DROP | 3 | ATD → 正常接通 → 5~15s 随机掉话 → ^CEND(101,3) |
| REJECT | 5 | ATD → 3s DIALING → 5s ALERTING → 3s 拒接 → ^CEND(105,17) |

### 1.5 关键定时参数

| 参数 | 值 | 说明 |
|------|---|------|
| DIALING 持续 | 3s | ATD 后到 ^ORIG |
| ALERTING 持续 | 5s | ^CONF 后到 ^CONN |
| ACTIVE 到语音 | 5s | ^CONN 后到语音注入开始 |
| RING 间隔 | 2s | 来电振铃间隔 |
| INCOMING 超时 | 30s | 来电不接听自动挂断 |
| BLE 保持超时 | 30s | 信号丢失后通话保持 |
| 语音空闲超时 | 30s | 无数据自动关闭语音服务 |
| NET_DROP | 5~15s | 随机掉话 |

### 1.6 BLE 断开 reason 编码

NimBLE 编码: `0x200 + HCI 错误码`

| 触发方式 | reason (十进制) | HCI 码 | 分类 |
|---------|----------------|--------|------|
| 走出范围 / 信号差 | 520 | 0x08 SPVN_TMO | 信号丢失 |
| LMP 超时 | 546 | 0x22 LMP_RSP_TMO | 信号丢失 |
| 手机关蓝牙 | 531 | 0x13 REM_USER_TERM | 用户主动 |
| 手机飞行模式 | 531 | 0x13 REM_USER_TERM | 用户主动 |
| 设备关闭蓝牙 | 534 | 0x16 LOCAL_TERM | 本端主动 |

### 1.7 关键日志关键字

| 关键字 | 含义 |
|--------|------|
| `preserving call for 30000 ms` | 通话保持已启动 |
| `no active call, normal cleanup` | 无通话，正常清理 |
| `BLE reconnect timeout` | 30s 超时挂断 |
| `AT+CHUP sent (timeout hangup)` | 超时挂断正常通话 |
| `Sim call CHUP sent (timeout hangup)` | 超时挂断模拟通话 |
| `BLE reconnected while call preserved` | 保持期间重连 |
| `Active call on user disconnect` | 用户断开 + 立即挂断 |
| `Voice service disabled` | 语音服务已关闭 |
| `voice idle timeout` | 语音空闲超时自动关闭 |
| `sim_call_sim_handle_at_gatt` | AT 命令处理 |
| `ERROR` | AT 命令返回错误 |

---

## 二、测试用例

### A 类：正常通话流程（主路径）

---

#### TC-A01: 正常外呼完整流程 (NORMAL)

**覆盖**: 外呼通话完整生命周期

```
步骤:
1. SIM_CTRL [1, 0]     → 开启模拟 (NORMAL)
2. SERVICE_START [5]   → 开启语音服务
3. 发送 ATD10086;
4. 等待 3s → 验证收到 ^ORIG:1,0 + ^DSCI:1,0,2
5. 等待 5s → 验证收到 ^CONF:1,0 + ^DSCI:1,0,3 (ALERTING)
6. 等待 5s → 验证收到 ^CONN:1,0 + ^DSCI:1,0,0 (ACTIVE)
7. 验证语音数据开始流动 (BLE notify 收到语音帧)
8. 等待 10s → 验证语音持续
9. 发送 AT+CHUP
10. 验证收到 ^CEND:1,0,101,17,0 + ^DSCI:6
11. SIM_GET_STATE → state=0 (IDLE)

验证:
- 状态转换: IDLE → DIALING(3s) → ALERTING(5s) → ACTIVE(5s) → IDLE
- URC 序列: ^ORIG → ^CONF → ^CONN → (voice) → ^CEND
- 语音数据在 ACTIVE 后开始
- CHUP 后通话干净结束
```

---

#### TC-A02: 正常来电完整流程

**覆盖**: 来电通话完整生命周期

```
步骤:
1. SIM_CTRL [1, 0]     → 开启模拟 (NORMAL)
2. SERVICE_START [5]   → 开启语音服务
3. SIM_INCOMING [0]    → 立即触发来电
4. 验证收到 RING + ^DSCI:2,1,4
5. 等待 2s → 验证收到第 2 个 RING + ^DSCI
6. 发送 ATA
7. 验证收到 OK + ^CONN:2,0 + ^DSCI:2,1,0 (ACTIVE)
8. 验证语音数据开始流动
9. 等待 10s → 验证语音持续
10. 发送 AT+CHUP
11. 验证 ^CEND + IDLE

验证:
- 状态: IDLE → INCOMING → ACTIVE → IDLE
- RING 间隔 = 2s
- ATA 后立即接通
- call_id = 2 (MT call)
```

---

#### TC-A03: 延迟触发来电

**覆盖**: SIM_INCOMING 带 delay_ms 参数

```
步骤:
1. SIM_CTRL [1, 0]
2. SIM_INCOMING [5000]  → 5 秒后来电
3. 等待 5s → 验证收到 RING
4. 发送 ATA → 验证接通

验证:
- 5s 前无 RING
- 5s 后正常来电流程
```

---

#### TC-A04: 先拨号后开语音

**覆盖**: 语音服务在通话建立后才启用

```
步骤:
1. SIM_CTRL [1, 0]
2. 发送 ATD10086;        → 不开语音
3. 等待 ACTIVE (^CONN)
4. SERVICE_START [5]     → 通话中开语音
5. 验证语音数据开始

验证:
- ACTIVE 前无语音数据
- 开启语音后立即注入
```

---

#### TC-A05: 语音先于拨号开启

**覆盖**: 语音服务先启用，等 ACTIVE 后自动注入

```
步骤:
1. SIM_CTRL [1, 0]
2. SERVICE_START [5]     → 先开语音
3. 发送 ATD10086;
4. DIALING/ALERTING 阶段 → 验证无语音注入 (或仅有 ringback)
5. ACTIVE 后 → 验证语音自动开始注入

验证:
- sat_call_sim_voice_enabled() 在 ACTIVE 时自动启动注入
- ALERTING 阶段只有回铃音 (440+480Hz)，无通话语音
```

---

#### TC-A06: 来电不接听等待超时

**覆盖**: 来电 30s 不接听 → 自动挂断

```
步骤:
1. SIM_CTRL [1, 0]
2. SIM_INCOMING [0]     → 触发来电
3. 不发 ATA，等待 30s
4. 验证每 2s 收到 RING + ^DSCI (约 15 次)
5. 30s 后验证收到 ^CEND:2,1,102,19,0 + ^DSCI:6

验证:
- RING 间隔稳定 2s
- 30s 后自动结束 (cause 102,19 = no answer)
- 状态回到 IDLE
```

---

### B 类：通话场景变体 (BUSY / NO_ANSWER / REJECT / NETWORK_DROP)

---

#### TC-B01: BUSY 场景 — 对方忙

**覆盖**: 拨号后对方忙线

```
步骤:
1. SIM_CTRL [1, 1]     → 开启 BUSY 场景
2. SERVICE_START [5]
3. 发送 ATD10086;
4. 等待 3s → DIALING → ^ORIG
5. 等待 5s → ALERTING → ^CONF (听到回铃音)
6. 等待 5s → 验证 ^CEND:1,0,103,17,0 + ^DSCI:6
7. SIM_GET_STATE → IDLE

验证:
- ALERTING 后 5s 收到 BUSY 挂断
- cause=103,17 (busy)
- 语音停止
```

---

#### TC-B02: NO_ANSWER 场景 — 对方不接

**覆盖**: 拨号后对方 30s 不接

```
步骤:
1. SIM_CTRL [1, 2]     → 开启 NO_ANSWER 场景
2. SERVICE_START [5]
3. 发送 ATD10086;
4. 等待 DIALING(3s) → ALERTING(5s)
5. 等待 30s → 验证 ^CEND:1,0,102,19,0 + ^DSCI:6
6. SIM_GET_STATE → IDLE

验证:
- ALERTING 持续 30s (有回铃音)
- 30s 后 no answer 挂断
- cause=102,19
```

---

#### TC-B03: REJECT 场景 — 对方拒接

**覆盖**: 拨号后对方快速拒接

```
步骤:
1. SIM_CTRL [1, 5]     → 开启 REJECT 场景
2. SERVICE_START [5]
3. 发送 ATD10086;
4. 等待 DIALING(3s) → ALERTING(5s)
5. 等待 3s → 验证 ^CEND:1,0,105,17,0 + ^DSCI:6
6. SIM_GET_STATE → IDLE

验证:
- ALERTING 后 3s 快速拒接
- cause=105,17 (reject)
```

---

#### TC-B04: NETWORK_DROP 场景 — 通话中网络掉话

**覆盖**: 通话中随机网络断开

```
步骤:
1. SIM_CTRL [1, 3]     → 开启 NETWORK_DROP 场景
2. SERVICE_START [5]
3. 发送 ATD10086;
4. 等待 ACTIVE → 语音开始
5. 等待 5~15s → 验证自动收到 ^CEND:1,0,101,3,0 + ^DSCI:6
6. SIM_GET_STATE → IDLE

验证:
- 正常接通后随机 5~15s 掉话
- cause=101,3 (network drop)
- 语音停止
- 重复 3 次验证随机性 (每次掉话时间不同)
```

---

#### TC-B05: 场景切换 — 通话结束后切换场景

**覆盖**: 一次通话结束后切换到另一个场景

```
步骤:
1. SIM_CTRL [1, 0]     → NORMAL
2. ATD10086; → ACTIVE → CHUP → IDLE
3. SIM_CTRL [1, 1]     → 切换到 BUSY
4. ATD10086; → ALERTING → 验证 BUSY 挂断
5. SIM_CTRL [1, 3]     → 切换到 NETWORK_DROP
6. ATD10086; → ACTIVE → 验证随机掉话

验证:
- 场景可自由切换
- 每次通话使用当前场景
```

---

#### TC-B06: 多次连续通话 (NORMAL)

**覆盖**: 连续发起多次通话，验证状态重置

```
步骤:
1. SIM_CTRL [1, 0]
2. 循环 5 次:
   a. ATD10086; → 等待 ACTIVE
   b. 等待 3s
   c. AT+CHUP → 等待 IDLE
   d. SIM_GET_STATE → 验证 IDLE

验证:
- 每次通话独立，状态正确重置
- call_id 不变 (模拟固定为 1)
- 无内存泄漏或资源耗尽
```

---

### C 类：命令错误 & 非法操作

---

#### TC-C01: 通话中再次拨号 (ATD in ACTIVE)

**覆盖**: 活跃通话中发送 ATD

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → 等待 ACTIVE
2. 发送 ATD10086;

验证:
- 收到 ERROR (不是 OK)
- 原通话不受影响，继续 ACTIVE
- SIM_GET_STATE → ACTIVE
```

---

#### TC-C02: 通话中再次拨号 (ATD in DIALING)

**覆盖**: 拨号阶段再次发送 ATD

```
步骤:
1. SIM_CTRL [1, 0] → 发送 ATD10086;
2. 不等待，立即再发 ATD10086;

验证:
- 第二次 ATD 收到 ERROR
- 第一次拨号继续正常流程
```

---

#### TC-C03: 无来电时接听 (ATA without INCOMING)

**覆盖**: IDLE 状态发 ATA

```
步骤:
1. SIM_CTRL [1, 0]
2. 发送 ATA

验证:
- 收到 ERROR
- SIM_GET_STATE → IDLE
```

---

#### TC-C04: 空闲时挂断 (CHUP while IDLE)

**覆盖**: IDLE 状态发 CHUP

```
步骤:
1. SIM_CTRL [1, 0]
2. 发送 AT+CHUP

验证:
- 收到 ERROR
- 状态保持 IDLE
```

---

#### TC-C05: DIALING 阶段挂断 (CHUP in DIALING)

**覆盖**: 拨号刚发出就取消

```
步骤:
1. SIM_CTRL [1, 0] → 发送 ATD10086;
2. 立即发送 AT+CHUP

验证:
- 收到 OK + ^CEND + ^DSCI:6
- 状态变为 IDLE
- 不继续到 ALERTING/ACTIVE
```

---

#### TC-C06: ALERTING 阶段挂断 (CHUP in ALERTING)

**覆盖**: 振铃阶段取消拨号

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086;
2. 等待 ALERTING (^CONF 收到)
3. 发送 AT+CHUP

验证:
- OK + ^CEND + IDLE
- 不继续到 ACTIVE
```

---

#### TC-C07: INCOMING 阶段挂断 (拒接来电)

**覆盖**: 来电时发 CHUP = 拒接

```
步骤:
1. SIM_CTRL [1, 0] → SIM_INCOMING [0]
2. 等待 RING
3. 发送 AT+CHUP

验证:
- OK + ^CEND:2,1,105,17,0 + ^DSCI:6
- 状态变为 IDLE
- RING 停止
```

---

#### TC-C08: 通话中开启/关闭模拟

**覆盖**: 活跃通话中尝试改变模拟状态

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE
2. SIM_CTRL [0, 0]  → 尝试关闭模拟
3. SIM_CTRL [1, 1]  → 尝试切换场景

验证:
- 步骤 2: 响应错误 (Cannot change sim state while call is active)
- 步骤 3: 响应错误
- 通话继续不受影响
```

---

#### TC-C09: 通话中触发新来电

**覆盖**: 已有通话时触发来电

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE
2. SIM_INCOMING [0]

验证:
- 触发失败 (Cannot trigger incoming, call active)
- 原通话继续
```

---

#### TC-C10: 未开启模拟时拨号

**覆盖**: SIM_CTRL disabled 状态下发送 ATD

```
步骤:
1. SIM_CTRL [0, 0]  → 关闭模拟
2. 发送 ATD10086;

验证:
- ATD 不被模拟拦截，走真实 TT 模块 AT 通道
- 或返回 ERROR (取决于实现)
```

---

#### TC-C11: 拨号号码格式

**覆盖**: ATD 不同号码格式

```
步骤:
1. SIM_CTRL [1, 0]
2. 分别测试:
   a. ATD10086;        → 正常号码
   b. ATD13800138000;  → 手机号
   c. ATD;             → 无号码 (应使用默认 SIM_TEST_NUMBER)
   d. ATDabc;          → 非数字

验证:
- a/b: 正常拨号
- c: 使用默认号码 "10086" 或 ERROR
- d: ERROR 或忽略非数字
```

---

### D 类：语音服务管理

---

#### TC-D01: 语音空闲超时自动关闭

**覆盖**: 语音服务 30s 无数据自动关闭

```
步骤:
1. SIM_CTRL [1, 0]
2. SERVICE_START [5]   → 开启语音
3. 不拨号，等待 30s
4. 验证日志: "voice idle timeout"
5. SERVICE_STATUS [5]  → 查询语音状态

验证:
- 30s 后语音服务自动关闭
- SERVICE_STATUS 返回 stopped
```

---

#### TC-D02: 通话接通重置空闲定时器

**覆盖**: ATA/^CONN 时重置 30s 空闲定时器

```
步骤:
1. SIM_CTRL [1, 0]
2. SERVICE_START [5]   → 开启语音 (空闲计时开始)
3. 等待 25s (接近超时)
4. SIM_INCOMING [0] → 立即 ATA → 接通
5. 验证空闲定时器被重置
6. 语音数据流动 → 不会在 30s+5s 后超时

验证:
- ATA 后定时器重置
- 语音注入开始 → 不会触发空闲超时
```

---

#### TC-D03: 通话中关闭语音服务

**覆盖**: ACTIVE 通话中 SERVICE_STOP [5]

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE → 语音
2. SERVICE_STOP [5]

验证:
- 语音数据停止
- sat_call_sim_voice_disabled() 被调用
- 通话本身继续 (不挂断)
- TT 模块/模拟侧仍 ACTIVE
```

---

#### TC-D04: 通话中关闭再重开语音

**覆盖**: 语音服务 disable → enable 循环

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE → 语音
2. SERVICE_STOP [5]   → 语音停
3. 等 3s
4. SERVICE_START [5]  → 语音重开

验证:
- 停止时语音立即中断
- 重开后语音立即恢复
- sat_call_sim_voice_enabled() 在 ACTIVE 时自动启动注入
- 数据无缝恢复
```

---

#### TC-D05: 帧模式切换 (1-frame vs 3-frame)

**覆盖**: 通话中切换帧模式

```
步骤:
1. GET_VOICE_FRAME_MODE → 记录当前模式
2. SIM_CTRL [1, 0] → ATD10086; → ACTIVE → 语音
3. SET_VOICE_FRAME_MODE [1]  → 切换到 1-frame (Android)
4. 等待 5s → 验证语音正常 (20ms 间隔)
5. SET_VOICE_FRAME_MODE [3]  → 切换到 3-frame (iPhone)
6. 等待 5s → 验证语音正常 (60ms 间隔, 3帧一组)

验证:
- 切换不中断通话
- 帧模式影响注入节奏
- BLE 连接参数可能更新 (itvl_min/max)
```

---

#### TC-D06: SPP 服务启停 (voice data channel)

**覆盖**: SPP 服务 (0x04) 独立于 VOICE 服务 (0x05)

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE
2. SERVICE_STOP [4]   → 停 SPP (语音数据传输通道)
3. 验证语音数据停止传输
4. SERVICE_START [4]  → 重开 SPP
5. 验证语音恢复

验证:
- SPP 控制数据传输通道
- VOICE 控制语音注入任务
- 两者独立控制
```

---

### E 类：BLE 断开 & 通话保持

---

#### TC-E01: 信号丢失 → 保持 30s → 超时挂断 (模拟 ACTIVE)

**覆盖**: 核心保持 + 超时路径

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE → SERVICE_START [5]
2. BLE_TX_POWER_TEMP [-24, 60]  → 降功率触发信号丢失
3. 等待 BLE 断开
4. 验证日志:
   - disconnect; reason=520 或 546
   - preserving call for 30000 ms
5. 30s 内不重连
6. 验证日志:
   - BLE reconnect timeout (30000 ms)
   - Sim call CHUP sent (timeout hangup)
7. SIM_GET_STATE → IDLE

通过标准: 保持 30s 后自动挂断
```

---

#### TC-E02: 信号丢失 → 保持 30s → 超时挂断 (各通话状态)

**覆盖**: 不同通话状态下信号丢失

```
子场景:
a) DIALING: ATD10086; → 立即降功率 → BLE 断开
   - 验证: preserving call (DIALING 算活跃通话)
   - 模拟序列继续: DIALING → ALERTING → ACTIVE
   - 30s 后超时挂断

b) ALERTING: ATD10086; → 等 ^CONF → 降功率 → BLE 断开
   - 验证: preserving call
   - 30s 后超时挂断

c) INCOMING: SIM_INCOMING [0] → 不接听 → 降功率 → BLE 断开
   - 验证: preserving call
   - RING 继续在设备侧
   - 30s 后超时挂断

通过标准: 所有非 IDLE 状态都触发保持
```

---

#### TC-E03: 信号丢失 → 5s 内重连 → 语音恢复

**覆盖**: 保持期间重连成功

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. BLE_TX_POWER_TEMP [-24, 60] → BLE 断开 → 保持
3. 等 5s
4. BLE_TX_POWER_TEMP [12, 0] → 恢复功率
5. 手机重连 BLE
6. 验证日志: BLE reconnected while call preserved
7. App 订阅 notification
8. SERVICE_START [5]
9. 验证语音数据恢复

通过标准: 重连取消定时器，语音恢复
```

---

#### TC-E04: 信号丢失 → 25s 重连 → 接近超时恢复

**覆盖**: 临近超时时重连

```
步骤:
1. 同 TC-E03 步骤 1-2
2. 等 25s (接近 30s)
3. 快速重连 + SERVICE_START [5]
4. 验证语音恢复
5. 无超时挂断

通过标准: 倒计时最后 5s 内重连成功
```

---

#### TC-E05: 信号丢失 → 重连但不开语音

**覆盖**: 重连后 app 未恢复语音

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. BLE 断开 → 保持
3. 5s 后重连
4. 不发 SERVICE_START [5]

验证:
- 重连取消定时器
- 通话在模拟侧继续
- 无语音数据传输
- 通话持续到对方挂断或用户手动操作
```

---

#### TC-E06: 用户主动断开 (关蓝牙) → 立即挂断

**覆盖**: 用户主动断开不触发保持

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. 手机关蓝牙

验证:
- 日志: disconnect; reason=531 (0x213)
- 日志: Active sim call on user disconnect, sent CHUP
- 日志: 无 "preserving call"
- SIM_GET_STATE → IDLE
- 无 30s 等待

通过标准: 立即挂断
```

---

#### TC-E07: 用户主动断开 — 各通话状态

**覆盖**: 不同通话状态下用户主动断开

```
子场景:
a) DIALING + 关蓝牙 → 立即挂断
b) ALERTING + 关蓝牙 → 立即挂断
c) INCOMING + 关蓝牙 → 立即挂断
d) ACTIVE + 关蓝牙 → 立即挂断

验证: 每个子场景都收到 CHUP + IDLE
```

---

#### TC-E08: 无通话时信号丢失 → 正常清理

**覆盖**: 无通话不触发保持

```
步骤:
1. SIM_CTRL [1, 0] (空闲状态)
2. BLE_TX_POWER_TEMP [-24, 60] → BLE 断开

验证:
- 日志: no active call, normal cleanup
- 无 "preserving call"
- 无 30s 定时器
- 正常清理 + cancel force_on
```

---

#### TC-E09: 通话已结束后信号丢失

**覆盖**: 刚挂断就信号丢失

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE → AT+CHUP → IDLE
2. 等 2s
3. BLE_TX_POWER_TEMP [-24, 60] → BLE 断开

验证:
- 无 preserving call
- 正常清理
```

---

#### TC-E10: 重连后立即再断开 (循环保持)

**覆盖**: 连续断开-重连的压力场景

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. 循环 3 次:
   a. BLE_TX_POWER_TEMP [-24, 10] → BLE 断开 → 保持
   b. 等 5s
   c. BLE_TX_POWER_TEMP [12, 0] → 重连
   d. SERVICE_START [5]

验证:
- 每次断开正确保持
- 每次重连正确取消定时器
- 无 crash/内存泄漏
- 最后一次语音仍正常
```

---

#### TC-E11: 保持期间通话自然结束 (NETWORK_DROP)

**覆盖**: 保持中模拟网络掉话

```
步骤:
1. SIM_CTRL [1, 3] → ATD10086; → ACTIVE → 语音
2. BLE 断开 → 保持
3. NETWORK_DROP 5~15s 后自动掉话 → 模拟侧 IDLE
4. 30s 定时器到期
5. 定时器回调: sim_state==IDLE → 不发 CHUP (已结束)

验证:
- 无异常/crash
- 定时器对已结束通话不产生错误
```

---

#### TC-E12: 保持期间 DEBUG 连接发 CHUP

**覆盖**: 保持中通过另一连接挂断

```
前提: CONFIG_BLE_MULTI_CONN_ENABLE

步骤:
1. PRIMARY 连接: SIM_CTRL [1, 0] → ATD10086; → ACTIVE
2. PRIMARY BLE 断开 (信号丢失) → 保持
3. DEBUG 连接发送 AT+CHUP

验证:
- 模拟通话被挂断 → IDLE
- 30s 定时器仍触发但 sim_state==IDLE → 跳过 CHUP
- 无异常
```

---

#### TC-E13: 飞行模式场景

**覆盖**: 手机开飞行模式的 reason 判断

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. 手机开飞行模式

验证:
- reason=531 (0x213, REM_USER_CONN_TERM)
- 立即挂断通话 (不保持)
- 如果需要测试保持: 先降功率触发 0x208 断开，保持期间开飞行模式 (已断开无影响)
```

---

### F 类：电源管理 & TT 模块交互

---

#### TC-F01: TT_FORCE_ON 在通话期间

**覆盖**: 通话期间 force_on 保持 TT 模块开机

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE
2. 验证 force_on 已生效 (由语音服务自动管理)
3. SERVICE_STOP [5] → 关语音
4. AT+CHUP → IDLE
5. 验证 force_on 被取消

验证:
- 通话期间 TT 模块不被关机
- 通话结束后 force_on 取消
```

---

#### TC-F02: 信号丢失保持 → force_on 不取消

**覆盖**: 保持期间 TT 模块保持开机

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. BLE 信号丢失断开 → 保持
3. 验证日志无 "cancel_force_on"
4. 30s 后超时 → cancel_force_on 被调用

验证:
- 保持期间 TT 模块开机
- 超时挂断后才取消 force_on
```

---

#### TC-F03: 用户断开 → force_on 取消 + 挂断

**覆盖**: 用户主动断开时正确清理

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. 手机关蓝牙

验证:
- cancel_force_on 被调用
- AT+CHUP 发出
- TT 模块可被电源管理正常关闭
```

---

#### TC-F04: 低电量 + 通话

**覆盖**: 低电量状态下通话行为

```
步骤:
1. 设备处于低电量状态 (电量 < 阈值)
2. SIM_CTRL [1, 0] → ATD10086;
3. 观察是否允许拨号
4. 如果 TT_FORCE_ON 被设置: 强制允许通话
5. 如果无 force_on: 可能被低电量保护阻止

验证:
- 低电量保护与通话的交互正确
- force_on 可覆盖低电量保护
```

---

### G 类：BLE TX 功率模拟触发

---

#### TC-G01: -24 dBm 触发信号丢失

**覆盖**: 极低功率确保断开

```
步骤:
1. ACTIVE 通话 → 语音
2. BLE_TX_POWER_TEMP [-24, 120]
3. 距离 2-3m
4. 等待 10-30s → BLE 断开

验证:
- reason=520 或 546
- 触发通话保持
```

---

#### TC-G02: -3 dBm 不触发断开

**覆盖**: 小幅降功率不触发

```
步骤:
1. ACTIVE 通话 → 语音
2. BLE_TX_POWER_TEMP [-3, 60]
3. 距离 2-3m，观察 60s

验证:
- BLE 连接保持
- 通话正常
- 60s 后功率恢复
```

---

#### TC-G03: 功率自动恢复后重连

**覆盖**: 功率超时恢复 → 信号恢复 → 重连

```
步骤:
1. ACTIVE 通话 → 语音
2. BLE_TX_POWER_TEMP [-12, 15]  → 15s 后恢复
3. BLE 断开 → 保持
4. 等 15s → 功率恢复 +12 dBm
5. 手机自动/手动重连
6. SERVICE_START [5]

验证:
- 保持期间功率恢复
- 重连成功
- 语音恢复
- 总保持时间 < 30s → 不超时
```

---

#### TC-G04: 功率恢复但超时前未重连

**覆盖**: 功率恢复但 app 未重连

```
步骤:
1. ACTIVE 通话 → 语音
2. BLE_TX_POWER_TEMP [-12, 5]   → 5s 恢复功率
3. BLE 断开 → 保持
4. 功率 5s 恢复但手机不重连
5. 等 30s → 超时挂断

验证:
- 功率恢复 ≠ 自动重连
- 30s 后正确超时挂断
```

---

#### TC-G05: 非法功率值

**覆盖**: 不支持的 dBm 值

```
步骤:
1. BLE_TX_POWER_TEMP [5, 30]    → +5 dBm (不支持)
2. BLE_TX_POWER_TEMP [-30, 30]  → -30 dBm (不支持)
3. BLE_TX_POWER_TEMP [25, 30]   → +25 dBm (不支持)

验证:
- 每次返回 resp_code=0x03 (INVALID_PARAM)
- 功率不变
```

---

#### TC-G06: 超时前再次设置功率 (重置定时器)

**覆盖**: 重复调用 TX 功率命令

```
步骤:
1. BLE_TX_POWER_TEMP [-12, 60]  → 设 -12dBm, 60s 恢复
2. 等 5s
3. BLE_TX_POWER_TEMP [-6, 30]   → 改为 -6dBm, 30s 恢复
4. 验证功率更新为 -6dBm
5. 等待 30s → 功率恢复 +12dBm (不是 60s)

验证:
- 重复调用重置定时器
- 新参数覆盖旧参数
```

---

#### TC-G07: 永久修改功率 (timeout=0)

**覆盖**: 无超时永久修改

```
步骤:
1. BLE_TX_POWER_TEMP [6]        → +6dBm, 无 timeout 字段 (param_len=1)
2. 等 60s → 功率不恢复
3. BLE_TX_POWER_TEMP [12]       → 恢复默认

验证:
- 无超时，永久生效
- 响应中 original 字段显示 +12dBm
```

---

### H 类：网络模拟 & AT 命令

---

#### TC-H01: 自定义信号强度 (CSQ)

**覆盖**: SIM_SET_NET 设置 CSQ

```
步骤:
1. SIM_CTRL [1, 0]
2. SIM_SET_NET [15, 1]  → CSQ=15, CREG=1 (注册)
3. 发送 AT+CSQ → 验证返回 +CSQ: 15,0
4. 发送 AT+CREG? → 验证返回 +CREG: 0,1
5. SIM_SET_NET [0, 0]   → CSQ=0, CREG=0 (无信号)
6. AT+CSQ → +CSQ: 0,0

验证:
- 模拟网络参数正确返回
```

---

#### TC-H02: 查询 SIM 状态 (AT+CPIN)

```
步骤:
1. SIM_CTRL [1, 0]
2. 发送 AT+CPIN?

验证:
- 返回 +CPIN: READY
```

---

#### TC-H03: 查询设备信息 (AT+CIMI / AT+CCID / AT+CGMR / AT+CGMM)

```
步骤:
1. SIM_CTRL [1, 0]
2. AT+CIMI → 460116072498875
3. AT+CCID → 89860326247551943462
4. AT+CGMR → HWA BP HS 6.0.5.260206
5. AT+CGMM → HTDM1310E

验证:
- 模拟固定设备信息正确返回
```

---

#### TC-H04: 查询通话列表 (AT+CLCC)

**覆盖**: 不同通话状态下 AT+CLCC

```
步骤:
1. SIM_CTRL [1, 0]
2. IDLE 时: AT+CLCC → 验证返回空列表或 OK
3. ATD10086; → DIALING → AT+CLCC → 验证返回通话信息
4. ACTIVE → AT+CLCC → 验证返回 ACTIVE 通话
5. CHUP → IDLE → AT+CLCC → 验证空

验证:
- CLCC 反映当前通话状态
```

---

#### TC-H05: AT+CLIP (来电显示)

```
步骤:
1. SIM_CTRL [1, 0]
2. AT+CLIP=1 → 开启来电显示
3. SIM_INCOMING [0] → 验证 RING 包含号码信息

验证:
- 来电显示信息正确
```

---

### I 类：音频 & PCM 数据

---

#### TC-I01: 回铃音 (ALERTING 阶段)

**覆盖**: 拨号后的回铃音

```
步骤:
1. SIM_CTRL [1, 0]
2. SERVICE_START [5]
3. ATD10086; → 等待 ALERTING
4. 分析语音数据 → 验证 440Hz + 480Hz 双音
5. 验证 1s on / 2s off 节奏

验证:
- ALERTING 阶段播放标准回铃音
```

---

#### TC-I02: 来电铃声 (INCOMING 阶段)

**覆盖**: 来电时的铃声

```
步骤:
1. SIM_CTRL [1, 0]
2. SERVICE_START [5]
3. SIM_INCOMING [0] → INCOMING
4. 分析语音数据 → 验证铃声模式

验证:
- INCOMING 播放铃声
- 与回铃音相同 (440+480Hz)
```

---

#### TC-I03: 通话语音 (ACTIVE 阶段) — PCM 或合成音

**覆盖**: ACTIVE 阶段的语音数据

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE → 语音
2. 分析语音数据

验证 (二选一):
- PCM: 来自 flash pcm_data 分区，8kHz/16bit/mono
- 合成: 600Hz 正弦波 + tremolo + 5% noise
- 帧: 320 bytes / 20ms (1-frame) 或 960 bytes / 60ms (3-frame)
```

---

#### TC-I04: PCM 数据循环播放

**覆盖**: PCM 数据播完后循环

```
步骤:
1. 准备一个较短的 PCM 文件 (~10s)
2. ATD10086; → ACTIVE → 语音
3. 等待播完 → 验证从头开始循环

验证:
- 循环播放无卡顿
- 无异常帧
```

---

### J 类：连接参数 & 多连接

---

#### TC-J01: 语音开启后连接参数更新

**覆盖**: 语音服务启用后 BLE 连接参数变化

```
步骤:
1. 连接后记录默认连接参数 (itvl=24-40, 即 30-50ms)
2. SERVICE_START [5]
3. 读取更新后的参数:
   - 1-frame: itvl=8-10 (10-12.5ms)
   - 3-frame: itvl=14-16 (17.5-20ms)

验证:
- 参数正确切换
```

---

#### TC-J02: 语音关闭后参数恢复

```
步骤:
1. 语音开启 → 参数已更新
2. SERVICE_STOP [5]
3. 读取参数 → 验证恢复默认 (24-40)

验证:
- 参数恢复
```

---

#### TC-J03: 最大连接数限制

**覆盖**: 超出最大连接数

```
步骤:
1. 连接设备 1 (PRIMARY) → 成功
2. 连接设备 2 (DEBUG) → 成功
3. 连接设备 3 → 应被拒绝 (terminated)

验证:
- 设备 3 被拒绝
- 原有连接不受影响
```

---

#### TC-J04: PRIMARY 断开后广播恢复

```
步骤:
1. PRIMARY + DEBUG 都连接
2. PRIMARY 断开
3. 验证广播重新开始
4. 新设备可连接

验证:
- 断开后重新广播
- 新设备可接管 PRIMARY
```

---

### K 类：边界 & 竞态条件

---

#### TC-K01: ATA 在 INCOMING 超时瞬间 (29.5s)

**覆盖**: 来电超时边界

```
步骤:
1. SIM_CTRL [1, 0] → SIM_INCOMING [0]
2. 等待 29.5s (接近 30s 超时)
3. 快速发送 ATA

验证:
- 两种可能:
  a. ATA 先到 → 接通成功
  b. 超时先到 → ATA 返回 ERROR
- 无论哪种，无 crash
```

---

#### TC-K02: CHUP 和 NET_DROP 同时发生

**覆盖**: 用户挂断和网络掉话竞态

```
步骤:
1. SIM_CTRL [1, 3] → ATD10086; → ACTIVE
2. NET_DROP 随机 5~15s 内会掉话
3. 在即将掉话时发 AT+CHUP

验证:
- 只收到一次 ^CEND
- 状态回到 IDLE
- 无重复挂断或 crash
```

---

#### TC-K03: BLE 断开在 DIALING → ALERTING 转换时

**覆盖**: 状态转换瞬间断开

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086;
2. 在 ~3s 时 (DIALING → ALERTING 转换点) 降功率

验证:
- 无论在 DIALING 还是 ALERTING 都触发保持
- 无 crash
```

---

#### TC-K04: 快速连续 ATD-CHUP-ATD

**覆盖**: 快速拨号挂断循环

```
步骤:
1. SIM_CTRL [1, 0]
2. 快速: ATD10086; → CHUP → ATD10086;
3. 验证第二次 ATD 成功
4. 等待 ACTIVE
5. 正常挂断

验证:
- 快速操作不卡在错误状态
- 每次操作独立
```

---

#### TC-K05: App crash 后重启重连

**覆盖**: app 崩溃恢复

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. 强制杀掉 App (杀进程)
3. BLE 断开 (reason=0x208 或 0x213)
4. 如果 reason=0x208 → 保持
5. 10s 内重新打开 App → BLE 重连
6. SIM_GET_STATE → ACTIVE
7. 订阅 + SERVICE_START [5] → 语音恢复

验证:
- crash 后通话保持
- 重连后恢复
```

---

#### TC-K06: 模拟通话与真实 TT 通话切换

**覆盖**: 模拟关闭后切到真实通话

```
步骤:
1. SIM_CTRL [1, 0] → ATD10086; → ACTIVE → CHUP → IDLE
2. SIM_CTRL [0, 0] → 关闭模拟
3. 通过 TT 模块拨打真实电话
4. 验证 AT 命令走真实通道

验证:
- 模拟关闭后不影响真实通话
- AT 命令不被模拟拦截
```

---

#### TC-K07: MTU 不足导致语音帧截断

**覆盖**: 小 MTU 场景

```
步骤:
1. 设置 MTU 较小 (如果可控)
2. ACTIVE 通话 → 语音
3. 验证大帧被正确处理或报错

验证:
- MTU 不足时不 crash
- 数据正确分片或报错
```

---

#### TC-K08: 语音队列满

**覆盖**: 语音数据堆积

```
步骤:
1. SIM_CTRL [1, 0] → ACTIVE → 语音
2. 模拟大量数据注入 (高频率)
3. 观察队列满时的行为

验证:
- 队列满时丢帧，不 crash
- 日志有 "queue full" 警告
- 恢复后正常
```

---

#### TC-K09: 不完整语音帧

**覆盖**: 收到不完整的 320 字节帧

```
步骤:
1. 通过 MUX CH9 发送不完整帧 (< 320 bytes)
2. 验证被丢弃

验证:
- 不完整帧被丢弃
- 日志有 "incomplete frame" 警告
- 下一帧正常处理
```

---

### L 类：SIM_GET_STATE 查询

---

#### TC-L01: 各状态下查询

```
步骤:
1. SIM_CTRL [1, 0]
2. 各阶段查询 SIM_GET_STATE:
   a. IDLE → state=0, scenario=0
   b. ATD → DIALING → state=1
   c. ALERTING → state=2
   d. ACTIVE → state=3
   e. SIM_INCOMING → state=4

验证:
- 每个阶段返回正确状态
```

---

#### TC-L02: 查询包含完整信息

```
步骤:
1. SIM_CTRL [1, 0]
2. SIM_GET_STATE → 验证响应包含:
   - enabled (bool)
   - scenario (uint8)
   - state (uint8)
   - csq (uint8)
   - creg (uint8)

验证:
- 所有字段正确
```

---

## 三、测试结果记录表

| ID | 类别 | 描述 | 结果 | 备注 |
|----|------|------|------|------|
| TC-A01 | 正常流程 | 外呼完整流程 NORMAL | ⬜ | |
| TC-A02 | 正常流程 | 来电完整流程 | ⬜ | |
| TC-A03 | 正常流程 | 延迟触发来电 | ⬜ | |
| TC-A04 | 正常流程 | 先拨号后开语音 | ⬜ | |
| TC-A05 | 正常流程 | 先开语音后拨号 | ⬜ | |
| TC-A06 | 正常流程 | 来电不接超时 | ⬜ | |
| TC-B01 | 场景变体 | BUSY 忙音 | ⬜ | |
| TC-B02 | 场景变体 | NO_ANSWER 不接 | ⬜ | |
| TC-B03 | 场景变体 | REJECT 拒接 | ⬜ | |
| TC-B04 | 场景变体 | NETWORK_DROP 掉话 | ⬜ | |
| TC-B05 | 场景变体 | 场景切换 | ⬜ | |
| TC-B06 | 场景变体 | 连续 5 次通话 | ⬜ | |
| TC-C01 | 命令错误 | ATD in ACTIVE | ⬜ | |
| TC-C02 | 命令错误 | ATD in DIALING | ⬜ | |
| TC-C03 | 命令错误 | ATA without INCOMING | ⬜ | |
| TC-C04 | 命令错误 | CHUP while IDLE | ⬜ | |
| TC-C05 | 命令错误 | CHUP in DIALING | ⬜ | |
| TC-C06 | 命令错误 | CHUP in ALERTING | ⬜ | |
| TC-C07 | 命令错误 | CHUP in INCOMING (拒接) | ⬜ | |
| TC-C08 | 命令错误 | 通话中切换模拟 | ⬜ | |
| TC-C09 | 命令错误 | 通话中触发来电 | ⬜ | |
| TC-C10 | 命令错误 | 未开启模拟时拨号 | ⬜ | |
| TC-C11 | 命令错误 | 拨号号码格式 | ⬜ | |
| TC-D01 | 语音服务 | 空闲超时 30s | ⬜ | |
| TC-D02 | 语音服务 | 接通重置空闲定时器 | ⬜ | |
| TC-D03 | 语音服务 | 通话中关闭语音 | ⬜ | |
| TC-D04 | 语音服务 | 语音关→开恢复 | ⬜ | |
| TC-D05 | 语音服务 | 帧模式切换 | ⬜ | |
| TC-D06 | 语音服务 | SPP 服务启停 | ⬜ | |
| TC-E01 | BLE 保持 | 信号丢失→超时挂断 | ⬜ | |
| TC-E02 | BLE 保持 | 各状态信号丢失 | ⬜ | |
| TC-E03 | BLE 保持 | 信号丢失→5s 重连 | ⬜ | |
| TC-E04 | BLE 保持 | 信号丢失→25s 重连 | ⬜ | |
| TC-E05 | BLE 保持 | 重连不开语音 | ⬜ | |
| TC-E06 | BLE 保持 | 关蓝牙立即挂断 | ⬜ | |
| TC-E07 | BLE 保持 | 关蓝牙各状态 | ⬜ | |
| TC-E08 | BLE 保持 | 无通话信号丢失 | ⬜ | |
| TC-E09 | BLE 保持 | 通话结束→信号丢失 | ⬜ | |
| TC-E10 | BLE 保持 | 循环断开重连 3 次 | ⬜ | |
| TC-E11 | BLE 保持 | 保持中 NET_DROP | ⬜ | |
| TC-E12 | BLE 保持 | 保持中 DEBUG 挂断 | ⬜ | |
| TC-E13 | BLE 保持 | 飞行模式 | ⬜ | |
| TC-F01 | 电源管理 | 通话期间 force_on | ⬜ | |
| TC-F02 | 电源管理 | 保持期 force_on 不取消 | ⬜ | |
| TC-F03 | 电源管理 | 用户断开 force_on 取消 | ⬜ | |
| TC-F04 | 电源管理 | 低电量 + 通话 | ⬜ | |
| TC-G01 | TX 功率 | -24dBm 触发断开 | ⬜ | |
| TC-G02 | TX 功率 | -3dBm 不触发 | ⬜ | |
| TC-G03 | TX 功率 | 功率恢复后重连 | ⬜ | |
| TC-G04 | TX 功率 | 功率恢复但未重连 | ⬜ | |
| TC-G05 | TX 功率 | 非法功率值 | ⬜ | |
| TC-G06 | TX 功率 | 重复设置功率 | ⬜ | |
| TC-G07 | TX 功率 | 永久修改功率 | ⬜ | |
| TC-H01 | AT 命令 | CSQ/CREG 设置查询 | ⬜ | |
| TC-H02 | AT 命令 | CPIN 查询 | ⬜ | |
| TC-H03 | AT 命令 | 设备信息查询 | ⬜ | |
| TC-H04 | AT 命令 | CLCC 通话列表 | ⬜ | |
| TC-H05 | AT 命令 | CLIP 来电显示 | ⬜ | |
| TC-I01 | 音频 | 回铃音 | ⬜ | |
| TC-I02 | 音频 | 来电铃声 | ⬜ | |
| TC-I03 | 音频 | 通话语音 PCM/合成 | ⬜ | |
| TC-I04 | 音频 | PCM 循环播放 | ⬜ | |
| TC-J01 | 连接参数 | 语音开启参数更新 | ⬜ | |
| TC-J02 | 连接参数 | 语音关闭参数恢复 | ⬜ | |
| TC-J03 | 连接参数 | 最大连接数 | ⬜ | |
| TC-J04 | 连接参数 | 断开后广播恢复 | ⬜ | |
| TC-K01 | 边界竞态 | ATA 在超时瞬间 | ⬜ | |
| TC-K02 | 边界竞态 | CHUP 与 NET_DROP 竞态 | ⬜ | |
| TC-K03 | 边界竞态 | 状态转换时断开 | ⬜ | |
| TC-K04 | 边界竞态 | 快速 ATD-CHUP-ATD | ⬜ | |
| TC-K05 | 边界竞态 | App crash 恢复 | ⬜ | |
| TC-K06 | 边界竞态 | 模拟↔真实切换 | ⬜ | |
| TC-K07 | 边界竞态 | MTU 不足 | ⬜ | |
| TC-K08 | 边界竞态 | 语音队列满 | ⬜ | |
| TC-K09 | 边界竞态 | 不完整语音帧 | ⬜ | |
| TC-L01 | 状态查询 | 各状态 SIM_GET_STATE | ⬜ | |
| TC-L02 | 状态查询 | 查询完整信息 | ⬜ | |

**总计: 60 个测试用例, 12 个类别**
