# 睡眠管理 & 天通模块状态管理 — 技术参考

## 一、睡眠状态机

### 四个状态

```
                         BLE 连接 + 空闲
    ┌──────────┐  ──────────────────────→  ┌──────────────┐
    │          │                            │              │
    │  ACTIVE  │  ←──── BLE/GPIO 唤醒 ─────│ LIGHT_SLEEP  │  (当前已禁用)
    │  ~60mA   │                            │ ~240µA       │
    │          │                            └──────────────┘
    └────┬─────┘
         │ BLE 断开 + TT 关 + 空闲 ≥ 阈值
         ▼
    ┌──────────────┐  ── 定时器唤醒 ──→  ┌──────────────┐
    │              │                      │              │
    │ DEEP_SLEEP   │  ←── 超时/无连接 ────│ TEMP_AWAKE   │
    │ ~7µA         │                      │ 广播 25s     │
    │              │  ←── 按键/USB/BB ────│              │
    └──────────────┘     (直接→ACTIVE)    └──────┬───────┘
                                                 │ BLE 连接
                                                 ▼
                                           延迟全初始化 → ACTIVE
```

### 状态说明

| 状态 | 功耗 | 说明 |
|------|------|------|
| **ACTIVE** | ~60mA | 正常工作，所有任务运行 |
| **LIGHT_SLEEP** | ~240µA | BLE 保持连接，CPU 在连接事件间隙暂停（当前已禁用） |
| **DEEP_SLEEP** | ~7µA | 芯片复位级睡眠，仅 RTC 存活，GPIO 状态保持 |
| **TEMP_AWAKE** | ~60mA | Deep sleep 定时唤醒，最小初始化 + BLE 广播 25 秒 |

---

## 二、睡眠决策流程（每秒执行）

```
sleep_decision_task():
│
├─ 1. inhibit_sleep / TEMP_AWAKE 模式 → 跳过
│
├─ 2. 充电中（IP5561 bit2 或 WPC）→ 重置计数，跳过
│
├─ 3. BLE 连接数同步（cache 修正，见下文）
│
├─ 4. if (BLE 连接 OR TT 上电):
│      ├─ 重置 deep sleep 计数
│      ├─ Auto收网检测（每 60s 查 CREG）
│      ├─ TT 初始化中 → 跳过 light sleep
│      ├─ 通话中 → 跳过 light sleep
│      └─ 空闲 ≥ 阈值 → enter_light_sleep()  [当前禁用]
│
├─ 5. else (BLE 断开 AND TT 关):
│      ├─ deep sleep 计数++
│      ├─ 到达阈值?
│      │   ├─ 最终检查: BLE 仍连接? → 中止
│      │   └─ enter_deep_sleep()
│      └─ 每 10s 打印调试日志
```

### 配置阈值

| 参数 | V1 | V2 | 说明 |
|------|----|----|------|
| Light Sleep 进入 | 3600000s（禁用）| 同 | 测试客户端超时太短，暂时禁用 |
| Light Sleep 重入 | 2s | 2s | 唤醒后快速再睡 |
| Deep Sleep 进入 | 300s | 120s | BLE 断开 + TT 关后的空闲超时 |
| Deep Sleep 定时器 | 60s | 1800s | TEMP_AWAKE 唤醒间隔 |
| TEMP_AWAKE 持续 | 25s | 25s | 等待 APP 连接窗口 |

---

## 三、BLE 连接计数管理

### 三重保护机制

| 机制 | 用途 | 实现 |
|------|------|------|
| callback cache (`g_ble_conn_count`) | light/deep sleep 决策 | GAP 事件回调 +1/-1 |
| NimBLE 查询 (`ble_gap_conn_find`) | stale 条目修正 | 决策任务每秒查 |
| `ble_conn_manager` 自愈 | 接受新连接 | `is_max_connections()` 满载时验证 |

### 同步规则

```
is_ble_connected():
  cache > 0  →  return true（信任 cache，不查 NimBLE）
  cache == 0 →  查 NimBLE，找到则 sync up

决策任务里：
  nimble > 0 且 < cache  →  修正 cache 向下（安全）
  nimble == 0 且 cache > 0 →  不修正（可能假阴性）
  nimble == cache  →  一致，不处理
```

---

## 四、Deep Sleep 唤醒源

| 唤醒源 | 机制 | 触发条件 | 唤醒后模式 |
|--------|------|---------|-----------|
| **Timer** | `esp_sleep_enable_timer_wakeup` | 每 60s(V1) / 1800s(V2) | TEMP_AWAKE |
| **ext1: GPIO9** | `ESP_EXT1_WAKEUP_ANY_LOW` | pwrkey 按下（V2） | ACTIVE |
| **ext1: GPIO21** | `ESP_EXT1_WAKEUP_ANY_LOW` | BB_WAKEUP_AP（TT 开时） | ACTIVE |
| **ext0: GPIO5** | `esp_sleep_enable_ext0_wakeup(GPIO5, 1)` | IP5561 INT 拉高（USB 插入） | ACTIVE |

### Deep Sleep GPIO 保持

| 场景 | GPIO1 (TTPWR/Boost) | GPIO38 (LDO) | GPIO36 (Boost Mode) | GPIO8 (KEY) |
|------|---------------------|--------------|---------------------|-------------|
| TT 开 | LOW (boost ON) | HIGH (LDO ON) | HIGH (normal) | 保持 |
| TT 关 + 电量低 | LOW (boost ON) | LOW (LDO OFF) | HIGH (normal) | 保持 |
| TT 关 + 电量正常 | HIGH (boost OFF) | LOW (LDO OFF) | LOW (低功耗) | 保持 |

---

## 五、TEMP_AWAKE 流程

```
Deep Sleep 定时器唤醒
  → sleep_manager_init() 检测 TIMER 唤醒
  → g_current_mode = TEMP_AWAKE
  → app_main() 最小初始化（BLE + 基础服务）
  → 开始广播 + 启动 25s 定时器
  │
  ├─ APP 连接进来:
  │   → 取消定时器
  │   → g_current_mode = ACTIVE
  │   → 启动延迟全初始化任务
  │   │   ├─ 音频编解码
  │   │   ├─ TT 模块（如果 NVS 未标记 manual_off）
  │   │   ├─ 电源监控
  │   │   ├─ 睡眠决策任务
  │   │   └─ BLE 健康监控
  │   → 系统完全运行
  │
  └─ 25s 超时:
      → 安全检查（BLE? 充电? inhibit? 模式?）
      → 全部通过 → 回 Deep Sleep
```

---

## 六、天通模块状态机

### 状态定义

```c
TT_STATE_HARDWARE_FAULT   = 0  // 硬件故障
TT_STATE_INITIALIZING     = 1  // 初始化中
TT_STATE_WAITING_MUX_RESP = 2  // 等待 MUX 响应（瞬态）
TT_STATE_LOW_BATTERY_OFF  = 3  // 低电自动关机
TT_STATE_USER_OFF         = 4  // 用户/收网 关机
TT_STATE_WORKING          = 5  // 正常工作
TT_STATE_UPDATING         = 6  // OTA 升级中
```

### 状态转换图

```
                    user_power_on / force_on
                          │
                          ▼
                   ┌─────────────┐
            ┌──────│ INITIALIZING │──────┐
            │      └──────┬──────┘      │
            │             │              │
       硬件超时       MUX 协商成功      │
            │             │              │
            ▼             ▼              │
    ┌──────────────┐ ┌─────────┐         │
    │HARDWARE_FAULT│ │ WORKING │←────────┘
    └──────────────┘ └────┬────┘
                          │
              ┌───────────┼───────────┐
              │           │           │
         低电自动关    OTA 升级    用户手动关
              │           │           │
              ▼           ▼           ▼
        ┌────────────┐┌─────────┐┌──────────┐
        │LOW_BATT_OFF││UPDATING ││ USER_OFF │
        └────────────┘└─────────┘└──────────┘
              │                       │
          电压恢复                user_power_on
          force_on              收网关→TEMP_AWAKE重试
```

---

## 七、TT 模块电源控制函数

### 五种关机方式对比

| 方式 | 函数 | NVS 标志 | TT 状态 | 自动恢复 | force_on 保护 | 互斥锁 |
|------|------|---------|---------|---------|--------------|--------|
| 手动关 | `user_power_off()` | ✅ manual_off=true | USER_OFF | ❌ 需用户手动开 | ❌ | ✅ |
| 低电关 | `low_battery_shutdown()` | ❌ 不写 | LOW_BATTERY_OFF | ✅ 电压恢复 | ✅ 跳过 | ✅ |
| 收网关 | `network_timeout_off()` | ❌ 不写 | USER_OFF | ✅ TEMP_AWAKE 重试 | ✅ 检查 | ✅ |
| 超长按 | `user_power_off()` + deep sleep | ✅ manual_off=true | USER_OFF | ❌ | ❌ | ✅ |
| OTA 完成 | 自动恢复到 WORKING | — | WORKING | — | — | — |

### 两种开机方式

| 方式 | 函数 | NVS 标志 | 适用场景 |
|------|------|---------|---------|
| 手动开 | `user_power_on()` | ✅ 清除 manual_off | APP 命令 / pwrkey 长按 |
| 强制开 | `force_on()` | ✅ 清除 manual_off | 来电（不受低电限制）|

### Power Mutex

```c
static SemaphoreHandle_t g_power_mutex = NULL;  // 懒创建，永不删除

// 保护所有 power lifecycle 函数:
// user_power_on / user_power_off / network_timeout_off / low_battery_shutdown / force_on
// 防止不同任务并发操作 TT 电源
```

---

## 八、Auto 收网机制

### 检测流程

```
sleep_decision_task（TT 上电 + WORKING 时）:
│
├─ 每 60s 发 AT+CREG?
│   ├─ stat=1(home) 或 stat=5(roaming) → 已注册 → g_tt_no_net_sec = 0
│   ├─ 其他 → 未注册 → g_tt_no_net_sec += 60
│   └─ AT 失败 → 算未注册
│
├─ 每 5 分钟打印进度日志
│
└─ g_tt_no_net_sec >= 600 (10 分钟):
    ├─ force_on? → 跳过
    ├─ 通话中? → 跳过
    └─ tt_module_network_timeout_off()
        → 关 TT（不写 NVS）
        → g_tt_powered = false
        → BLE 断开后 → 进入 Deep Sleep
        → TEMP_AWAKE 唤醒 → TT 自动重启 → 重试搜网
```

---

## 九、Pwrkey 按键处理（V2 专属）

### 中断驱动 + 按下后轮询

```
GPIO9 下降沿中断 → ISR 唤醒 pwrkey_monitor_task
  │
  ├─ 空闲: task 阻塞在 ulTaskNotifyTake（零 CPU）
  │
  └─ 按下后: 每 20ms 轮询
      │
      ├─ 松手 < 50ms → 消抖忽略
      ├─ 松手 < 2s   → 短按: notify_activity("pwrkey")
      ├─ 松手 2~5s   → 长按: toggle TT 电源
      └─ 按住 ≥ 5s   → 超长按: 关 TT + 强制 deep sleep
```

---

## 十、跨模块交互矩阵

### 谁影响睡眠决策

| 因素 | 效果 | 来源 |
|------|------|------|
| BLE 连接 | 阻止 deep sleep | `g_ble_conn_count` |
| TT 上电 | 阻止 deep sleep，触发 auto收网 | `g_tt_powered` |
| 充电中 | 阻止所有睡眠 | `power_manage_get_charging_status()` |
| 通话中 | 阻止 light sleep + auto收网 | `spp_voice_server_is_call_active()` |
| OTA 中 | 阻止 TT 关机 | `tt_module_is_user_control_allowed()` |
| inhibit 标志 | 阻止所有睡眠 | `sleep_manager_set_inhibit()` |
| force_on 标志 | 阻止低电关 + auto收网 | `g_tt_force_on` |

### 通知链

```
BLE 连接/断开:
  ble_gatt_server.c → sleep_manager_notify_ble_connected/disconnected()

TT 上电/关电:
  tt_module.c → sleep_manager_notify_tt_powered_on/off()

APP 命令:
  gatt_system_server.c → sleep_manager_notify_activity("sys_cmd_rx")

AT 命令:
  spp_at_server.c → sleep_manager_notify_activity("spp_at_rx")

语音数据:
  spp_voice_server.c → sleep_manager_refresh_idle()

按键短按:
  pwrkey_monitor_task → sleep_manager_notify_activity("pwrkey")
```

---

## 十一、RTC 数据持久化

```c
RTC_DATA_ATTR static volatile rtc_data_t rtc_data = { ... };

// 字段:
//   boot_count       — 正常启动次数
//   deep_sleep_count — Deep sleep 进入次数
//   last_battery_mv  — 上次 deep sleep 前的电池电压
//   sleep_timestamp  — 进入 deep sleep 的时间戳
//   crc16            — CRC-16-CCITT 校验
//   magic            — 0x52544344 ("RTCD")
```

用于：
- Deep sleep 唤醒后恢复状态
- 判断是否需要低电保护（V2 boost 仲裁）
- 统计 deep sleep 次数

---

## 十二、关键阈值汇总

### 睡眠

| 参数 | 值 | 说明 |
|------|----|------|
| `SLEEP_LIGHT_IDLE_SEC` | 3600000 | Light sleep（当前禁用）|
| `SLEEP_LIGHT_REENTER_SEC` | 2 | Light sleep 重入 |
| `SLEEP_DEEP_IDLE_SEC` | V1:300 / V2:120 | Deep sleep 进入超时 |
| `SLEEP_DEEP_TIMER_SEC` | V1:60 / V2:1800 | Deep sleep 定时器间隔 |
| `SLEEP_TEMP_AWAKE_SEC` | 25 | TEMP_AWAKE 广播窗口 |
| `SLEEP_LIGHT_TIMER_SEC` | 30 | Light sleep 定时检查 |

### Auto 收网

| 参数 | 值 |
|------|----|
| `TT_NO_NET_CHECK_SEC` | 60s |
| `TT_NO_NET_TIMEOUT_SEC` | 600s（10 分钟）|

### Pwrkey（V2）

| 参数 | 值 |
|------|----|
| 轮询间隔 | 20ms |
| 消抖 | 50ms |
| 短按阈值 | < 2000ms |
| 超长按阈值 | ≥ 5000ms |

### 电池保护

| 参数 | 值 |
|------|----|
| TT 模块关机电压 | 3100mV |
| TT 模块恢复电压 | 3500mV |
| V2 低电 boost 开启 | 3300mV |
| V2 低电 boost 关闭 | 3400mV |

### TT 初始化

| 参数 | 值 |
|------|----|
| SIMST 等待超时 | 15s |
| SIMST 最大重试 | 3 次 |
| SIMST 重试间隔 | 2000ms |
| AT 命令超时 | 5000ms |
