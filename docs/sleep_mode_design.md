# ESP32-S3 混合低功耗方案设计

## 一、背景

关闭天通模块后，ESP32-S3 全速运行 + BLE 广播功耗约 ~60mA。通过混合使用 Light Sleep 和 Deep Sleep，可将待机功耗降至 ~240uA（BLE保持连接）或 ~7uA（BLE断开）。

## 二、整体状态机

```
                        ┌──────────────┐
                        │   ACTIVE     │
                        │  正常工作     │
                        │  ~60mA       │
                        └──────┬───────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
     BLE已连接,空闲>T1    BLE未连接,     用户关机/
     (无通话/AT)          TT模块关闭      低压保护
              │           空闲>T2            │
              ▼                ▼              ▼
      ┌──────────────┐  ┌──────────────┐  ┌──────────┐
      │ LIGHT_SLEEP  │  │ DEEP_SLEEP   │  │ USER_OFF │
      │ BLE保持连接   │  │ ~7uA         │  │ 全关     │
      │ ~240uA       │  │              │  └──────────┘
      └──────┬───────┘  └──────┬───────┘
             │                 │
     BLE活动/GPIO21     定时唤醒/GPIO21
     /通话/AT命令       (广播5秒)
             │                 │
             ▼                 ▼
      ┌──────────────┐  ┌──────────────┐
      │   ACTIVE     │  │ TEMP_AWAKE   │
      │              │  │ 广播5秒等连接  │
      └──────────────┘  │ 无连接→再睡   │
                        └──────────────┘
```

## 三、三阶段定义

| 参数 | Light Sleep | Deep Sleep | Temp Awake |
|---|---|---|---|
| **进入条件** | BLE已连接 + 空闲>T1 | BLE未连接 + 空闲>T2 | Deep Sleep 定时/GPIO唤醒 |
| **功耗** | ~240uA | ~7uA | ~60mA（短暂） |
| **CPU** | 暂停，保持RAM | 关闭，重启恢复 | 正常运行 |
| **BLE** | 保持连接/广播 | 完全关闭 | 重新广播 |
| **唤醒源** | BLE事件/GPIO21/Timer | GPIO21/Timer | - |
| **唤醒延迟** | <1ms | ~500ms（重启+BLE初始化） | - |
| **建议超时** | T1 = 10秒 | T2 = 60秒 | 广播5秒 |

## 四、详细流程

### 4.1 ACTIVE → LIGHT_SLEEP

```
条件: BLE已连接 && 无通话 && 无AT命令在执行 && 空闲>10s

入睡前:
  1. 暂停 power_monitor 任务（不需要持续轮询）
  2. 配置唤醒源:
     - esp_sleep_enable_bt_wakeup()        // BLE事件自动唤醒
     - gpio_wakeup_enable(GPIO_NUM_21, GPIO_INTR_LOW_LEVEL)  // BB唤醒AP
     - esp_sleep_enable_timer_wakeup(30s)   // 30秒醒一次查电量
  3. esp_light_sleep_start()

唤醒后（原地继续执行）:
  1. 检查唤醒原因
  2. 如果是Timer唤醒: 读电量，低于阈值则关TT模块
  3. 如果是BLE唤醒: 恢复处理BLE数据
  4. 如果是GPIO21: BB要通信，恢复ACTIVE
  5. 重新评估是否继续Light Sleep
```

### 4.2 ACTIVE → DEEP_SLEEP

```
条件: BLE未连接 && TT模块关闭(或USER_OFF) && 空闲>60s

入睡前:
  1. 保存关键状态到 RTC_DATA_ATTR:
     - TT模块状态
     - 电量快照
     - 睡眠时间戳
     - deep_sleep_boot_count
  2. 保存BLE绑定信息（NimBLE已有NVS存储）
  3. 配置唤醒源:
     - esp_sleep_enable_ext0_wakeup(GPIO_NUM_21, 0)  // BB_WAKEUP_AP_PIN低电平
     - esp_sleep_enable_timer_wakeup(60s)              // 每60秒醒一次
  4. 配置GPIO为低漏电状态:
     - 未用引脚设为 pull-up/pull-down
     - gpio_deep_sleep_hold_en()
  5. esp_deep_sleep_start()  // 不会返回，重启
```

### 4.3 DEEP_SLEEP → TEMP_AWAKE（定时唤醒）

```
app_main() 中检测唤醒原因:

  cause = esp_sleep_get_wakeup_cause()

  if cause == ESP_SLEEP_WAKEUP_TIMER:
    // 定时唤醒，目的是短暂广播BLE
    1. 仅初始化最小子系统（NVS + BLE）
    2. 开始BLE广播（不启动TT模块）
    3. 启动一个5秒定时器
    4. 等待:
       - 如果5秒内有BLE连接 → 完整初始化 → ACTIVE
       - 如果5秒内无连接 → 重新进入DEEP_SLEEP
    5. 每次定时唤醒时读电量，过低则不再唤醒直到充电

  if cause == ESP_SLEEP_WAKEUP_EXT0:
    // GPIO21触发，天通模块要通信
    1. 完整初始化 → ACTIVE
    2. 启动BLE广播
```

## 五、新增文件和修改

### 5.1 新增文件：`main/system/sleep_manager.h`

```c
// 睡眠模式定义
typedef enum {
    SLEEP_MODE_NONE = 0,       // 活跃模式
    SLEEP_MODE_LIGHT,          // Light Sleep（BLE保持）
    SLEEP_MODE_DEEP,           // Deep Sleep
    SLEEP_MODE_TEMP_AWAKE      // Deep Sleep定时唤醒后的临时活跃
} sleep_mode_t;

// 配置参数
#define SLEEP_LIGHT_IDLE_SEC       10    // BLE连接时空闲多久进Light Sleep
#define SLEEP_DEEP_IDLE_SEC        60    // BLE断开后多久进Deep Sleep
#define SLEEP_TEMP_AWAKE_SEC       5     // 定时唤醒后广播多久
#define SLEEP_DEEP_TIMER_SEC       60    // Deep Sleep定时唤醒间隔
#define SLEEP_LIGHT_TIMER_SEC      30    // Light Sleep定时唤醒间隔（查电量）

// API
esp_err_t sleep_manager_init(void);
sleep_mode_t sleep_manager_get_mode(void);
void sleep_manager_notify_activity(void);    // 通知有活动，重置空闲计时
void sleep_manager_notify_ble_connected(void);
void sleep_manager_notify_ble_disconnected(void);
void sleep_manager_notify_tt_powered_off(void);
void sleep_manager_notify_tt_powered_on(void);
void sleep_manager_enter_deep_sleep(void);   // 主动进入Deep Sleep
bool sleep_manager_is_deep_sleep_wakeup(void); // 是否从Deep Sleep唤醒
void sleep_manager_task_start(void);         // 启动睡眠决策任务
```

### 5.2 新增文件：`main/system/sleep_manager.c`

```
核心任务 sleep_decision_task():
  while(1):
    if BLE已连接 && 无通话 && 空闲>T1:
      enter_light_sleep()
    elif BLE未连接 && TT已关 && 空闲>T2:
      enter_deep_sleep()
    else:
      vTaskDelay(1s)
```

### 5.3 修改文件

| 文件 | 修改内容 |
|---|---|
| `main/main.c` | 启动时判断是否Deep Sleep唤醒，决定走完整初始化还是最小初始化 |
| `main/system/power_manage.c` | 入睡前暂停monitor任务，唤醒后恢复 |
| `main/ble/ble_gatt_server.c` | 连接/断开事件通知sleep_manager |
| `main/ble/ble_conn_manager.c` | 连接/断开事件通知sleep_manager |
| `main/tt/tt_module.c` | TT模块开关状态通知sleep_manager |
| `main/config/user_params.h` | 新增sleep相关配置参数 |
| `CMakeLists.txt` | 新增 sleep_manager.c |

### 5.4 RTC 数据保留（跨Deep Sleep）

```c
// 保存在RTC内存，Deep Sleep唤醒后仍在
RTC_DATA_ATTR static uint32_t rtc_boot_count = 0;
RTC_DATA_ATTR static uint32_t rtc_deep_sleep_count = 0;
RTC_DATA_ATTR static uint16_t rtc_last_battery_mv = 0;
RTC_DATA_ATTR static bool rtc_tt_was_on = false;
RTC_DATA_ATTR static int64_t rtc_sleep_timestamp = 0;
```

## 六、关键场景流程

### 6.1 用户打开APP → 连接BLE → 打开天通模块

```
DEEP_SLEEP (定时唤醒)
  │
  ├─ Timer 60s 到
  │   └─ TEMP_AWAKE: 初始化BLE，广播5秒
  │       │
  │       ├─ 手机扫描到 → 连接BLE
  │       │   └─ sleep_manager_notify_ble_connected()
  │       │       └─ 完整初始化 → ACTIVE
  │       │           └─ APP发"开机"命令 → tt_module_user_power_on()
  │       │
  │       └─ 5秒无连接 → 重新DEEP_SLEEP
```

### 6.2 天通模块主动唤醒（来电）

```
DEEP_SLEEP
  │
  ├─ GPIO21 低电平（天通模块唤醒AP）
  │   └─ app_main() 检测到 EXT0 唤醒
  │       └─ 完整初始化 → ACTIVE
  │           └─ BLE广播 → 手机连接
  │               └─ APP显示来电
```

### 6.3 通话结束后省电

```
ACTIVE (通话中)
  │
  ├─ 通话结束
  │   └─ 用户在APP关天通模块
  │       └─ TT模块下电 → sleep_manager_notify_tt_powered_off()
  │           │
  │           ├─ BLE仍连接 → 空闲10s → LIGHT_SLEEP (~240uA)
  │           │   └─ 保持BLE连接，等手机操作
  │           │       └─ 手机断开BLE → 空闲60s → DEEP_SLEEP (~7uA)
  │           │
  │           └─ BLE已断开 → 空闲60s → DEEP_SLEEP (~7uA)
```

## 七、功耗估算

| 场景 | 当前 | 优化后 | 节省 |
|---|---|---|---|
| BLE连接+天通关+空闲 | ~60mA | ~0.24mA (Light Sleep) | 250倍 |
| BLE断开+天通关+待机 | ~60mA | ~0.007mA (Deep Sleep) | 8500倍 |
| 定时唤醒广播5秒/60秒 | - | ~5mA平均 | - |
| 3.7V/3000mAh电池待机 | ~50小时 | **数月** | - |

## 八、风险和注意事项

1. **IP5561/BQ27220 静态功耗**：即使ESP32 Deep Sleep，这两个IC仍有自身功耗（IP5561 ~50uA，BQ27220 ~数十uA），实际板级 Deep Sleep 可能在 50~100uA
2. **LDO/DCDC 效率**：板上的稳压器自身静态电流会影响最终功耗
3. **BLE 绑定信息**：Deep Sleep 后 BLE 连接会断开，需要确保 NimBLE bonding keys 存在 NVS 中，避免用户重新配对
4. **GPIO 漏电**：入睡前必须把所有 GPIO 配置为确定状态，否则漏电可能吃掉省下的电量
5. **BB_WAKEUP_AP_PIN (GPIO21)**：ESP32-S3 的 RTC GPIO，支持 ext0/ext1 Deep Sleep 唤醒
6. **SDK 配置**：需要在 menuconfig 中启用 `CONFIG_FREERTOS_USE_TICKLESS_IDLE` 以支持自动 Light Sleep
