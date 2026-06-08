# 天通模块 (GMR MES) 呼叫状态机与 iOS CallKit 集成架构

**目标受众:** iOS App 开发者 / AI 辅助开发助手 (Claude 等)
**架构背景:** 将天通卫星模块的 AT 命令及底层 URC (Unsolicited Result Code) 映射到 iOS `CallKit` (`CXProvider` / `CXCallObserver`)。整体状态机设计借鉴了 Android 16 Telephony 框架中 `CallTracker` 的流转逻辑，并将其适配至 iOS 生态。

---

## 一、 核心状态映射 (State Mapping)

天通模块上报的底层状态需要被精确映射为 App 层及 `CallKit` 可识别的标准呼叫状态：

| `^DSCI` 状态码 | 业务含义 | 对应的应用层 / `CallKit` 状态 | 触发的 AT 动作/说明 |
| :--- | :--- | :--- | :--- |
| **`4`** | MT 来电建立中 | `Incoming` (新来电) | 上报后，调用 `CXProvider.reportNewIncomingCall` |
| **`2`** | MO 拨号中 | `Dialing` (正在呼出) | 由 `ATD` 触发，向 `CallKit` 报告呼出已发起 |
| **`3`** | MO 振铃中 | `Alerting` (对方振铃) | 向 `CallKit` 报告呼出正在响铃 |
| **`5`** | 呼叫等待中 | `Waiting` (呼叫等待) | 记录新 Call ID，更新 UI 提示等待接听 |
| **`0`** | 呼叫活动中 | `Active` (通话接通) | 主被叫接通（`ATA` 或对方接听），启动计时器 |
| **`1`** | 通话保留中 | `Holding` (通话保持) | 由 `AT+CHLD` 触发，暂停当前通话音频流 |
| **`6`** | 呼叫结束 | `Disconnected` (挂断) | 解析 `<cause>` 码，调用 `CXProvider.reportCall` 结束 |

---

## 二、 核心 URC 字典与信令定义

除了 `^DSCI` 提供的 7 种呼叫状态外，底层还提供了一组关键的业务进程 URC[cite: 1, 2]。在解析串口数据时，需统筹处理：

| URC 信令 | 含义 / 触发时机 | 报文示例 | CallKit / UI 动作指导 |
| :--- | :--- | :--- | :--- |
| `^ORIG` | **呼叫发起指示** | `^ORIG: 1,0` | 本地发出的呼叫请求已被 Modem 接收并开始处理。 |
| `^DSCI: <id>,0,2...` | **拨号中 (Dialing)** | `^DSCI: 1,0,2,0,0,"10086",129` | 转换状态为 **Dialing**。 |
| `^DSCI: <id>,0,3...` | **振铃中 (Alerting)** | `^DSCI: 1,0,3,0,0,"10086",129` | 转换状态为 **Alerting**，可向用户播放本地回铃音。 |
| `^CONF` | **网络连通指示** | `^CONF: 1,0` | **关键心跳：** 表示卫星核心网已分配资源并连通承载，但对方**尚未接听**。重置应用层的呼叫等待超时定时器。 |
| `^CONN` | **呼叫接通指示** | `^CONN: 1,0` | 对方已提机接听。 |
| `^DSCI: <id>,0,0...` | **活动中 (Active)** | `^DSCI: 1,0,0,0,0,"10086",129` | 紧随 `^CONN` 上报，转换状态为 **Active**，此时音频路由应建立完毕，开始计费/计时。 |
| `^CEND` | **通话结束指示** | `^CEND: 1,,101,17,0` | 包含网络原因码，早于或与 `DSCI: 6` 同时到达。 |
| `^DSCI: <id>,0,6...` | **呼叫结束 (End)**[cite: 2] | `^DSCI: 1,0,6,0,0,"10086",129,16` | 提取 `<cause>` 字段[cite: 2]，调用 `CXProvider.reportCall` 销毁通话，释放 `Call ID`。 |
| `^DSCI: <id>,1,4...` | **来电建立中 (MT)**[cite: 2] | `^DSCI: 2,1,4,0,0,"138...",145` | 触发 `reportNewIncomingCall` 显示 iOS 系统来电界面。 |

---

## 三、 呼叫主流程设计 (Call Flows)

在初始化阶段，App 必须确保下发 `AT^DSCI=1`（开启呼叫状态上报）[cite: 2] 和 `AT+CLIP=1`（开启主叫号码显示）。

### 1. 主叫流程 (MO - Mobile Originated)
由用户在 iOS App 发起：
* **用户拨号：** UI 发起拨号，App 下发 `ATD<number>;`[cite: 1]。
* **状态流转：**
  * 模块上报：`^DSCI: <id>,0,2...` 👉 进入 **Dialing** 状态。
  * 模块上报：`^DSCI: <id>,0,3...` 👉 进入 **Alerting** 状态。
  * 模块上报：`^CONF: <id>,0` 👉 确认承载打通。
  * 模块上报：`^CONN: <id>,0` 和 `^DSCI: <id>,0,0...` 👉 进入 **Active** 状态，接通音频路由。
* **结束通话：** 用户点击挂断，下发 `AT+CHUP`[cite: 1]，模块上报 `^DSCI: <id>,0,6...`，销毁该 Call ID。

### 2. 被叫流程 (MT - Mobile Terminated)
由卫星网络侧触发：
* **收到来电：** 模块主动上报 `^DSCI: <id>,1,4,0,0,"<number>",<type>`[cite: 2]（可能伴随 `^ALERT`[cite: 1]）。
* **状态流转：** App 解析出 `id` 和 `number`，唤起 iOS `CallKit` 来电界面 👉 进入 **Incoming** 状态。
* **用户接听：**
  * 用户滑动接听，App 下发 `ATA`[cite: 1]。
  * 模块上报：`^DSCI: <id>,1,0...` 👉 进入 **Active** 状态，建立双向音频。

---

## 四、 完整呼叫状态机时序图 (CallTracker 视角)

此图展示了包含 `^CONF` 及相关信令的完整闭环，请以此作为 Swift 中 `CallStateMachine` 的开发基准。

```mermaid
sequenceDiagram
    participant UI as iOS App (CallKit / UI)
    participant SM as StateMachine (串口解析层)
    participant Modem as 天通模块 (硬件)

    Note over UI, Modem: 【预置条件】上电初始化
    SM->>Modem: AT^DSCI=1\r (开启状态上报)
    Modem-->>SM: OK
    
    Note over UI, Modem: 【场景 A】主动呼叫 (Mobile Originated)
    UI->>SM: 发起呼出 (号码: 10086)
    SM->>Modem: ATD10086;\r
    Modem-->>SM: ^ORIG: 1,0 (呼叫发起)
    Modem-->>SM: ^DSCI: 1,0,2,0,0,"10086",129
    SM->>UI: State -> Dialing (正在呼叫)
    
    Modem-->>SM: ^DSCI: 1,0,3,0,0,"10086",129
    SM->>UI: State -> Alerting (对方振铃)
    
    Note right of Modem: 卫星基站侧确认资源
    Modem-->>SM: ^CONF: 1,0 (网络已连通承载)
    Note over SM: 内部记录网络已就绪，重置超时 Timer
    
    Note right of Modem: 对方点击接听
    Modem-->>SM: ^CONN: 1,0 (呼叫接通)
    Modem-->>SM: ^DSCI: 1,0,0,0,0,"10086",129
    SM->>UI: State -> Active (通话建立，开始计时)

    Note over UI, Modem: 【场景 B】被动接听 (Mobile Terminated)
    Modem-->>SM: ^ALERT (可选的底层提醒)
    Modem-->>SM: ^DSCI: 2,1,4,0,0,"13800138000",145
    SM->>UI: 唤起 CallKit (Incoming Call)
    UI->>SM: 用户滑动接听
    SM->>Modem: ATA\r
    Modem-->>SM: ^CONN: 2,0
    Modem-->>SM: ^DSCI: 2,1,0,0,0,"13800138000",145
    SM->>UI: State -> Active (接通)
    
    Note over UI, Modem: 【场景 C】通话结束 (Disconnection)
    UI->>SM: 用户点击挂断
    SM->>Modem: AT+CHUP\r
    Modem-->>SM: ^CEND: 1,,101,16,0 (正常挂断)
    Modem-->>SM: ^DSCI: 1,0,6,0,0,"10086",129,16
    SM->>UI: State -> Disconnected (释放系统资源)
    
## 五、 开发与异常处理指南

在卫星通信的高延迟、易遮挡环境下，除了被动依赖 URC 上报外，应用层必须具备极强的容错与纠偏能力。

### 1. 状态对齐与“幽灵通话”恢复 (Ghost Call Recovery)
* **场景描述:** 卫星信号瞬间严重遮挡导致 URC 丢失（如漏收 `^DSCI: <id>,<dir>,6...`[cite: 2]），造成 iOS UI 界面停留在“通话中”，或 UI 层面已挂断但底层模块仍在悄悄计费。
* **处理策略:** 
  * 引入主动轮询机制。在通话建立 (Active) 后，App 层应启动低频定时器，下发 `AT+CLCC` (查询当前呼叫列表)[cite: 1] 校验底层真实的通话状态。
  * 将 `AT+CLCC` 返回的列表与本地 `CallStateMachine` 中的 Call ID 进行严格比对。如果发现本地有通话记录但模块返回空列表，则强制调用 `CXProvider.reportCall` 终结 UI 状态；反之，若 UI 已挂断但模块仍有 Active 记录，必须立即补发 `AT+CHUP`[cite: 1] 掐断底层链路。

### 2. AT 指令超时与重传机制 (Timeout & Retry)
* **场景描述:** 向串口下发 `ATD`[cite: 1] 或 `ATA`[cite: 1] 等关键动作后，由于硬件处理瓶颈，未能在预期时间内收到对应的响应或 `^ORIG`[cite: 1]。
* **处理策略:** 
  * 为每一条控制类 AT 指令设立 2000ms - 3000ms 的超时守护定时器。
  * 触发超时后，应先下发基本的 `AT` 指令探测模块存活状态。若模块能够响应 `OK`，可进行至多 1 次指令重试；若重试依然无响应或模块离线，立刻上抛底层硬件错误通知。

### 3. 信令去重与防抖 (Debouncing)
* **场景描述:** 状态转换瞬间，模块可能会紧挨着连续吐出表示接通的 `^CONN`[cite: 1] 和 `^DSCI: <id>,<dir>,0...`[cite: 2]。
* **处理策略:** 必须针对同一个 Call ID 进行状态幂等处理。如果当前该呼叫已经处于 `Active` 状态，应忽略后续重复抵达的接通 URC 报文，避免向 CallKit 密集提交状态变更导致抛出 Exception。

### 4. 断接原因提取与错误降级 (Cause Code Degradation)
* **场景描述:** 呼叫非正常终止，需要向用户解释网络原因。
* **处理策略:** 拦截 `^DSCI` 状态 6 结尾的 `<cause>` 字段[cite: 2]。
  * 如果 `<cause> = 16`，执行静默挂断。
  * 如果 `<cause> = 17` (用户忙)[cite: 2]，将错误码转换为本地化语言提示“对方正忙”。
  * 对于没有明确指定原因的网络掉线断开（如 `<cause> = 31`），准确告知用户“卫星链路异常断开”，以安抚用户对网络波动的焦虑。    
    