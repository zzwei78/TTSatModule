# IP5561 充电管理芯片驱动

## 概述

IP5561 是一个高度集成的电源管理IC，支持锂电池充电管理、升压转换和电源路径管理。

**重要特性**：IP5561使用**双I2C地址**架构，不同寄存器组使用不同的I2C地址。

## 硬件特性

- **双I2C地址架构**：
  - **0xE8/0xE9** - 控制寄存器（系统控制、充电控制等）
  - **0xEA/0xEB** - 状态和ADC寄存器（电压、电流、电量百分比等）
- **I2C频率**：最高300kHz（推荐200kHz）
- **充电电压**：4.2V - 4.4V
- **通信方式**：8位寄存器地址，8位数据，MSB优先

## I2C地址分配

### I2C地址 0xE8/0xE9 - 控制寄存器

用于系统控制、充电使能、升压控制等：

| 寄存器地址 | 名称 | 功能 |
|-----------|------|------|
| 0x00 | SYS_CTL0 | 系统控制0 - 充电/升压使能 |
| 0x03 | SYS_CTL1 | 系统控制1 - 轻载关机 |
| 0x0C | CHG_CTL1 | 9V输入充电欠压环路 |
| 0x0D | CHG_CTL2 | 5V输入充电欠压环路 |
| 0x10 | VOUT_CTL0 | VOUT控制0 |
| 0x18 | VBUS_CTL0 | VBUS控制0 |
| 0x33 | CHG_CTL5 | 充电电压使能控制 |

### I2C地址 0xEA/0xEB - 状态和ADC寄存器

用于读取状态、电压、电流、电量等：

| 寄存器地址 | 名称 | 功能 |
|-----------|------|------|
| 0x01 | SYS_STAT0 | 系统状态0 |
| 0x03 | SYS_STAT1 | 系统状态1 |
| 0x19 | SYS_STAT2 | 系统状态2 |
| 0x30-0x3A | CHG_CFG | 充电电压/电流配置 |
| 0x60-0x61 | VBAT_ADC | 电池电压ADC |
| 0x62-0x63 | IBAT_ADC | 电池电流ADC |
| 0x64-0x65 | NTC1_ADC | NTC1温度ADC |
| 0x68-0x69 | VBUS_ADC | VBUS电压ADC |
| 0x6A-0x6B | IBUS_ADC | VBUS电流ADC |
| 0x6E-0x6F | VOUT_ADC | VOUT电压ADC |
| 0x7B | SOC_PERCENT | 电池百分比(0-100%) |

## 软件架构

驱动自动处理双I2C地址：

```c
typedef struct {
    i2c_master_dev_handle_t i2c_ctrl_handle;   // 0xE8 (控制寄存器)
    i2c_master_dev_handle_t i2c_stat_handle;   // 0xEA (状态/ADC寄存器)
} ip5561_data_t;
```

读写寄存器时，驱动根据寄存器地址自动选择正确的I2C设备：

```c
static bool ip5561_reg_uses_stat_addr(uint8_t reg)
{
    // ADC和状态寄存器使用0xEA地址
    if ((reg >= 0x60 && reg <= 0x7B) ||  // ADC和SOC
        (reg >= 0x30 && reg <= 0x3A) ||  // 充电配置
        reg == 0x01 || reg == 0x03 ||    // 系统状态
        (reg >= 0xA0 && reg <= 0xC3)) {  // 其他状态
        return true;  // 使用0xEA
    }
    return false;  // 使用0xE8
}
```

## API接口

### 初始化

```c
ip5561_config_t config = {
    .i2c_bus = i2c_bus_handle,
    .scl_freq_hz = 200000,  // 可选，默认200kHz
};
ip5561_handle_t handle = ip5561_create(&config);

// 驱动会自动创建两个I2C设备：
// - 0xE8 (右移1位 = 0x74, 7位地址)
// - 0xEA (右移1位 = 0x75, 7位地址)

// 销毁设备
ip5561_delete(handle);
```

### 充电控制

```c
// 使能/禁用充电
ip5561_set_charge_enable(handle, true);

// 设置充电电压 (4200-4400mV)
ip5561_set_charge_voltage(handle, 4200);  // 4.2V

// 使能/禁用升压模式
ip5561_set_boost_enable(handle, true);
```

### 状态读取

```c
// 获取充电状态
ip5561_chg_status_t status;
ip5561_get_charge_status(handle, &status);

// 检查各状态位
bool charging = ip5561_is_charging(handle);
bool done = ip5561_is_charge_complete(handle);
bool vbus = ip5561_is_vbus_present(handle);
```

### ADC读取

```c
// 读取电池百分比
uint8_t percent = ip5561_get_battery_percent(handle);

// 读取电池电压 (ADC值)
uint16_t vbat = ip5561_get_battery_voltage(handle);

// 读取电池电流 (ADC值)
int16_t ibat = ip5561_get_battery_current(handle);

// 读取VBUS电压
uint16_t vbus = ip5561_get_vbus_voltage(handle);

// 读取VOUT电压
uint16_t vout = ip5561_get_vout_voltage(handle);
```

### 寄存器级别访问

```c
// 单字节读写
uint8_t value;
ip5561_read_reg(handle, 0x00, &value);  // 自动使用0xE8
ip5561_write_reg(handle, 0x00, value);

ip5561_read_reg(handle, 0x60, &value);  // 自动使用0xEA
ip5561_write_reg(handle, 0x60, value);

// 多字节读写
uint8_t data[4];
ip5561_read_regs(handle, 0x60, data, 4);
ip5561_write_regs(handle, 0x60, data, 4);
```

## 数据结构

### ip5561_chg_status_t

充电状态结构体：

```c
typedef struct {
    bool chg_enabled;     /* 充电使能状态 */
    bool vbus_valid;      /* VBUS电源有效 */
    bool chg_done;        /* 充电完成 */
    bool charging;        /* 正在充电 */
    uint8_t chg_status;   /* 原始状态寄存器值 */
} ip5561_chg_status_t;
```

## 使用示例

```c
#include "IP5561.h"

void app_main(void)
{
    // 创建IP5561设备（自动创建双I2C设备）
    ip5561_config_t config = {
        .i2c_bus = g_i2c_bus,
    };
    ip5561_handle_t ip5561 = ip5561_create(&config);
    if (ip5561 == NULL) {
        ESP_LOGE(TAG, "Failed to create IP5561");
        return;
    }

    // 配置充电参数
    ip5561_set_charge_voltage(ip5561, 4200);  // 4.2V

    // 使能充电
    ip5561_set_charge_enable(ip5561, true);

    // 监控充电状态
    while (1) {
        ip5561_chg_status_t status;
        ip5561_get_charge_status(ip5561, &status);

        uint8_t percent = ip5561_get_battery_percent(ip5561);
        uint16_t vbat = ip5561_get_battery_voltage(ip5561);

        ESP_LOGI(TAG, "Charging: %d, Done: %d, SOC: %d%%, VBAT: %d",
                 status.charging, status.chg_done, percent, vbat);

        if (ip5561_is_charge_complete(ip5561)) {
            ESP_LOGI(TAG, "Charging complete!");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    // 清理
    ip5561_delete(ip5561);
}
```

## 重要注意事项

### 1. 双I2C地址架构

IP5561使用两个I2C地址，**不同的寄存器必须使用正确的地址**：

```c
// 正确 - 驱动自动选择地址
ip5561_read_reg(handle, 0x00, &val);  // 自动使用0xE8
ip5561_read_reg(handle, 0x60, &val);  // 自动使用0xEA

// 错误 - 手动选择错误的I2C地址会导致通信失败
// 不要直接调用i2c_master_transmit，必须使用ip5561_*函数
```

### 2. I2C模式进入条件

从休眠唤醒时，IP5561会检测LED1/LED2引脚：
- **LED1和LED2都上拉到3.3V** → 进入I2C模式
- 其他状态 → LED灯显模式

确保MCU在IP5561休眠时将SDA/SCL配置为输入或高阻状态。

### 3. INT信号时序

IP5561从休眠唤醒后：
1. INT引脚输出高电平
2. 等待100ms后才能进行I2C通信
3. 提前通信会导致IP5561无法正确进入I2C模式

### 4. 寄存器读写规则

**必须使用读-修改-写的方式**：

```c
// 正确的方式
uint8_t val;
ip5561_read_reg(handle, 0x00, &val);   // 先读
val &= ~0x01;                            // 修改需要的bit
val |= 0x02;                             // 设置需要的bit
ip5561_write_reg(handle, 0x00, val);   // 再写

// 错误的方式 - 会改变未开放的bit
ip5561_write_reg(handle, 0x00, 0x02);
```

### 5. ADC值转换

当前驱动返回原始ADC值，需要根据具体应用转换：

```c
// 电池电压 (示例，需要根据实际调整)
uint16_t vbat_adc = ip5561_get_battery_voltage(handle);
float vbat_v = vbat_adc * 0.001;  // 转换为伏特

// 电池电流 (示例，需要根据实际调整)
int16_t ibat_adc = ip5561_get_battery_current(handle);
float ibat_ma = ibat_adc * 1.0;    // 转换为毫安
```

## 错误处理

所有API返回 `esp_err_t` 类型：
- `ESP_OK` - 操作成功
- `ESP_ERR_INVALID_ARG` - 参数无效
- `ESP_ERR_NOT_FOUND` - I2C通信失败

## 与bq27220的对比

| 特性 | bq27220 | IP5561 |
|------|---------|--------|
| 功能 | 电池电量计 | 充电管理+升压 |
| I2C地址 | 单地址 0x55 | 双地址 0xE8/0xEA |
| 寄存器宽度 | 16位 | 8位 |
| 驱动复杂度 | 中等（CEDV算法） | 高（双地址自动路由） |
| ADC | 内置电量算法 | 需要外部转换 |

## 文件结构

```
components/IP5561/
├── IP5561.h                 # 公开API
├── IP5561.c                 # 实现（双I2C地址处理）
├── priv_include/
│   └── IP5561_reg.h         # 寄存器定义（双地址）
├── CMakeLists.txt
└── README.md                # 本文档
```

## 参考文档

- 原厂PDF: `IP5561(with reg) V1.00.pdf`
- I2C通信频率：≤ 300kHz
- 推荐频率：200kHz
- 通信方式：8位寄存器地址，8位数据，MSB优先
