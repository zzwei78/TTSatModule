IP5561 无线充电相关寄存器完整列表
1. 控制寄存器 (I²C 地址 0xE8)
1.1 VWPC_CTL0 (VWPC 控制寄存器) — 地址 0x14
此寄存器是无线充电功能的核心控制寄存器，用于使能或关闭无线充输出 MOS 管。
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX0En_Vwpc_mos无线充 MOS 输出通路使能1: enable (开启输出)0: disable (关闭输出)R/W1
操作示例：
c// 开启无线充输出
void WPC_Enable(void) {
    uint8_t reg = I2C_ReadByte(0xE8, 0x14);
    reg |= (1 << 0);      // 设置 Bit 0 为 1
    I2C_WriteByte(0xE8, 0x14, reg);
}

// 关闭无线充输出
void WPC_Disable(void) {
    uint8_t reg = I2C_ReadByte(0xE8, 0x14);
    reg &= ~(1 << 0);     // 设置 Bit 0 为 0
    I2C_WriteByte(0xE8, 0x14, reg);
}

2. 只读状态指示寄存器 (I²C 地址 0xEA)
这些寄存器用于读取无线充电时的电压和电流数据。
2.1 IVWPC_IADC_DAT0 (VWPC 输出电流寄存器 — 低字节) — 地址 0x58
表格Bit(s)NameDescriptionR/W7:0IVWPCADC[7:0]VWPC 无线充输入电流数据的低 8 bit R
2.2 IVWPC_IADC_DAT2 (VWPC 输出电流寄存器 — 高字节) — 地址 0x59
表格Bit(s)NameDescriptionR/W7:0IVWPCADC[15:8]VWPC 无线充输入电流数据的高 8 bitR
电流计算公式（文档原文）：
c// IVWPCADC = (high_byte << 8) | low_byte
float i_wpc_mA = IVWPCADC * 0.671387f;  // 单位：mA
读取代码示例：
cfloat WPC_Get_Current_mA(void) {
    uint8_t low  = I2C_ReadByte(0xEA, 0x58);
    uint8_t high = I2C_ReadByte(0xEA, 0x59);
    uint16_t adc_val = ((uint16_t)high << 8) | low;
    return adc_val * 0.671387f;
}
2.3 VWPCVADC_DAT0 (VWPC 电压寄存器 — 低字节) — 地址 0x60
表格Bit(s)NameDescriptionR/W7:0VWPCVADC[7:0]VWPC 无线充输入电压数据的低 8 bitR
2.4 VWPCVADC_DAT1 (VWPC 电压寄存器 — 高字节) — 地址 0x61
表格Bit(s)NameDescriptionR/W7:0VWPCVADC[15:8]VWPC 无线充输入电压数据的高 8 bitR
电压计算公式（文档原文）：
c// VWPCVADC = (high_byte << 8) | low_byte
float v_wpc_mV = VWPCVADC * 1.611328f;  // 单位：mV
读取代码示例：
cfloat WPC_Get_Voltage_mV(void) {
    uint8_t low  = I2C_ReadByte(0xEA, 0x60);
    uint8_t high = I2C_ReadByte(0xEA, 0x61);
    uint16_t adc_val = ((uint16_t)high << 8) | low;
    return adc_val * 1.611328f;
}

3. NTC2 PIN — 无线充状态指示（非寄存器，硬件引脚）
虽然这不是一个寄存器，但它是监控无线充状态的最重要方式。文档明确指出 IP5561 通过 NTC2 PIN (33 PIN) 输出的电平脉冲来指示无线充电状态。
表格状态NTC2 (33PIN) 电平输出上电3次 1HZ 脉冲无线充异常持续 1HZ 脉冲充电完成低电平充电中0.5HZ 脉冲待机高电平

4. 汇总表
表格寄存器名I²C 地址寄存器地址功能类型公式VWPC_CTL00xE80x14无线充 MOS 输出使能控制R/WBit 0 = 1 开启，= 0 关闭IVWPC_IADC_DAT00xEA0x58无线充输出电流低 8 位RIVWPC_IADC_DAT20xEA0x59无线充输出电流高 8 位RI(mA) = ADC × 0.671387VWPCVADC_DAT00xEA0x60无线充输入电压低 8 位RVWPCVADC_DAT10xEA0x61无线充输入电压高 8 位RV(mV) = ADC × 1.611328
5. 操作注意事项

读-修改-写规则：修改 0xE8/0x14 寄存器时，必须先读取原值，修改目标 bit 后写入，不要影响其他位。

INT 引脚时序：所有 I²C 读写操作必须在 INT 引脚拉高并稳定 100ms 之后进行。

异常处理：当检测到无线充异常（NTC2 输出 1Hz 脉冲）时，可先关闭 VWPC MOS（写 0x14 Bit 0 = 0），等待 2 秒后再重新开启。