IP5561 全部寄存器详细说明

芯片型号：英集芯 IP5561
I²C 设备地址：两组可选

组1：写 0xE8，读 0xE9
组2：写 0xEA，读 0xEB
寄存器格式：8 位寄存器地址，8 位寄存器数据，高位在前 (MSB)
重要规则：所有写操作必须遵循 读-修改-写 规则，仅修改目标 bit 位

总览：寄存器分组
表格地址组寄存器类型说明0xE8可读写控制寄存器充放电控制、快充协议配置、电压电流设置、NTC控制等0xEA可读写控制寄存器充电恒压、线补、过压、低电关机、按键状态、软件关机等0xEA只读状态寄存器电量、电压、电流、温度、系统状态、快充状态指示等

第一部分：可读写控制寄存器 (I²C 地址 0xE8)
1. SYS_CTL0 (Boost 和 Charger 使能寄存器) — 0x00
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3:2En_C2B_Det拔掉输入充电是否自动转 boost 输出1X: 系统不转 Boost 直接进入待机00: 无论输出是否有负载，自动开启 BoostR/W001En_BoostBoost 输出使能 R/W10En_ChargerCharger 充电使能R/W1

2. SYS_CTL1 (轻载关机控制寄存器) — 0x03
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WXX5:4Set_Ilow_Time轻载关机时间设置00: 8S, 01: 16S, 10: 32S, 11: 64SR/W103En_Ipow_Low轻载关机选择 VSYS 功率使能R/W02En_Isys_Low轻载关机选择 VSYS 电流使能 R/W11:0Reserved保留R/WXX

3. CHG_CTL1 (9V 输入充电欠压环路电压控制) — 0x0C
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WXX5:39V_Uvloop充电 9V 欠压环路电压000: 7.98V, 001: 8.13V, 010: 8.43V, 011: 8.50V100: 8.58V, 101: 8.65V, 110: 8.73V, 111: 8.80V R/W1002:0Reserved保留R/WXX

4. CHG_CTL2 (5V 输入充电欠压环路电压控制) — 0x0D
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WXX5:3Ppath_Uvloop充电同充同放环路电压000: 4.46V, 001: 4.54V, 010: 4.7V, 011: 4.75V100: 4.79V, 101: 4.83V, 110: 4.88V, 111: 4.92VR/W1112:05V_Uvloop充电 5V 欠压环路电压000: 4.46V, 001: 4.54V, 010: 4.7V, 011: 4.75V100: 4.79V, 101: 4.83V, 110: 4.88V, 111: 4.92V R/W001

5. SYS_CTL2 (轻载关机控制) — 0x0F
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3Lowcur_Off_Act退出常开 N 小时按键方式选择0: 短按, 1: 和进入常开 N 小时按键方式一样R/W02Lowcur_On_Act进入常开 N 小时按键方式选择0: 连续短按两次, 1: 长按 2S R/W01Dsb_Ahort连续两次短按是否屏蔽短按使能0: 不屏蔽短按, 1: 屏蔽短按R/W10Reserved保留R/WX

6. VOUT_CTL0 (VOUT 控制寄存器) — 0x10
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3En_Vout_DcpVOUT 口普通 5V DCP 使能R/W12En_Vout_qcVOUT 口快充使能 R/W11En_Vout_detVOUT 负载检测使能 (不控制 DMDP 检测)R/W10En_Vout_mosVOUT MOS 输出通路使能R/W1

7. VOUT_CTL1 (VOUT 控制寄存器) — 0x13
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3:2Set_Vout_Ilow_TimeVOUT 口径载关输出口的时间设置00: 2S, 01: 4S, 10: 8S, 11: 16SR/W111En_Vout_Vhgilow充电状态轻载关 VOUT 使能R/W10En_Vout_Chgilow放电状态轻载关 VOUT 使能 (多口才起作用)R/W1

8. VOUT_CTL2 (VOUT 控制寄存器) — 0x1C
表格Bit(s)NameDescriptionR/WRESET7:1Reserved保留R/WXX0En_Vout_DmDp_DetVOUT_DM DP 负载检测使能R/W1

9. VWPC_CTL0 (VWPC 控制寄存器 — 无线充) — 0x14
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX0En_Vwpc_mos无线充 MOS 输出通路使能1: enable, 0: disableR/W1

10. VBUS_CTL0 (VBUS 控制寄存器) — 0x18
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3En_Vbus_DcpVBUS 口普通 5V DCP 使能R/W12En_Vbus_QcVBUS 口快充使能 (DPDM 协议) R/W11Reserved保留R/WX0En_Vbus_MosVBUS MOS 输出通路使能R/W1

11. VBUS_CTL1 (VBUS 控制寄存器) — 0x1B
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3:2Set_Vbus_Ilow_TimeVBUS 口径载关输出口的时间设置00: 2S, 01: 4S, 10: 8S, 11: 16SR/W111Reserved保留R/WXX0En_Vbus_Chgilow放电状态轻载关 VBUS 使能R/W1

12. CHG_CTL1 (充电超时控制) — 0x21
表格Bit(s)NameDescriptionR/WRESET7:5Reserved保留R/WXX4:2Set_chg_timeCharge 超时设置0: disable, 1: 12h, 2: 18h, 3: 24h, 4: 30h, 5: 36h, 6: 42h, 7: 48hR/W1001:0Reserved保留R/WXX

13. CHG_CTL2 (充电超时控制) — 0x22
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WX6:4Set_cv_timeCharge CV 超时时间设置0: disable, 1: 2h, 2: 3h, 3: 4h, 4: 5h, 5: 6h, 6: 7h, 7: 8hR/W0113:1Set_tk_timeCharge 涓流超时时间设置0: disable, 1: 2h, 2: 3h, 3: 4h, 4: 5h, 5: 6h, 6: 7h, 7: 8hR/W0010Reserved保留R/WX

14. PPATH_CTL0 (同充同放控制) — 0x24
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WXX5En_Ppath_VbusVBUS 输入 5V 同充同放使能R/W14:0Reserved保留R/WXX

15. SYS_CTL3 (输入快充控制) — 0x25
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3En_Vbus_SinkqcVBUS 输入快充使能R/W12:0Reserved保留R/WXX

16. CHG_CTL3 (同充同放电流控制) — 0x26
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:0Chg_Iset_Ppath同充同放输入电流设置I = 25mA * N，校准值约 0.5AR/WXX

17. CHG_CTL4 (VBUS 5V 充电电流控制) — 0x29
表格Bit(s)NameDescriptionR/WRESET7Chg_Iset_Ppath_EN同充同放用同充同放电流 (0x26) 作为输入电流使能 R/W16:0Chg_Iset_Vbus5v5V VBUS 输入电流设置I = 25mA * N，校准值约 2.9AR/WXX

18. CHG_CTL5 (VBUS 9V 充电电流控制) — 0x6F
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:0Chg_Iset_Vbus9v9V VBUS 输入电流设置I = 25mA * N，校准值约 2A R/WXX

19. SYS_CTL4 (按键关机) — 0x31
表格Bit(s)NameDescriptionR/WRESET7:3Reserved保留R/WXX2En_Long_Wk长按 2S 按键唤醒使能R/W01:0Set_Key按键关机00: disable 按键关机使能01: 短按按键关机10: 连续短按两次关机11: 长按 2S 按键关机R/W10

20. SYS_CTL5 (常开 N 小时和 VSET 检测使能) — 0x33
表格Bit(s)NameDescriptionR/WRESET7:6Set_Lowcur_Time常开 N 小时时间设置00: 2H, 01: 4H, 10: 6H, 11: 8HR/W005En_Lowcur常开 N 小时使能 R/W04En_Vset_Det电芯电压 VSET 使能1: VSET 检测设置电芯充满电压0: 寄存器设置充满电压 R/W13:0Reserved保留R/WXX

21. POW_LOW (轻载关机功率设置) — 0x44
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WXX5:0Set_Pow_lowIsys 轻载关机输出功率阈值设置POW_LOW = 17.7 * N mW R/WXX

22. ISYS_LOW (轻载关机电流设置) — 0x45
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WXX5:0Set_isy_lowIsys 轻载关机 ADC 输出电流阈值设置ISYS_LOW = 5.4 * N mA R/WXX

23. VOUT_IMOSLOW (VOUT 口径载电流设置) — 0x49
表格Bit(s)NameDescriptionR/WRESET7:0Set_vout_imoslowVOUT 口输出电流轻载关 MOS 电流阈值设置IMOS_LOW = 2.68 * N mA R/WXX

24. VBUS_IMOSLOW (VBUS 口径载电流设置) — 0x4B
表格Bit(s)NameDescriptionR/WRESET7:0Set_vbus_imoslowVBUS 口输出电流轻载关 MOS 电流阈值设置IMOS_LOW = 2.68 * N mAR/WXX

25. FCAP (电量计容量设置) — 0x4C
表格Bit(s)NameDescriptionR/WRESET7:0FCAP电量计总容量设置FCAP = 385 * N mAH默认由 FCAP PIN 外挂下拉电阻设定电芯容量 R/WXX

26. MFP_CTL0 (LED4/LED5 功能选择) — 0x65
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3:2LED5LED5 GPIO 功能选择00: LED5, 01: LED5_ADC 功能R/W001:0LED4LED4 GPIO 功能选择00: LED4, 01: LED4_ADC 功能R/W00

27. EN_FCAP (外部容量使能) — 0x78
表格Bit(s)NameDescriptionR/WRESET7En_FcapFCAP 外部容量使能1: 使用外部电阻设置电芯容量0: 使用内部寄存器设置电芯容量 R/W16:0Reserved保留R/WXX

28. NTC1_CTL0 (NTC1 电流选择) — 0xF6
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WXX5:4NTC1_20uA_SelNTC1 PIN 电流控制选择00: 内部状态机自动控制01: 内部状态机自动控制10: 80uA11: 20uA R/W003:0Reserved保留R/WXX

29. NTC_CTL1 (NTC1 控制寄存器) — 0xFD
表格Bit(s)NameDescriptionR/WRESET7En_chg_ml充电 NTC1 中低温 (5℃左右) 充电电流减半使能R/W06En_chg_mh充电 NTC1 中高温 (41℃左右) 充电电流减半使能R/W05En_boost_lt放电 NTC1 低温 (-20℃左右) 关 boost 使能R/W14En_boost_ht放电 NTC1 高温 (60℃左右) 关 boost 使能R/W13En_chg_lt充电 NTC 低温 (0℃左右) 关 charge 使能R/W12En_chg_ht充电 NTC1 高温 (45℃左右) 关 charge 使能R/W11Reserved保留R/WX0En_ntc1NTC1 使能1: enable (硬件自动模式)0: disable (MCU 手动读取模式)R/W1

30. SINK_QC_EN (输入快充使能) — 0x81
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WX3En_sink_afcSINK AFC 三星快充使能R/W12En_sink_fcpSINK FCP 华为快充使能R/W11En_sink_qcSINK 输入快充使能 R/W10Reserved保留R/WX

31. SYS_CTL6 (线补使能) — 0x84
表格Bit(s)NameDescriptionR/WRESET7:2Reserved保留R/WXX1Set_dcp_appleDCP 苹果模式选择1: 2.4A, 0: 2.1AR/W10En_Lc线补使能 R/W1

32. SRC_QC_EN (输出快充使能) — 0x85
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WX5En_src_scpSRC SCP 快充使能 R/W14En_src_fcpSRC FCP 快充使能 R/W13En_src_afcSRC AFC 三星快充使能R/W12En_src_qc3.0SRC QC3.0 快充使能 R/W11En_src_qc2.0SRC QC2.0 快充使能 R/W10En_src_dcp_appleSRC 苹果模式使能R/W1

33. SRC_QC_EN2 (输出快充使能) — 0x86
表格Bit(s)NameDescriptionR/WRESET7:6Src_at_same同充同放状态下输出 DCP 模式选择11: 短接, 10: 浮空, 0X: 自动R/W115:0Reserved保留R/WXX

34. BST_VSET (输出电压设置) — 0xAA
表格Bit(s)NameDescriptionR/WRESET7:3Reserved保留R/WXX2BST_VSET_RBOOST 输出电压控制方式选择1: 使用寄存器 TRSEL_REG 值设定0: 由硬件状态自动控制R/W01:0TRSEL_REGTRSEL_REG[9:8]BOOST_VSET = 3.2V + 10mV * TRSEL_REG[9:0]最大输出电压不要超过 12VR/WXX

35. BST_VSET (输出电压设置) — 0xAB
表格Bit(s)NameDescriptionR/WRESET7:0TRSEL_REGTRSEL_REG[7:0]BOOST_VSET = 3.2V + 10mV * TRSEL_REG[9:0]R/WXX

36. BST_5V (输出电压设置) — 0xAD
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3:0TRSEL_REG5V 档位输出电压设定4.6V + 0.1 * N出厂校准到 5VR/WXX

37. BST_12V_9V (输出电压设置) — 0xAE
表格Bit(s)NameDescriptionR/WRESET7:4TRSEL_REG12V 档位输出电压设定11.2V + 0.1 * N出厂校准到 12VR/WXX3:0TRSEL_REG9V 档位输出电压设定8.2V + 0.1 * N出厂校准到 9VR/WXX

38. VOUT_5V (5V 输出电流设置) — 0xB1
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:0TRSEL_REG50mA * N出厂校准到 3.3AR/WXX

39. VOUT_9V (9V 输出电流设置) — 0xB3
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:0TRSEL_REG50mA * N出厂校准到 2.3AR/WXX

40. VOUT_12V (12V 输出电流设置) — 0xB4
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:0TRSEL_REG50mA * N出厂校准到 1.7AR/WXX

41. VBUS_5V (5V 输出电流设置) — 0xB9
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:0TRSEL_REG50mA * N出厂校准到 3.3AR/WXX

42. VBUS_9V (9V 输出电流设置) — 0xBB
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:0TRSEL_REG50mA * N出厂校准到 2.3AR/WXX

43. VBUS_12V (12V 输出电流设置) — 0xBC
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:0TRSEL_REG50mA * N出厂校准到 1.7AR/WXX

44. TEMP_LP (IC 内部温度环设置) — 0xC7
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WX6:2TSEL温度环档位选择10000: 70°C, 10100: 75°C, 11001: 80°C, 11110: 85°C00000: 90°C, 00100: 95°C, 01001: 100°C, 01110: 105°CR/W000001:0Reserved保留R/WXX

45. TYPEC_CTL0 (PD 控制寄存器) — 0xD0
表格Bit(s)NameDescriptionR/WRESET7:5Reserved保留R/WXX4:3CC_src_isetTYPEC CC 上拉能力00: default, 01: 1.5A, 10: 3AR/W102:0Reserved保留R/WXX

46. TYPEC_CTL0 (PD 控制寄存器) — 0xD1
表格Bit(s)NameDescriptionR/WRESET7:6Reserved保留R/WXX5:4CC_MODE_SELTYPEC CC 模式选择00: UFP, 01: DFP, 11: DRPR/W113:0Reserved保留R/WXX

47. TYPEC_CTL1 (PD 控制寄存器) — 0xD4
表格Bit(s)NameDescriptionR/WRESET7:1Reserved保留R/WXX0EN_PD_SRCPD 快充使能1: enable, 0: disableR/W1

48. TYPEC_CTL2 (PD 控制寄存器) — 0xD5
表格Bit(s)NameDescriptionR/WRESET7Reserved保留R/WXX6:5PD_SINK_VMAXPD SINK 输入最大电压设置00: 5V, 01: 9VR/W014:0Reserved保留R/WXX

第二部分：可读写控制寄存器 (I²C 地址 0xEA)
49. VBUS_OV (输入过压寄存器) — 0x01
表格Bit(s)NameDescriptionR/WRESET7:6VBUS_OVVBUS 输入过压设置00: 6V, 01: 10V, 10: 14.5V, 11: 16VR/W015:0Reserved保留R/WXX

50. BATLOW (模拟电芯低电关机电压) — 0x03
表格Bit(s)NameDescriptionR/WRESET7:1Reserved保留R/WXX0BATLOW_SET_A模拟 BATLOW 电压设置 (下降沿保护电压 - 上升沿恢复电压)0: 2.9V - 3.0V1: 3.0V - 3.1VR/W0

51. GPIO_20UA_EN (GPIO 电流输出使能) — 0x19
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3LED5_20UA_ENLED5 20uA 输出使能R/W02LED4_20UA_ENLED4 20uA 输出使能R/W01LED3_20UA_ENLED3 20uA 输出使能R/W00NTC1_20UA_ENNTC1 20uA/80uA 输出使能R/W1

52. LC_SET (线补选择寄存器) — 0x30
表格Bit(s)NameDescriptionR/WRESET7:1Reserved保留R/WXX0Lc_set线补电压选择1: 300mV@2A, 0: 150mV@2AR/W0

53. CHG_CTL6 (充电恒压充电电压设置) — 0x3A
表格Bit(s)NameDescriptionR/WRESET7:4Reserved保留R/WXX3:2VSET_BAT充电恒压电压设置00: 4.20V, 01: 4.30V, 10: 4.35V, 11: 4.40V R/W001:0R_CV充电恒压加压电压00: 加 0mV, 01: 加 14mV, 10: 加 28mV, 11: 加 42mVR/W10

54. KEYIN_STATE (按键状态指示 — 可写清0) — 0xF4
注意：此寄存器虽归类于状态指示，但部分 bit 为 R/W，写 1 可以清除标志位。
表格Bit(s)NameDescriptionR/W7On_off_2short按键连续短按两次标志位，需写 1 清 0R/W6On_off_long按键长按 2S 标志位，需写 1 清 0R/W5On_off_short按键短按标志位，需写 1 清 0R/W4:0Reserved保留R/W

55. SOC_CAP_SET (电芯电量设置) — 0x87
表格Bit(s)NameDescriptionR/W7:0SOC_CAP_SET电芯百分比电量控制寄存器可直接向寄存器值写相应的百分比电量数据 R/W

56. FORCE_STANDBY (软件关机) — 0x86
表格Bit(s)NameDescriptionR/W7Force_Standby写 1 可以使 IP5561 进入休眠状态 R/W6:0Reserved只读R

第三部分：只读状态指示寄存器 (I²C 地址 0xEA)
57. BATVADC_DAT0 (VBAT 电压寄存器低字节) — 0x50
表格Bit(s)NameDescription7:0BATVADC[7:0]BATVADC 数据的低 8 bit，VBAT PIN 的电压

58. IVBUS_IADC_DAT0 (输入电流寄存器低字节) — 0x54
表格Bit(s)NameDescription7:0IVBUSADC[7:0]充电输入电流数据的低 8 bit

59. IVBUS_IADC_DAT2 (输入电流寄存器高字节) — 0x55
表格Bit(s)NameDescription7:0IVBUSADC[15:8]充电输入电流数据的高 8 bitIin = IVBUSADC * 0.671387mA

60. IVOUT_IADC_DAT0 (VOUT 输出电流寄存器低字节) — 0x56
表格Bit(s)NameDescription7:0IVOUTADC[7:0]VOUT 输出电流数据的低 8 bit

61. IVOUT_IADC_DAT1 (VOUT 输出电流寄存器高字节) — 0x57
表格Bit(s)NameDescription7:0IVOUTADC[15:8]VOUT 输出电流数据的高 8 bitIOUT = IVOUTADC * 0.671387mA

62. IVWPC_IADC_DAT0 (VWPC 无线充输出电流寄存器低字节) — 0x58
表格Bit(s)NameDescription7:0IVWPCADC[7:0]VWPC 无线充输入电流数据的低 8 bit

63. IVWPC_IADC_DAT2 (VWPC 无线充输出电流寄存器高字节) — 0x59
表格Bit(s)NameDescription7:0IVWPCADC[15:8]VWPC 无线充输入电流数据高 8 bitIVWPC = IVWPCADC * 0.671387mA

64. IBUS_IADC_DAT0 (VBUS 输出电流寄存器低字节) — 0x5A
表格Bit(s)NameDescription7:0IVBUSADC[7:0]VBUS 输出电流数据的低 8 bit

65. IBUS_IADC_DAT2 (VBUS 输出电流寄存器高字节) — 0x5B
表格Bit(s)NameDescription7:0IVBUSADC[15:8]VBUS 输出电流数据的高 8 bitIVBUS = IVBUSADC * 0.671387mA

66. VOUTVADC_DAT0 (VOUT 电压寄存器低字节) — 0x5E
表格Bit(s)NameDescription7:0VOUTVADC[7:0]VOUT 输出电压数据的低 8 bit

67. VOUTVADC_DAT1 (VOUT 电压寄存器高字节) — 0x5F
表格Bit(s)NameDescription7:0VOUTVADC[15:8]VOUT 输出电压数据的高 8 bitVOUT = VOUTVADC * 1.611328mV

68. VWPCVADC_DAT0 (VWPC 无线充电压寄存器低字节) — 0x60
表格Bit(s)NameDescription7:0VWPCVADC[7:0]VWPC 无线充输入电压数据的低 8 bit

69. VWPCVADC_DAT1 (VWPC 无线充电压寄存器高字节) — 0x61
表格Bit(s)NameDescription7:0VWPCVADC[15:8]VWPC 无线充输入电压数据的高 8 bitVWPC = VWPCVADC * 1.611328mV

70. VBUSVADC_DAT0 (VBUS 电压寄存器低字节) — 0x62
表格Bit(s)NameDescription7:0VBUSVADC[7:0]VBUS 输入输出电压数据的低 8 bit

71. NTC1VADC_DAT0 (NTC1 电压寄存器低字节) — 0x64
表格Bit(s)NameDescription7:0NTC1VADC[7:0]NTC1 电压数据的低 8 bit

72. NTC1VADC_DAT1 (NTC1 电压寄存器高字节) — 0x65
表格Bit(s)NameDescription7:0NTC1VADC[15:8]NTC1 电压数据的高 8 bitNTC1 = NTC1VADC * 0.26855mV

73. LED4VADC_DAT1 (LED4 电压寄存器高字节) — 0x69
表格Bit(s)NameDescription7:0LED4VADC[15:8]LED4 电压数据的高 8 bitLED4 = LED4VADC * 0.26855mV

74. IBATIADC_DAT0 (BAT 端电流寄存器低字节) — 0x6E
表格Bit(s)NameDescription7:0IBATIADC[7:0]电芯端电流 IBATIADC 数据的低 8 bit

75. IBATIADC_DAT1 (BAT 端电流寄存器高字节) — 0x6F
表格Bit(s)NameDescription7:0IBATIADC[15:8]电芯端电流 IBATIADC 数据的高 8 bitIBAT = IBATIADC * 1.6785mA，电流不区分正负方向

76. ISYS_IADC_DAT0 (IVSYS 端电流寄存器低字节) — 0x70
表格Bit(s)NameDescription7:0ISYIADC[7:0]IVSYS 端电流数据的低 8 bit 

77. IVSYS_IADC_DAT1 (IVSYS 端电流寄存器高字节) — 0x71
表格Bit(s)NameDescription7:0IVSYSIADC[15:8]IVSYS 端电流数据的高 8 bitIVSYS = ISYSVADC * 0.671387mA，电流不区分正负方向 

78. VSYS_POW_DAT0 (VSYS 端功率寄存器低字节) — 0x74
表格Bit(s)NameDescription7:0VSYS_POW_ADC[7:0]VSYS 端功率 ADC 数据的低 8 bit 

79. VSYS_POW_DAT1 (VSYS 端功率寄存器高字节) — 0x75
表格Bit(s)NameDescription7:0VSYS_POW_ADC[15:8]VSYS 端功率 ADC 数据的高 8 bitVSYS_POW = VSYS_POW_ADC * 4.431mW

80. SOC_CAP_DATA (电芯电量数据寄存器) — 0x7B
表格Bit(s)NameDescription7:0SOC_CAP电芯百分比电量数据 (%) 

81. FCP_STATUS (输出 FCP 指示寄存器) — 0xA1
表格Bit(s)NameDescription7:6FCP_VSELSRC FCP 电压00: 5V, 01: 9V, 10: 12V5:0Reserved保留

82. STATUS_SRC0 (输出快充状态指示 — VOUT) — 0xA4
表格Bit(s)NameDescription7:4Reserved保留3:0VOUT_STATEVOUT 输出快充协议标志位0001: DCP0110: QC 握手 OK1001: QC2 (9V 12V)1010: QC31100: FCP/SCP1101: AFC

83. STATUS_SRC1 (输出快充状态指示 — VBUS) — 0xA5
表格Bit(s)NameDescription7Reserved保留6:4CHAL_STATESRC 快充所在的输出口 (DPDM 协议)000: 无, 001: VOUT, 100: VBUS 3:0VBUS_STATEVBUS 输出快充协议标志位0001: DCP0110: QC 握手 OK1001: QC2 (9V 12V)1010: QC31100: FCP/SCP1101: AFC

84. STATUS_SRC2 (输出快充状态指示) — 0xA8
表格Bit(s)NameDescription7LOW_VSET_OK低压快充标志位6:0Reserved保留

85. AFC_STATUS (输出 AFC 指示寄存器) — 0xAF
表格Bit(s)NameDescription7Reserved保留5:4AFC_VSELSRC AFC 输出电压00: 5V, 01: 9V, 11: 12V3:0Reserved保留

86. PD_STATE0 (系统状态指示寄存器) — 0xB1
表格Bit(s)NameDescription7:1Reserved保留0Sink_pd_OkPD SINK 输入连接标志位1: 有效, 0: 无效

87. PD_STATE1 (系统状态指示寄存器) — 0xC2
表格Bit(s)NameDescription7Src_Pd_OkPD SRC 输出连接标志位1: 有效, 0: 无效6:0Reserved保留

88. PD_STATE2 (系统状态指示寄存器) — 0xC3
表格Bit(s)NameDescription7:1Reserved保留0Src_Pps_OkPPS SRC 输出连接标志位1: 有效, 0: 无效

89. SYS_STATE0 (系统状态指示寄存器) — 0xC4
表格Bit(s)NameDescription7:6Reserved保留5VBUSOVVBUS 输入过压标志1: VBUS 输入过压, 0: VBUS 输入没有过压 4VBUSOKVBUS 电压有效标志 (TYPEC 充电放电该 bit 都会有效)1: VBUS 有电, 0: VBUS 没电3:0Reserved保留

90. SYS_STATE1 (系统状态指示寄存器) — 0xC5
表格Bit(s)NameDescription7:3Reserved保留2VBATLOW电芯电压 VBATLOW 标志1: 电芯低电有效, 0: 电芯电压没有低电1VSYS_OVVSYS 过压标志1: VSYS 过压, 0: VSYS 没有过压

91. SYS_STATE2 (系统状态指示寄存器) — 0xCD
表格Bit(s)NameDescription7Reserved保留6Src_qc_ok输出快充有效标志位1: 有效, 0: 无效 5:0Reserved保留

92. SYS_STATE3 (系统状态指示寄存器) — 0xD0
表格Bit(s)NameDescription7Reserved保留6AT_SAME1: 同充同放状态, 0: 不在同充同放状态5Charge_en充电使能状态1: 充电使能已经打开, 0: 充电使能已经关闭4Boost_en放电 boost 使能状态1: 放电 boost 使能已经打开, 0: 放电 boost 使能已经关闭3Reserved保留2:0Sys_state当前系统状态000: 待机状态001: 开启 boost 延时状态010: 开启 boost 状态011: 关机延时状态100: 开启 charge 延时状态101: charge 开启状态110: charge 转 boost 延时状态111: charge 转 boost 时，等待输出口检测结果

93. SYS_STATE4 (系统状态指示寄存器) — 0xD4
表格Bit(s)NameDescription7:3Reserved保留2:0Chg_qc_state充电输入快充状态000: 待机状态001: 处于延时 0.5S 申请快充的状态中010: 未申请快充011: 快充输入100: 输入快充待申请状态 (如涓流充电)101: 同充同放状态

94. LOWCUR_STATE (系统状态指示寄存器) — 0xE1
表格Bit(s)NameDescription7Lowcur_state常开 N 小时状态1: 进入了常开 N 小时模式, 0: 未进入6:0Reserved保留

95. CHG_STATE1 (系统状态指示寄存器) — 0xE8
表格Bit(s)NameDescription7Vbus_mosi_stateC 口输入 MOS 状态1: 开启, 0: 关闭6Reserved保留5:4Vchg_state00: 5V 充电, 01: 7V 充电, 10: 9V 充电3:0Reserved保留

96. CHG_STATE2 (系统状态指示寄存器) — 0xE9
表格Bit(s)NameDescription7SINK_QC_OK输入快充有效标志位 (DM DP 快充和 PD 快充均有效)1: 有效, 0: 无效6:4chg_state充电状态000: 未充电状态001: 充电状态010: 充电状态011: 充电状态100: 恒压断开检测电芯电压101: 充满状态110: 充电超时状态3Charge_en_state充电使能有效标准1: 正在充电状态, 0: 未在充电状态2:0Reserved保留

97. MOS_STATE (输出 MOS 状态指示寄存器) — 0xEB
表格Bit(s)NameDescription7At_same同充同放标志位0: 未在同充同放, 1: 在同充同放6Mos_vbus_stateVBUS 口输出 MOS 状态0: 关闭状态, 1: 开启状态5Mos_vwpc_stateVWPC 无线充口输出 MOS 状态0: 关闭状态, 1: 开启状态4Mos_vout_stateVOUT 口输出 MOS 状态0: 关闭状态, 1: 开启状态3Src_qc_ok输出快充有效标志位1: 有效, 0: 无效 2:0Reserved保留

98. ILOW_STATE (系统轻载状态指示寄存器) — 0xF2
表格Bit(s)NameDescription7Reserved保留6Isys_lowIsys 输出电流轻载标志位1: 有效, 0: 无效 5Reserved保留4Pow_lowIsys 输出功率轻载标志位1: 有效, 0: 无效 3:0Reserved保留

99. TYPEC_STATE (系统轻载状态指示寄存器) — 0xF3
表格Bit(s)NameDescription7cc_src_okCC_SRC_OK, TYPEC 连接成 SRC (作为输出)1: 有效, 0: 无效6cc_sink_okCC_SINK_OK, TYPEC 连接成 SINK (作为输入)1: 有效, 0: 无效5:0Reserved保留

100. KEYIN_STATE (按键状态指示寄存器) — 0xF4
注意：此寄存器部分 bit 为 R/W，写 1 可清除标志。
表格Bit(s)NameDescriptionR/W7On_off_2short按键连续短按两次标志位，需写 1 清 0R/W6On_off_long按键长按 2S 标志位，需写 1 清 0R/W5On_off_short按键短按标志位，需写 1 清 0R/W4:0Reserved保留R/W

101. NTC1_STATE (NTC1 和输出 MOS 电流状态指示) — 0xFB
表格Bit(s)NameDescription7Ntc1_htNTC1 高温标志位 (1: 有效)6Ntc1_mhtNTC1 中高温标志位 (1: 有效)5Ntc1_mltNTC1 中低温标志位 (1: 有效)4Ntc1_ltNTC1 低温标志位 (1: 有效)3Mos_vbus_ilowVBUS 输出口轻载标志位 (1: 有效) 2Mos_vwpc_ilowVWPC 无线充输出口轻载标志位 (1: 有效)1Mos_vout_ilowVOUT 输出口轻载标志位 (1: 有效) 0Reserved保留

102. OCP_STATE (系统过流状态指示寄存器) — 0xFC
表格Bit(s)NameDescriptionR/W7:3Reserved保留R/W2Boost_uvBoost 过流标志位，需写 1 清 0当检测到第一次过流信号时，先写 1 清 0，然后再读，如果 600ms 内连续检测到两次以上的过流信号就认为过流信号有效01Reserved保留R/W0Boost_scdtBoost 短路标志位，需写 1 清 0当检测到第一次短路信号时，先写 1 清 0，然后再读，如果 600ms 内连续检测到两次以上的短路信号就认为短路信号有效0

开发注意事项

读-修改-写规则：所有写操作前必须先读取原值，仅修改目标 bit，避免影响其他未开放的 bit。
默认值差异：文档强调“不同批次 IC 的寄存器默认值可能存在差异”，默认值仅作参考，请以实际读回为准。
I²C 地址确认：在读写寄存器前，需要确认该寄存器对应的正确 I²C 地址是 0xE8 还是 0xEA。
INT 引脚时序：必须在 INT 引脚拉高并保持 100ms 之后才能开始 I²C 读写。
Reserved 位：保留位不可随意写入数据，不可改变原有值。
