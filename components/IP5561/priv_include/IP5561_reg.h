/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

/* IP5561 I2C Device Addresses
 *
 * IP5561 has two I2C address groups for different register sets:
 * - 0xE8 (write) / 0xE9 (read): Control registers
 * - 0xEA (write) / 0xEB (read): Status and ADC registers
 */

/* I2C Address 1: Control Registers (0xE8/0xE9) */
#define IP5561_I2C_ADDR_CTRL_WRITE  0xE8
#define IP5561_I2C_ADDR_CTRL_READ   0xE9

/* I2C Address 2: Status and ADC Registers (0xEA/0xEB) */
#define IP5561_I2C_ADDR_STAT_WRITE  0xEA
#define IP5561_I2C_ADDR_STAT_READ   0xEB

/* ========== Control Register Map (I2C: 0xE8/0xE9) ========== */

/* System Control Registers */
#define IP55XX_SYS_CTL0         0x00    /* System Control 0 - Boost/Charger Enable */
#define IP55XX_SYS_CTL1         0x03    /* System Control 1 - Low Power Shutdown */
#define IP55XX_SYS_CTL2         0x0F    /* System Control 2 - Button Control */
#define IP55XX_SYS_CTL3         0x25    /* System Control 3 - Input Fast Charge Control */
#define IP55XX_SYS_CTL4         0x31    /* System Control 4 - Button Power Off */
#define IP55XX_SYS_CTL5         0x33    /* System Control 5 - Always-On N Hours & VSET Detect */

/* Charging Control Registers */
#define IP55XX_CHG_CTL0         0x01    /* Charge Control 0 */
#define IP55XX_CHG_CTL1         0x0C    /* Charge Control 1 - 9V Input UV Loop */
#define IP55XX_CHG_CTL2         0x0D    /* Charge Control 2 - 5V Input UV Loop */
#define IP55XX_CHG_CTL3         0x26    /* Charge Control 3 */
#define IP55XX_CHG_CTL4         0x29    /* Charge Control 4 - VBUS 5V Charge Current */
#define IP55XX_CHG_CTL5         0x6F    /* Charge Control 5 - VBUS 9V Charge Current (I2C 0xE8) */

/* Charging Timeout Registers */
#define IP55XX_CHG_TMO_CTL1     0x21    /* Charge Timeout Control 1 */
#define IP55XX_CHG_TMO_CTL2     0x22    /* Charge Timeout Control 2 */

/* VBUS Control Registers */
#define IP55XX_VBUS_CTL0        0x18    /* VBUS Control 0 */
#define IP55XX_VBUS_CTL1        0x1B    /* VBUS Control 1 */

/* VOUT Control Registers */
#define IP55XX_VOUT_CTL0        0x10    /* VOUT Control 0 */
#define IP55XX_VOUT_CTL1        0x13    /* VOUT Control 1 */
#define IP55XX_VOUT_CTL2        0x1C    /* VOUT Control 2 */

/* VWPC (Wireless Power) Control */
#define IP55XX_VWPC_CTL0        0x14    /* VWPC Control 0 */

/* Power Path Control */
#define IP55XX_PPATH_CTL0       0x24    /* Power Path Control 0 */

/* Voltage/Current Monitoring */
#define IP55XX_POW_LOW          0x44    /* Power Low Byte */
#define IP55XX_POW_HIGH         0x45    /* Power High Byte */

/* NTC Control */
#define IP55XX_NTC_CTRL         0xF6    /* NTC Control */
#define IP55XX_NTC_ENABLE       0xFD    /* NTC Enable */

/* Fast Charge Control */
#define IP55XX_QC_CTRL0         0x81    /* Quick Charge Control 0 - SINK_QC_EN */
#define IP55XX_QC_CTRL1         0x84    /* Quick Charge Control 1 - SYS_CTL6/Line Comp */
#define IP55XX_QC_CTRL2         0x85    /* Quick Charge Control 2 - SRC_QC_EN */
#define IP55XX_PD_CTRL          0xD4    /* PD Protocol Control - TYPEC_CTL1 */
#define IP55XX_TYPEC_CTL0       0xD0    /* TYPEC PD Control 0 */
#define IP55XX_TYPEC_CTL1       0xD1    /* TYPEC PD Control 1 */
#define IP55XX_TYPEC_CTL2       0xD5    /* TYPEC PD Control 2 */
#define IP55XX_SRC_QC_EN2       0x86    /* SRC QC Enable 2 / MOS_CTRL / FORCE_STANDBY */

/* Boost Output Control */
#define IP55XX_BST_VSET_L       0xAA    /* Boost Output Voltage Setting Low Byte */
#define IP55XX_BST_VSET_H       0xAB    /* Boost Output Voltage Setting High Byte */
#define IP55XX_BST_5V           0xAD    /* Boost 5V Output Current Setting */
#define IP55XX_BST_12V_9V       0xAE    /* Boost 12V/9V Output Current Setting */
#define IP55XX_VOUT_5V          0xB1    /* VOUT 5V Output Current Setting */
#define IP55XX_VOUT_9V          0xB3    /* VOUT 9V Output Current Setting */
#define IP55XX_VOUT_12V         0xB4    /* VOUT 12V Output Current Setting */
#define IP55XX_VBUS_5V          0xB9    /* VBUS 5V Output Current Setting */
#define IP55XX_VBUS_9V          0xBB    /* VBUS 9V Output Current Setting */
#define IP55XX_VBUS_12V         0xBC    /* VBUS 12V Output Current Setting */

/* Light Load Current Threshold */
#define IP55XX_VOUT_IMOSLOW     0x49    /* VOUT Light Load MOS Current Threshold */
#define IP55XX_VBUS_IMOSLOW     0x4B    /* VBUS Light Load MOS Current Threshold */

/* Battery Capacity Configuration */
#define IP55XX_FCAP             0x4C    /* Battery Capacity Setting */
#define IP55XX_EN_FCAP          0x78    /* External Capacity Enable */
#define IP55XX_SOC_CAP_SET      0x87    /* Battery Percentage Setting */

/* Multi-Function Pin Control */
#define IP55XX_MFP_CTL0         0x65    /* LED4/LED5 Function Selection */

/* Temperature Loop */
#define IP55XX_TEMP_LP          0xC7    /* IC Internal Temperature Loop */

/* ========== Status and ADC Register Map (I2C: 0xEA/0xEB) ========== */

/* System Status Registers */
#define IP55XX_VBUS_OV          0x01    /* VBUS Over Voltage Setting */
#define IP55XX_BATLOW           0x03    /* Battery Low Voltage Threshold */
#define IP55XX_GPIO_20UA_EN     0x19    /* GPIO 20uA Current Output Enable */
#define IP55XX_CHG_STATE1       0xE8    /* Charging State 1 */
#define IP55XX_CHG_STATE2       0xE9    /* Charging State 2 */
#define IP55XX_MOS_STATE        0xEB    /* MOS Output State */
#define IP55XX_LOWCUR_STATE     0xE1    /* Light Load State */
#define IP55XX_ILOW_STATE       0xF2    /* System Light Load State */
#define IP55XX_TYPEC_STATE      0xF3    /* TYPEC State */
#define IP55XX_KEYIN_STATE      0xF4    /* Key Button State */
#define IP55XX_NTC1_STATE       0xFB    /* NTC1 and MOS Current State */
#define IP55XX_OCP_STATE        0xFC    /* Over Current State */
#define IP55XX_SYS_STATE0       0xC4    /* System State 0 */
#define IP55XX_SYS_STATE1       0xC5    /* System State 1 */
#define IP55XX_SYS_STATE2       0xCD    /* System State 2 */
#define IP55XX_SYS_STATE3       0xD0    /* System State 3 */
#define IP55XX_SYS_STATE4       0xD4    /* System State 4 */

/* Charging Voltage/Current Control */
#define IP55XX_CHG_VOLT_CFG     0x30    /* Charging Voltage Configuration */
#define IP55XX_CHG_VOLT_SET     0x3A    /* Charging Voltage Setting */

/* ADC Registers (I2C: 0xEA/0xEB) */
#define IP55XX_VBAT_ADC_L       0x50    /* Battery Voltage ADC Low Byte */
#define IP55XX_VBAT_ADC_H       0x51    /* Battery Voltage ADC High Byte */
#define IP55XX_VSYS_ADC_L       0x52    /* VSYS Voltage ADC Low Byte */
#define IP55XX_VSYS_ADC_H       0x53    /* VSYS Voltage ADC High Byte */
#define IP55XX_IBUS_IADC_L      0x54    /* Input Current (IVBUS) ADC Low Byte */
#define IP55XX_IBUS_IADC_H      0x55    /* Input Current (IVBUS) ADC High Byte */
#define IP55XX_VOUT_IADC_L      0x56    /* VOUT Current ADC Low Byte */
#define IP55XX_VOUT_IADC_H      0x57    /* VOUT Current ADC High Byte */
#define IP55XX_VWPC_IADC_L      0x58    /* VWPC Current ADC Low Byte */
#define IP55XX_VWPC_IADC_H      0x59    /* VWPC Current ADC High Byte */
#define IP55XX_VBUS_OADC_L      0x5A    /* VBUS Output Current ADC Low Byte */
#define IP55XX_VBUS_OADC_H      0x5B    /* VBUS Output Current ADC High Byte */
#define IP55XX_VOUT_ADC_L       0x5E    /* VOUT Voltage ADC Low Byte */
#define IP55XX_VOUT_ADC_H       0x5F    /* VOUT Voltage ADC High Byte */
#define IP55XX_VWPC_VADC_L      0x60    /* VWPC Voltage ADC Low Byte */
#define IP55XX_VWPC_VADC_H      0x61    /* VWPC Voltage ADC High Byte */
#define IP55XX_VBUS_ADC_L       0x62    /* VBUS Voltage ADC Low Byte */
#define IP55XX_VBUS_ADC_H       0x63    /* VBUS Voltage ADC High Byte */
#define IP55XX_NTC1_ADC_L       0x64    /* NTC1 Voltage ADC Low Byte */
#define IP55XX_NTC1_ADC_H       0x65    /* NTC1 Voltage ADC High Byte */
#define IP55XX_LED4_ADC_L       0x68    /* LED4 Voltage ADC Low Byte */
#define IP55XX_LED4_ADC_H       0x69    /* LED4 Voltage ADC High Byte */
#define IP55XX_LED5_ADC_L       0x6A    /* LED5 Voltage ADC Low Byte */
#define IP55XX_LED5_ADC_H       0x6B    /* LED5 Voltage ADC High Byte */
#define IP55XX_IBAT_ADC_L       0x6E    /* Battery Current ADC Low Byte */
#define IP55XX_IBAT_ADC_H       0x6F    /* Battery Current ADC High Byte (I2C 0xEA) */
#define IP55XX_ISYS_ADC_L       0x70    /* ISYS Current ADC Low Byte */
#define IP55XX_ISYS_ADC_H       0x71    /* ISYS Current ADC High Byte */

/* Temperature Registers */
#define IP55XX_TEMP_IC_L        0x74    /* IC Temperature Low Byte */
#define IP55XX_TEMP_IC_H        0x75    /* IC Temperature High Byte */

/* SOC (State of Charge) */
#define IP55XX_SOC_PERCENT      0x7B    /* Battery Percentage */

/* MOS Control */
#define IP55XX_MOS_CTRL         0x86    /* MOS Control Register */
#define IP55XX_MOS_STATUS       0x87    /* MOS Status Register */

/* Other Status Registers */
#define IP55XX_FCP_STATUS        0xA1    /* FCP Status Register */
#define IP55XX_STATUS_SRC0       0xA4    /* SRC Output Fast Charge Status 0 */
#define IP55XX_STATUS_SRC1       0xA5    /* SRC Output Fast Charge Status 1 */
#define IP55XX_STATUS_SRC2       0xA8    /* SRC Output Fast Charge Status 2 */
#define IP55XX_AFC_STATUS        0xAF    /* AFC Status Register */
#define IP55XX_PD_STATE0         0xB1    /* PD Sink Input Connection Status */
#define IP55XX_PD_STATE1         0xC2    /* PD SRC Output Connection Status */
#define IP55XX_PD_STATE2         0xC3    /* PD PPS SRC Output Connection Status */

/* ========== SYS_CTL0 Register Bits (I2C: 0xE8) ========== */
#define IP55XX_SYS_CTL0_EN_BOOST       (1 << 1)  /* Boost Output Enable */
#define IP55XX_SYS_CTL0_EN_CHARGER     (1 << 0)  /* Charger Enable */
#define IP55XX_SYS_CTL0_EN_C2B_DET     (1 << 2)  /* Auto-Boost on Unplug */

/* ========== SYS_CTL1 Register Bits (I2C: 0xE8) ========== */
#define IP55XX_SYS_CTL1_EN_ILOW_TIME_SHIFT  4   /* Light Load Timeout */
#define IP55XX_SYS_CTL1_EN_ILOW_TIME_MASK    (0x3 << 4)
#define IP55XX_SYS_CTL1_EN_POW_LOW      (1 << 3)  /* Power-based Light Load */
#define IP55XX_SYS_CTL1_EN_ISYS_LOW     (1 << 2)  /* Current-based Light Load */

/* ========== CHG_VOLT_SET Register Bits (I2C: 0xEA) ========== */
#define IP55XX_CHG_VOLT_VSET_SHIFT      2       /* Voltage Setting */
#define IP55XX_CHG_VOLT_VSET_MASK       (0x3 << 2)
#define IP55XX_CHG_VOLT_VSET_4V2        (0x0 << 2)  /* 4.2V */
#define IP55XX_CHG_VOLT_VSET_4V3        (0x1 << 2)  /* 4.3V */
#define IP55XX_CHG_VOLT_VSET_4V35       (0x2 << 2)  /* 4.35V */
#define IP55XX_CHG_VOLT_VSET_4V4        (0x3 << 2)  /* 4.4V */
#define IP55XX_CHG_VOLT_RCV_SHIFT       0       /* CV Voltage Offset */
#define IP55XX_CHG_VOLT_RCV_MASK        (0x3 << 0)
#define IP55XX_CHG_VOLT_RCV_0MV         (0x0 << 0)  /* +0mV */
#define IP55XX_CHG_VOLT_RCV_14MV        (0x1 << 0)  /* +14mV */
#define IP55XX_CHG_VOLT_RCV_28MV        (0x2 << 0)  /* +28mV (default) */
#define IP55XX_CHG_VOLT_RCV_42MV        (0x3 << 0)  /* +42mV */

/* ========== VWPC_CTL0 Register Bits (I2C: 0xE8) ========== */
#define IP55XX_VWPC_CTL0_EN_WPC      (1 << 0)  /* Wireless Charger Enable */
#define IP55XX_VWPC_CTL0_WPC_DET     (1 << 1)  /* Wireless Power Detected */
#define IP55XX_VWPC_CTL0_WPC_CHG     (1 << 2)  /* Wireless Charging Active */
#define IP55XX_VWPC_CTL0_WPC_DONE    (1 << 3)  /* Wireless Charging Complete */

/* ========== ADC Conversion Constants ========== */
/* Note: IP5561 ADC values are 16-bit unsigned integers */
/* LSB values based on IP5561 datasheet specifications */
#define IP55XX_VBAT_ADC_LSB          1.0      /* mV per LSB: VBAT = ADC_value * 1.0 mV */
#define IP55XX_IBAT_ADC_LSB          1.0      /* mA per LSB: IBAT = ADC_value * 1.0 mA (signed) */
#define IP55XX_VBUS_ADC_LSB          1.0      /* mV per LSB: VBUS = ADC_value * 1.0 mV */
#define IP55XX_IBUS_ADC_LSB          1.0      /* mA per LSB: IBUS = ADC_value * 1.0 mA */
#define IP55XX_VOUT_ADC_LSB          1.0      /* mV per LSB: VOUT = ADC_value * 1.0 mV */
#define IP55XX_NTC1_ADC_LSB          0.26855  /* mV per LSB: NTC = ADC_value * 0.26855 mV */
