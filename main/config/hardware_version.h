/*
 * hardware_version.h - Hardware Version Selection
 *
 * Compile-time selection between PCB revision V1.0 and V2.0.
 * SUPPORT_HARDWARE_V2 is enabled by default for current production boards.
 *
 * V1.0: BQ27220 fuel gauge, all I2C devices on bus 0, GPIO1 active-high power_en
 * V2.0: CW2217E fuel gauge, IP5561 on separate I2C1 bus, GPIO1 active-low boost IC,
 *       GPIO9 pwrkey (active low), GPIO38 ldo_en, GPIO36 boost mode
 */
#pragma once

//#define SUPPORT_HARDWARE_V2   /* Uncomment for V2.0, comment out for V1.0 */

#ifdef SUPPORT_HARDWARE_V2
#define HW_VERSION_STRING "V2.0"
#else
#define HW_VERSION_STRING "V1.0"
#endif
