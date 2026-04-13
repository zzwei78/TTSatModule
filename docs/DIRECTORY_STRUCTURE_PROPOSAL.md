# 目录结构重构方案

## 方案对比

### 方案一：main 内部子目录结构（简单）✅ 推荐

```
TTSatModule/
├── main/                          # 主组件（保持 ESP-IDF 规范）
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild
│   ├── idf_component.yml
│   ├── main.c                     # 主程序入口
│   ├── README.md                  # 组件说明
│   │
│   ├── ble/                       # BLE GATT 服务
│   │   ├── ble_gatt_server.c/h
│   │   ├── gatt_system_server.c/h
│   │   ├── gatt_log_server.c/h
│   │   ├── gatt_ota_server.c/h
│   │   ├── spp_at_server.c/h
│   │   └── spp_voice_server.c/h
│   │
│   ├── tt/                        # 天通模块
│   │   ├── tt_module.c/h
│   │   ├── tt_module_ota.c/h
│   │   ├── tt_hardware.c/h
│   │   ├── tt_hardware_test.c
│   │   └── gsm0710_manager.c/h
│   │
│   ├── audio/                     # 音频/语音处理
│   │   ├── voice_packet_handler.c/h
│   │   ├── audiosvc.c/h
│   │   ├── amrnb_data.c
│   │   ├── audiodata.c
│   │   └── base64.c/h
│   │
│   └── system/                    # 系统管理
│       ├── power_manage.c/h
│       ├── ota_partition.c/h
│       ├── debuglog.c/h
│       └── syslog.c/h
│
├── components/                    # 其他组件
│   ├── audio_codec2/              # ESP-ADF 音频编解码器
│   ├── bq27220/                   # 电池管理芯片驱动
│   ├── IP5561/                    # 充电管理芯片驱动
│   └── libgsm0710/                # GSM0710 MUX 库
│
├── docs/                          # 文档
│   ├── ANDROID_GATT_INTEGRATION.md
│   └── DIRECTORY_STRUCTURE_PROPOSAL.md
│
├── build/
├── CMakeLists.txt
├── sdkconfig
└── README.md
```

**优点**:
- ✅ 符合 ESP-IDF 组件规范
- ✅ main.c 保持在 main 目录
- ✅ 子目录清晰分隔功能模块
- ✅ 易于导航和维护
- ✅ CMakeLists.txt 修改最小

**缺点**:
- ⚠️ 需要更新所有 `#include` 路径
- ⚠️ 头文件引用需要加子目录前缀

---

### 方案二：多组件架构（更规范）

```
TTSatModule/
├── main/                          # 主组件（精简）
│   ├── CMakeLists.txt
│   ├── main.c
│   └── README.md
│
├── components/
│   ├── ble_services/              # BLE GATT 服务组件
│   │   ├── CMakeLists.txt
│   │   ├── ble_gatt_server.c/h
│   │   ├── gatt_system_server.c/h
│   │   ├── gatt_log_server.c/h
│   │   ├── gatt_ota_server.c/h
│   │   ├── spp_at_server.c/h
│   │   └── spp_voice_server.c/h
│   │
│   ├── tt_module/                 # 天通模块组件
│   │   ├── CMakeLists.txt
│   │   ├── tt_module.c/h
│   │   ├── tt_module_ota.c/h
│   │   ├── tt_hardware.c/h
│   │   ├── tt_hardware_test.c
│   │   └── gsm0710_manager.c/h
│   │
│   ├── audio_processor/            # 音频处理组件
│   │   ├── CMakeLists.txt
│   │   ├── voice_packet_handler.c/h
│   │   ├── audiosvc.c/h
│   │   ├── amrnb_data.c
│   │   ├── audiodata.c
│   │   └── base64.c/h
│   │
│   ├── system_manager/             # 系统管理组件
│   │   ├── CMakeLists.txt
│   │   ├── power_manage.c/h
│   │   ├── ota_partition.c/h
│   │   ├── debuglog.c/h
│   │   └── syslog.c/h
│   │
│   ├── audio_codec2/              # ESP-ADF 音频编解码器
│   ├── bq27220/                   # 电池管理芯片驱动
│   ├── IP5561/                    # 充电管理芯片驱动
│   └── libgsm0710/                # GSM0710 MUX 库
│
├── docs/
├── build/
├── CMakeLists.txt
└── sdkconfig
```

**优点**:
- ✅ 完全符合 ESP-IDF 组件化最佳实践
- ✅ 每个组件独立，可复用
- ✅ 组件间依赖清晰
- ✅ 更易于单元测试

**缺点**:
- ⚠️ 需要为每个组件创建 CMakeLists.txt
- ⚠️ 组件间通信需要更谨慎
- ⚠️ 构建时间可能增加
- ⚠️ 修改工作量较大

---

## 推荐：方案一（main 内部子目录）

### 实施步骤

1. **创建子目录**
   ```bash
   mkdir -p main/ble main/tt main/audio main/system
   ```

2. **移动文件**
   - BLE 相关 → main/ble/
   - 天通模块 → main/tt/
   - 音频处理 → main/audio/
   - 系统管理 → main/system/

3. **更新 CMakeLists.txt**
   - 添加子目录源文件
   - 更新 include 路径

4. **更新头文件引用**
   - 添加子目录前缀
   - 例如: `#include "ble_gatt_server.h"` → `#include "ble/ble_gatt_server.h"`

5. **编译验证**

---

## 头文件路径映射

### 原路径 → 新路径

```
ble_gatt_server.h       → ble/ble_gatt_server.h
gatt_system_server.h    → ble/gatt_system_server.h
spp_at_server.h         → ble/spp_at_server.h
spp_voice_server.h      → ble/spp_voice_server.h
gatt_log_server.h       → ble/gatt_log_server.h
gatt_ota_server.h       → ble/gatt_ota_server.h

tt_module.h             → tt/tt_module.h
tt_module_ota.h         → tt/tt_module_ota.h
tt_hardware.h           → tt/tt_hardware.h
gsm0710_manager.h       → tt/gsm0710_manager.h

voice_packet_handler.h  → audio/voice_packet_handler.h
audiosvc.h              → audio/audiosvc.h
base64.h                → audio/base64.h

power_manage.h          → system/power_manage.h
ota_partition.h         → system/ota_partition.h
debuglog.h              → system/debuglog.h
syslog.h                → system/syslog.h
```

---

## 项目重命名建议

同时建议将项目目录重命名：

**当前**: `TTSatModule`
**建议**: `ttsat_firmware` 或 `ttsat_ble_gateway`

```bash
# 方案 1: 全小写（推荐）
mv TTSatModule ttsat_firmware

# 方案 2: 保留描述性
mv TTSatModule TTSat-BLE-Gateway
```

---

## 最终目录树（完整）

```
ttsat_firmware/
├── README.md                      # 项目说明
├── CMakeLists.txt                 # 顶层 CMake
├── sdkconfig                      # 配置文件
├── docs/                          # 文档目录
│   ├── ANDROID_GATT_INTEGRATION.md
│   └── DIRECTORY_STRUCTURE_PROPOSAL.md
│
├── main/                          # 主组件
│   ├── CMakeLists.txt             # 组件 CMake
│   ├── Kconfig.projbuild
│   ├── idf_component.yml
│   ├── main.c                     # 程序入口
│   ├── README.md                  # 组件说明
│   │
│   ├── ble/                       # BLE GATT 服务
│   │   ├── ble_gatt_server.c
│   │   ├── ble_gatt_server.h
│   │   ├── gatt_system_server.c
│   │   ├── gatt_system_server.h
│   │   ├── spp_at_server.c
│   │   ├── spp_at_server.h
│   │   ├── spp_voice_server.c
│   │   ├── spp_voice_server.h
│   │   ├── gatt_log_server.c
│   │   ├── gatt_log_server.h
│   │   ├── gatt_ota_server.c
│   │   └── gatt_ota_server.h
│   │
│   ├── tt/                        # 天通模块
│   │   ├── tt_module.c
│   │   ├── tt_module.h
│   │   ├── tt_module_ota.c
│   │   ├── tt_module_ota.h
│   │   ├── tt_hardware.c
│   │   ├── tt_hardware.h
│   │   ├── tt_hardware_test.c
│   │   ├── gsm0710_manager.c
│   │   └── gsm0710_manager.h
│   │
│   ├── audio/                     # 音频处理
│   │   ├── voice_packet_handler.c
│   │   ├── voice_packet_handler.h
│   │   ├── audiosvc.c
│   │   ├── audiosvc.h
│   │   ├── amrnb_data.c
│   │   ├── audiodata.c
│   │   ├── base64.c
│   │   └── base64.h
│   │
│   └── system/                    # 系统管理
│       ├── power_manage.c
│       ├── power_manage.h
│       ├── ota_partition.c
│       ├── ota_partition.h
│       ├── debuglog.c
│       ├── debuglog.h
│       ├── syslog.c
│       └── syslog.h
│
├── components/                    # 可选组件
│   ├── audio_codec2/              # ESP-ADF 音频编解码
│   ├── bq27220/                   # 电池管理驱动
│   ├── IP5561/                    # 充电管理驱动
│   └── libgsm0710/                # GSM0710 MUX 库
│
└── build/                         # 构建输出
    └── ...
```

---

## 结论

**推荐采用方案一**，原因：
1. 改动适中，风险可控
2. 符合 ESP-IDF 规范
3. 清晰的模块划分
4. 易于后续维护

**同时建议项目重命名**为 `ttsat_firmware`（全小写，避免跨平台问题）。

