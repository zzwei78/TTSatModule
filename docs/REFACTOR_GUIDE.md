# 目录结构重构执行指南

## 当前状态

✅ 项目已重命名: `/data/trae_project/ttsat_firmware`
⏳ main 目录有 39 个文件全部平铺
⏳ 需要重构为子目录结构

---

## 重构目标目录结构

```
ttsat_firmware/
└── main/
    ├── CMakeLists.txt
    ├── main.c
    ├── README.md
    ├── ble/               # BLE GATT 服务 (12个文件)
    ├── tt/                # 天通模块 (7个文件)
    ├── audio/             # 音频处理 (8个文件)
    └── system/            # 系统管理 (8个文件)
```

---

## 重构脚本

**重要**: 请在 Windows PowerShell 或 Git Bash 中执行！

### 方式一：PowerShell 脚本（推荐）

保存为 `refactor.ps1`:

```powershell
# 设置项目路径
$projectPath = "Z:\trae_project\ttsat_firmware"
$mainPath = "$projectPath\main"

Write-Host "开始重构目录结构..." -ForegroundColor Green

# 创建子目录
Write-Host "创建子目录..." -ForegroundColor Yellow
New-Item -ItemType Directory -Force -Path "$mainPath\ble" | Out-Null
New-Item -ItemType Directory -Force -Path "$mainPath\tt" | Out-Null
New-Item -ItemType Directory -Force -Path "$mainPath\audio" | Out-Null
New-Item -ItemType Directory -Force -Path "$mainPath\system" | Out-Null

# 移动 BLE GATT 服务文件
Write-Host "移动 BLE GATT 服务文件..." -ForegroundColor Yellow
Move-Item -Path "$mainPath\ble_gatt_server.c" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\ble_gatt_server.h" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\gatt_system_server.c" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\gatt_system_server.h" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\spp_at_server.c" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\spp_at_server.h" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\spp_voice_server.c" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\spp_voice_server.h" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\gatt_log_server.c" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\gatt_log_server.h" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\gatt_ota_server.c" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\gatt_ota_server.h" -Destination "$mainPath\ble\" -Force
Move-Item -Path "$mainPath\ble_spp_server.h" -Destination "$mainPath\ble\" -Force

# 移动天通模块文件
Write-Host "移动天通模块文件..." -ForegroundColor Yellow
Move-Item -Path "$mainPath\tt_module.c" -Destination "$mainPath\tt\" -Force
Move-Item -Path "$mainPath\tt_module.h" -Destination "$mainPath\tt\" -Force
Move-Item -Path "$mainPath\tt_module_ota.c" -Destination "$mainPath\tt\" -Force
Move-Item -Path "$mainPath\tt_module_ota.h" -Destination "$mainPath\tt\" -Force
Move-Item -Path "$mainPath\tt_hardware.c" -Destination "$mainPath\tt\" -Force
Move-Item -Path "$mainPath\tt_hardware.h" -Destination "$mainPath\tt\" -Force
Move-Item -Path "$mainPath\gsm0710_manager.c" -Destination "$mainPath\tt\" -Force
Move-Item -Path "$mainPath\gsm0710_manager.h" -Destination "$mainPath\tt\" -Force

# 移动音频处理文件
Write-Host "移动音频处理文件..." -ForegroundColor Yellow
Move-Item -Path "$mainPath\voice_packet_handler.c" -Destination "$mainPath\audio\" -Force
Move-Item -Path "$mainPath\voice_packet_handler.h" -Destination "$mainPath\audio\" -Force
Move-Item -Path "$mainPath\audiosvc.c" -Destination "$mainPath\audio\" -Force
Move-Item -Path "$mainPath\audiosvc.h" -Destination "$mainPath\audio\" -Force
Move-Item -Path "$mainPath\amrnb_data.c" -Destination "$mainPath\audio\" -Force
Move-Item -Path "$mainPath\audiodata.c" -Destination "$mainPath\audio\" -Force
Move-Item -Path "$mainPath\base64.c" -Destination "$mainPath\audio\" -Force
Move-Item -Path "$mainPath\base64.h" -Destination "$mainPath\audio\" -Force

# 移动系统管理文件
Write-Host "移动系统管理文件..." -ForegroundColor Yellow
Move-Item -Path "$mainPath\power_manage.c" -Destination "$mainPath\system\" -Force
Move-Item -Path "$mainPath\power_manage.h" -Destination "$mainPath\system\" -Force
Move-Item -Path "$mainPath\ota_partition.c" -Destination "$mainPath\system\" -Force
Move-Item -Path "$mainPath\ota_partition.h" -Destination "$mainPath\system\" -Force
Move-Item -Path "$mainPath\debuglog.c" -Destination "$mainPath\system\" -Force
Move-Item -Path "$mainPath\debuglog.h" -Destination "$mainPath\system\" -Force
Move-Item -Path "$mainPath\syslog.c" -Destination "$mainPath\system\" -Force
Move-Item -Path "$mainPath\syslog.h" -Destination "$mainPath\system\" -Force

# 移动测试文件到 tt
Move-Item -Path "$mainPath\tt_hardware_test.c" -Destination "$mainPath\tt\" -Force

# 移动文档文件
Write-Host "移动文档文件..." -ForegroundColor Yellow
Move-Item -Path "$mainPath\readme_ble_gatt.md" -Destination "$mainPath\README.md" -Force
Move-Item -Path "$mainPath\service_init_flow.md" -Destination "$mainPath\docs" -Force

Write-Host "目录结构重构完成！" -ForegroundColor Green
Write-Host "下一步：运行 update_includes.py 更新头文件引用" -ForegroundColor Cyan
```

**执行**:
```powershell
# 在 Z:\trae_project\ttsat_firmware 目录下
.\refactor.ps1
```

---

### 方式二：Bash 脚本

保存为 `refactor.sh`:

```bash
#!/bin/bash

# 设置项目路径（根据你的实际路径修改）
PROJECT_PATH="/data/trae_project/ttsat_firmware"
MAIN_PATH="$PROJECT_PATH/main"

cd "$MAIN_PATH" || exit 1

echo "开始重构目录结构..."

# 创建子目录
echo "创建子目录..."
mkdir -p ble tt audio system

# 移动 BLE GATT 服务文件
echo "移动 BLE GATT 服务文件..."
mv ble_gatt_server.* ble/
mv gatt_system_server.* ble/
mv gatt_log_server.* ble/
mv gatt_ota_server.* ble/
mv spp_at_server.* ble/
mv spp_voice_server.* ble/
mv ble_spp_server.h ble/

# 移动天通模块文件
echo "移动天通模块文件..."
mv tt_module.* tt/
mv tt_module_ota.* tt/
mv tt_hardware.* tt/
mv tt_hardware_test.c tt/
mv gsm0710_manager.* tt/

# 移动音频处理文件
echo "移动音频处理文件..."
mv voice_packet_handler.* audio/
mv audiosvc.* audio/
mv amrnb_data.c audio/
mv audiodata.c audio/
mv base64.* audio/

# 移动系统管理文件
echo "移动系统管理文件..."
mv power_manage.* system/
mv ota_partition.* system/
mv debuglog.* system/
mv syslog.* system/

# 移动文档
echo "移动文档文件..."
mv readme_ble_gatt.md README.md
mkdir -p ../docs
mv service_init_flow.md ../docs/

echo "目录结构重构完成！"
echo "下一步：运行 update_includes.py 更新头文件引用"
```

**执行**:
```bash
chmod +x refactor.sh
./refactor.sh
```

---

## 更新头文件引用

### Python 脚本

保存为 `update_includes.py`:

```python
#!/usr/bin/env python3
"""
更新头文件引用路径
"""

import os
import re
from pathlib import Path

# 头文件映射
HEADER_MAPPINGS = {
    # BLE 相关
    'ble_gatt_server.h': 'ble/ble_gatt_server.h',
    'gatt_system_server.h': 'ble/gatt_system_server.h',
    'spp_at_server.h': 'ble/spp_at_server.h',
    'spp_voice_server.h': 'ble/spp_voice_server.h',
    'gatt_log_server.h': 'ble/gatt_log_server.h',
    'gatt_ota_server.h': 'ble/gatt_ota_server.h',

    # 天通模块
    'tt_module.h': 'tt/tt_module.h',
    'tt_module_ota.h': 'tt/tt_module_ota.h',
    'tt_hardware.h': 'tt/tt_hardware.h',
    'gsm0710_manager.h': 'tt/gsm0710_manager.h',

    # 音频处理
    'voice_packet_handler.h': 'audio/voice_packet_handler.h',
    'audiosvc.h': 'audio/audiosvc.h',
    'base64.h': 'audio/base64.h',

    # 系统管理
    'power_manage.h': 'system/power_manage.h',
    'ota_partition.h': 'system/ota_partition.h',
    'debuglog.h': 'system/debuglog.h',
    'syslog.h': 'system/syslog.h',
}

def update_includes(file_path, subdir):
    """更新文件中的头文件引用"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    original_content = content

    # 更新 #include 语句
    for old_header, new_header in HEADER_MAPPINGS.items():
        # 匹配 #include "old_header"
        pattern = f'#include "{old_header}"'
        replacement = f'#include "{new_header}"'
        content = content.replace(pattern, replacement)

    if content != original_content:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✓ 更新: {file_path}")
        return True
    return False

def process_directory(root_path):
    """处理目录下的所有 C/H 文件"""
    root = Path(root_path)
    updated_count = 0

    # 遍历所有子目录
    for subdir in ['ble', 'tt', 'audio', 'system', '']:
        subdir_path = root / subdir

        if not subdir_path.exists():
            continue

        # 处理当前目录的所有 .c 和 .h 文件
        for ext in ['*.c', '*.h']:
            for file_path in subdir_path.glob(ext):
                if update_includes(file_path, subdir):
                    updated_count += 1

    print(f"\n总计更新 {updated_count} 个文件")

if __name__ == '__main__':
    main_path = Path(__file__).parent / 'main'
    print(f"处理目录: {main_path}")
    print("=" * 50)
    process_directory(main_path)
    print("=" * 50)
    print("\n✓ 头文件引用更新完成！")
```

**执行**:
```bash
python3 update_includes.py
```

---

## 更新 CMakeLists.txt

新的 `main/CMakeLists.txt`:

```cmake
# 主要源文件
set(MAIN_SRCS
    "main.c"
)

# BLE GATT 服务源文件
set(BLE_SRCS
    "ble/ble_gatt_server.c"
    "ble/gatt_system_server.c"
    "ble/spp_at_server.c"
    "ble/spp_voice_server.c"
    "ble/gatt_log_server.c"
    "ble/gatt_ota_server.c"
)

# 天通模块源文件
set(TT_SRCS
    "tt/tt_module.c"
    "tt/tt_module_ota.c"
    "tt/tt_hardware.c"
    "tt/tt_hardware_test.c"
    "tt/gsm0710_manager.c"
)

# 音频处理源文件
set(AUDIO_SRCS
    "audio/voice_packet_handler.c"
    "audio/audiosvc.c"
    "audio/amrnb_data.c"
    "audio/audiodata.c"
    "audio/base64.c"
)

# 系统管理源文件
set(SYSTEM_SRCS
    "system/power_manage.c"
    "system/ota_partition.c"
    "system/debuglog.c"
    "system/syslog.c"
)

idf_component_register(
    SRCS "${MAIN_SRCS}" "${BLE_SRCS}" "${TT_SRCS}" "${AUDIO_SRCS}" "${SYSTEM_SRCS}"
    INCLUDE_DIRS "." "ble" "tt" "audio" "system"
    PRIV_REQUIRES
        audio_codec2
        nvs_flash
        bt
        esp_driver_uart
        esp-adf-libs
        audio_stream
        mbedtls
        esp_partition
        bq27220
        IP5561
        openthread
        libgsm0710
)
```

---

## 执行步骤总结

1. **备份项目** (重要!)
   ```powershell
   # Windows
   Copy-Item -Path "Z:\trae_project\ttsat_firmware" -Destination "Z:\trae_project\ttsat_firmware_backup" -Recurse
   ```

2. **移动文件**
   ```powershell
   # 在项目根目录执行
   .\refactor.ps1
   ```

3. **更新头文件引用**
   ```bash
   python3 update_includes.py
   ```

4. **更新 CMakeLists.txt**
   - 手动替换为上面提供的新内容

5. **清理并重新编译**
   ```bash
   rm -rf build/
   idf.py build
   ```

6. **验证编译**
   ```bash
   idf.py build
   ```

---

## 注意事项

1. **路径分隔符**: Windows 使用 `\`，Linux/Mac 使用 `/`
2. **备份**: 重构前务必备份！
3. **IDE**: 重构后可能需要刷新索引或重新导入项目
4. **Git**: 如果使用 Git，可能需要调整 `.gitignore`

---

## 验证清单

- [ ] 备份完成
- [ ] 文件移动完成
- [ ] 头文件引用更新完成
- [ ] CMakeLists.txt 更新完成
- [ ] 编译成功
- [ ] 功能测试通过

---

## 回滚方案

如果出现问题：

```powershell
# Windows
Remove-Item -Recurse -Force "Z:\trae_project\ttsat_firmware"
Copy-Item -Path "Z:\trae_project\ttsat_firmware_backup" -Destination "Z:\trae_project\ttsat_firmware" -Recurse
```

```bash
# Linux/Mac
rm -rf /data/trae_project/ttsat_firmware
cp -r /data/trae_project/ttsat_firmware_backup /data/trae_project/ttsat_firmware
```

---

**准备好后，请告诉我执行哪一步！**
