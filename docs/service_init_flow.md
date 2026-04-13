# 服务初始化调用流程图

## SPP AT 服务和日志服务初始化调用链

```
main.c: app_main()
    └── ble_gatt_server.c: ble_gatt_server_init()
        └── ble_gatt_server.c: gatt_svr_init()
            ├── 初始化基础服务:
            │   ├── ble_svc_gap_init()
            │   └── ble_svc_gatt_init()
            ├── 注册主SPP服务:
            │   ├── ble_gatts_count_cfg(new_ble_svc_gatt_defs)
            │   └── ble_gatts_add_svcs(new_ble_svc_gatt_defs)
            ├── 注册SPP AT命令服务:
            │   └── spp_at_server_init()  <-- 在这里调用
            └── 注册日志上报服务:
                └── gatt_log_server_init()  <-- 在这里调用
```

## 调用说明

1. **入口点**: `app_main()` 是程序的主入口点，在其中调用 `ble_gatt_server_init()` 来初始化整个BLE GATT服务器。

2. **BLE GATT服务器初始化**: `ble_gatt_server_init()` 函数完成NimBLE协议栈的初始化，并调用 `gatt_svr_init()` 来注册所有GATT服务。

3. **服务注册中心**: `gatt_svr_init()` 函数是所有GATT服务的注册中心，它按照以下顺序初始化各种服务：
   - 首先初始化基础的GAP和GATT服务
   - 然后注册主SPP服务（用于语音数据传输）
   - 接着注册SPP AT命令服务（用于AT命令透传）
   - 最后注册日志上报服务（用于日志信息上报）

4. **服务具体实现**: `spp_at_server_init()` 和 `gatt_log_server_init()` 函数分别实现了各自服务的初始化逻辑，包括：
   - 计算服务所需的配置空间
   - 添加服务到GATT服务器
   - 初始化服务相关的资源

## 为什么你可能没直接看到调用？

1. **间接调用**: 这些函数是通过多层调用链间接调用的，而不是直接在主函数中调用。

2. **模块化设计**: 采用模块化设计将不同功能分离到不同文件中，提高代码的可维护性和可扩展性。

3. **集中管理**: 所有服务的初始化都集中在 `gatt_svr_init()` 函数中，便于统一管理和维护。

## 验证方法

你可以通过在这些初始化函数中添加日志输出来验证它们是否被正确调用。例如，在 `spp_at_server_init()` 和 `gatt_log_server_init()` 函数中添加：

```c
ESP_LOGI(TAG, "SPP AT server initialized successfully");
```

和

```c
ESP_LOGI(TAG, "Log server initialized successfully");
```

当程序运行时，如果在日志中看到这些输出，就说明这些初始化函数被正确调用了。