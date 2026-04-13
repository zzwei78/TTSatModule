# Main Component CMakeLists.txt

# Main source files
set(MAIN_SRCS
    "main.c"
    "version.c"
)

# BLE GATT service source files
set(BLE_SRCS
    "ble/ble_conn_manager.c"
    "ble/ble_gatt_server.c"
    "ble/gatt_system_server.c"
    "ble/spp_at_server.c"
    "ble/spp_voice_server.c"
    "ble/gatt_log_server.c"
    "ble/gatt_ota_server.c"
)

# Tiantong module source files
set(TT_SRCS
    "tt/tt_module.c"
    "tt/tt_module_ota.c"
    "tt/tt_hardware.c"
    "tt/tt_hardware_test.c"
    "tt/gsm0710_manager.c"
)

# Audio processing source files
set(AUDIO_SRCS
    "audio/voice_packet_handler.c"
    "audio/audiosvc.c"
    "audio/base64.c"
)

# System management source files
set(SYSTEM_SRCS
    "system/power_manage.c"
    "system/ota_partition.c"
    "system/syslog.c"
    "system/ble_monitor.c"
)

# Config source files
set(CONFIG_SRCS
    "config/user_params.c"
)

idf_component_register(
    SRCS "${MAIN_SRCS}" "${BLE_SRCS}" "${TT_SRCS}" "${AUDIO_SRCS}" "${SYSTEM_SRCS}" "${CONFIG_SRCS}"
    INCLUDE_DIRS "." "ble" "tt" "audio" "system" "config"
    PRIV_REQUIRES
        audio_codec2
        nvs_flash
        bt
        esp_driver_uart
        esp-adf-libs
        audio_stream
        mbedtls
        esp_partition
        app_update
        bq27220
        IP5561
        openthread
        libgsm0710
)
