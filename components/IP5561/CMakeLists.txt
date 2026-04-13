idf_component_register(
    SRCS "IP5561.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES esp_driver_i2c
)
