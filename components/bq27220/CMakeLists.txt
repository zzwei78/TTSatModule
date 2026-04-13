idf_component_register(
    SRCS "bq27220.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES esp_driver_i2c
)
