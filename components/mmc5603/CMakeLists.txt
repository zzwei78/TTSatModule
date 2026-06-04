idf_component_register(
    SRCS "mmc5603.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c freertos
)
