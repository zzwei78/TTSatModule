idf_component_register(
    SRCS "fuel_gauge.c"
    INCLUDE_DIRS "."
    REQUIRES bq27220 cw2217e esp_driver_i2c
)
