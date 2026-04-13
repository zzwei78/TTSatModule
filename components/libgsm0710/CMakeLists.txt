# ESP-IDF CMakeLists.txt for libgsm0710 component

idf_component_register(
    SRCS "gsm0710.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
