idf_component_register(
    SRCS "cw2217e.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)

# Break circular dependency: cw2217e.h includes fuel_gauge.h for shared types,
# but fuel_gauge component links against cw2217e. Add fuel_gauge's include
# directory manually (header-only visibility, no linking dependency).
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../fuel_gauge")
