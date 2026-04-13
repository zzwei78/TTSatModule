set(COMPONENT_SRCS "raw_stream.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_REQUIRES audio_pipeline driver audio_sal esp-adf-libs mbedtls)

register_component()
