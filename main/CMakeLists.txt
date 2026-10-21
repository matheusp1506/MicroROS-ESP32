idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi nvs_flash micro_ros)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    -DUCLIENT_CUSTOM_TRANSPORT_MTU=2048
    -DRMW_UXRCE_MAX_BUFFER_SIZE=2048
    -DRMW_UXRCE_MAX_HISTORY=4
    -DRMW_UXRCE_TRANSPORT_CUSTOM
)