idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES esp_netif lwip esp_timer
)

target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--start-group"
    "${CMAKE_CURRENT_LIST_DIR}/libmicroros.a"
    "-Wl,--end-group"
    "-u lwip_getaddrinfo"
    "-u lwip_freeaddrinfo"
)