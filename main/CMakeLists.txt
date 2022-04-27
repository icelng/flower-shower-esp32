idf_component_register(SRCS
                       "config_manager.cc"
                       "gatt_server.cc"
                       "main.cc"
                       "motor.cc"
                       "rtc_ds3231.cc"
                       "water_timer_manager.cc"
                       INCLUDE_DIRS ".")
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json)
target_link_libraries(${COMPONENT_LIB} PUBLIC nlohmann_json::nlohmann_json)
