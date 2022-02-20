idf_component_register(SRCS "gatt_server.cc" "motor.cc" "main.cc" INCLUDE_DIRS ".")
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json)
target_link_libraries(${COMPONENT_LIB} PUBLIC nlohmann_json::nlohmann_json)
