# When building for ESP-IDF, idf_component_register is available.
# When building natively (desktop), we use standard CMake.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "control_system.cpp"
                           INCLUDE_DIRS "include"
                           REQUIRES hal units_lib eigen_lib)
else()
    add_library(control_system control_system.cpp)
    target_include_directories(control_system PUBLIC include)
    # Desktop build needs to find hal headers
    target_include_directories(control_system PUBLIC ../hal/include)
    # Desktop build needs to link against native targets
    target_link_libraries(control_system PUBLIC units_lib eigen_lib)
endif()
