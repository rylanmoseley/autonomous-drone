# When building for ESP-IDF, idf_component_register is available.
# When building natively (desktop), we use standard CMake.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "control_system.cpp"
                           INCLUDE_DIRS "include"
                           REQUIRES hal)
else()
    add_library(control_system control_system.cpp)
    target_include_directories(control_system PUBLIC include)
    # Desktop build needs to find hal headers
    target_include_directories(control_system PUBLIC ../hal/include)
endif()
