cmake_minimum_required(VERSION 3.16)
project(desktop_sim)

set(CMAKE_CXX_STANDARD 17)

# Include the control system library natively
add_subdirectory(../components/control_system control_system_build)

add_executable(sim_main main.cpp mock_hardware.cpp)
target_include_directories(sim_main PRIVATE ../components/hal/include)
target_link_libraries(sim_main PRIVATE control_system)
