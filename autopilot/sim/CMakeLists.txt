cmake_minimum_required(VERSION 3.14)
project(autopilot_sim LANGUAGES CXX)

add_library(autopilot_sim STATIC
    src/follower_simulator.cpp
    src/autopilot_sim_harness.cpp
)

target_include_directories(autopilot_sim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(autopilot_sim PUBLIC
    autopilot_core
)

# Standalone CLI Scenario Runner
add_executable(autopilot_sim_runner
    src/sim_runner_main.cpp
)

target_link_libraries(autopilot_sim_runner PRIVATE
    autopilot_sim
)
