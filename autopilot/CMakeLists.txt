cmake_minimum_required(VERSION 3.14)
project(autopilot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(mavlink-daemon)
add_subdirectory(goal-estimator)
add_subdirectory(path-planner)

# Core Autopilot Library
add_library(autopilot_core STATIC
    autopilot_node.cpp
)
target_link_libraries(autopilot_core PUBLIC
    mavlink_daemon
    goal_estimator
    path_planner
    shared_contracts
    Threads::Threads
    m
)
target_include_directories(autopilot_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Physical Flight Executable
add_executable(autopilot_node main.cpp)
target_link_libraries(autopilot_node PRIVATE
    autopilot_core
)

# Simulation Subsystem
add_subdirectory(sim)

# Autopilot State Machine Tests
add_executable(test_autopilot tests/test_autopilot.cpp)
target_link_libraries(test_autopilot PRIVATE
    GTest::gtest_main
    autopilot_core
)
gtest_discover_tests(test_autopilot)

# Autopilot Closed-Loop Simulator & Imputed Noise Tests
add_executable(test_autopilot_sim tests/test_autopilot_sim.cpp)
target_link_libraries(test_autopilot_sim PRIVATE
    GTest::gtest_main
    autopilot_sim
)
gtest_discover_tests(test_autopilot_sim)
