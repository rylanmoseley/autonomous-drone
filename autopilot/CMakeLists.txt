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

find_package(Python3 QUIET COMPONENTS Interpreter)
set(SURVEYED_TAGS_JSON "${CMAKE_CURRENT_SOURCE_DIR}/config/surveyed_tags.json")
set(GEN_TAG_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_tag_map.py")
set(GEN_TAG_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/generated_surveyed_tags.h")

if(Python3_FOUND)
    add_custom_command(
        OUTPUT "${GEN_TAG_HEADER}"
        COMMAND ${Python3_EXECUTABLE} "${GEN_TAG_SCRIPT}" "${SURVEYED_TAGS_JSON}" "${GEN_TAG_HEADER}"
        DEPENDS "${SURVEYED_TAGS_JSON}" "${GEN_TAG_SCRIPT}"
        COMMENT "Generating compiled tag map from surveyed_tags.json"
        VERBATIM
    )
    add_custom_target(generate_tag_map ALL DEPENDS "${GEN_TAG_HEADER}")
endif()

add_subdirectory(mavlink-daemon)
add_subdirectory(goal-estimator)
add_subdirectory(path-planner)

# Core Autopilot Library
add_library(autopilot_core STATIC
    mission_state_machine.cpp
    mission_sequencer.cpp
    perception_coordinator.cpp
    safety_supervisor.cpp
    telemetry_bridge.cpp
    autopilot_node.cpp
)
if(TARGET generate_tag_map)
    add_dependencies(autopilot_core generate_tag_map)
endif()
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

# Camera Perception Isolated Unit Tests
add_executable(test_sim_perception tests/test_sim_perception.cpp)
target_link_libraries(test_sim_perception PRIVATE
    GTest::gtest_main
    autopilot_sim
)
gtest_discover_tests(test_sim_perception)

# Isolated Subsystem Unit Tests
add_executable(test_mission_state_machine tests/test_mission_state_machine.cpp)
target_link_libraries(test_mission_state_machine PRIVATE
    GTest::gtest_main
    autopilot_core
)
gtest_discover_tests(test_mission_state_machine)

add_executable(test_mission_sequencer tests/test_mission_sequencer.cpp)
target_link_libraries(test_mission_sequencer PRIVATE
    GTest::gtest_main
    autopilot_core
)
gtest_discover_tests(test_mission_sequencer)

add_executable(test_safety_supervisor tests/test_safety_supervisor.cpp)
target_link_libraries(test_safety_supervisor PRIVATE
    GTest::gtest_main
    autopilot_core
)
gtest_discover_tests(test_safety_supervisor)

# Python Simulator Unit & Contract Tests
if(Python3_FOUND)
    add_test(NAME test_sim_web COMMAND ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/sim/tests)
endif()
