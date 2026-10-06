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




add_subdirectory(mavlink-daemon)
add_subdirectory(goal-estimator)
add_subdirectory(path-planner)

# Physical Flight Executable
add_executable(autopilot_node main.cpp autopilot_node.cpp)
target_link_libraries(autopilot_node PRIVATE
    mavlink_daemon
    goal_estimator
    path_planner
    shared_contracts
    Threads::Threads
    m
)



# Autopilot State Machine Tests
add_executable(test_autopilot tests/test_autopilot.cpp autopilot_node.cpp)
target_link_libraries(test_autopilot PRIVATE
    GTest::gtest_main
    mavlink_daemon
    goal_estimator
    path_planner
    shared_contracts
    Threads::Threads
    m
)
gtest_discover_tests(test_autopilot)
