add_library(goal_estimator STATIC
    goal_track_filter.cpp
    goal_registry.cpp
    goal_telemetry_formatter.cpp
    goal_estimator.cpp
)
target_include_directories(goal_estimator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(goal_estimator PUBLIC shared_contracts)

add_executable(test_estimator tests/test_estimator.cpp)
target_link_libraries(test_estimator PRIVATE goal_estimator gtest gtest_main)
add_test(NAME test_estimator COMMAND test_estimator)

add_executable(test_goal_track_filter tests/test_goal_track_filter.cpp)
target_link_libraries(test_goal_track_filter PRIVATE goal_estimator gtest gtest_main)
add_test(NAME test_goal_track_filter COMMAND test_goal_track_filter)

add_executable(test_goal_registry tests/test_goal_registry.cpp)
target_link_libraries(test_goal_registry PRIVATE goal_estimator gtest gtest_main)
add_test(NAME test_goal_registry COMMAND test_goal_registry)
