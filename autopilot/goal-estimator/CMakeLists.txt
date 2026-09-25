add_library(goal_estimator STATIC goal_estimator.cpp)
target_include_directories(goal_estimator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(goal_estimator PUBLIC shared_contracts)

add_executable(test_estimator tests/test_estimator.cpp)
target_link_libraries(test_estimator PRIVATE goal_estimator gtest gtest_main)
add_test(NAME test_estimator COMMAND test_estimator)
