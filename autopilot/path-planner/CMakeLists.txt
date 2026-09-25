add_library(path_planner STATIC path_planner.cpp)
target_include_directories(path_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(path_planner PUBLIC shared_contracts)

add_executable(test_planner tests/test_planner.cpp)
target_link_libraries(test_planner PRIVATE path_planner gtest gtest_main)
add_test(NAME test_planner COMMAND test_planner)
