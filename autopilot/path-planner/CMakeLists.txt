add_library(path_planner STATIC
    path_planner.cpp
    kinematic_profile_filter.cpp
    approach_corridor_planner.cpp
    orthogonal_path_generator.cpp
    obstacle_avoidance_potential_field.cpp
    phase_alignment_evaluator.cpp
)
target_include_directories(path_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(path_planner PUBLIC shared_contracts)

# Modular isolated unit tests
add_executable(test_kinematic_profile tests/test_kinematic_profile.cpp)
target_link_libraries(test_kinematic_profile PRIVATE path_planner GTest::gtest GTest::gtest_main)
add_test(NAME test_kinematic_profile COMMAND test_kinematic_profile)

add_executable(test_orthogonal_generator tests/test_orthogonal_generator.cpp)
target_link_libraries(test_orthogonal_generator PRIVATE path_planner GTest::gtest GTest::gtest_main)
add_test(NAME test_orthogonal_generator COMMAND test_orthogonal_generator)

add_executable(test_obstacle_avoidance tests/test_obstacle_avoidance.cpp)
target_link_libraries(test_obstacle_avoidance PRIVATE path_planner GTest::gtest GTest::gtest_main)
add_test(NAME test_obstacle_avoidance COMMAND test_obstacle_avoidance)

# Integration planner test
add_executable(test_planner tests/test_planner.cpp)
target_link_libraries(test_planner PRIVATE path_planner GTest::gtest GTest::gtest_main)
add_test(NAME test_planner COMMAND test_planner)
