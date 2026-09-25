add_library(mavlink_daemon STATIC mavlink_daemon.cpp)
target_include_directories(mavlink_daemon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(mavlink_daemon PUBLIC shared_contracts Threads::Threads)

add_executable(test_mavlink tests/test_mavlink.cpp)
target_link_libraries(test_mavlink PRIVATE mavlink_daemon gtest gtest_main Threads::Threads)
add_test(NAME test_mavlink COMMAND test_mavlink)
