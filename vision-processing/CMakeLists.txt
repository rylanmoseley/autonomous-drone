cmake_minimum_required(VERSION 3.16)
project(vision_processing LANGUAGES CXX)

find_package(OpenCV REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Core gate detection library
add_library(gate_detector STATIC
    src/gate_detector.cpp
)
target_link_libraries(gate_detector PUBLIC
    shared_contracts
    ${OpenCV_LIBS}
)
target_include_directories(gate_detector PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Production Vision Node
add_executable(vision_node AdvancedVisionTesting.cpp)
target_link_libraries(vision_node PRIVATE
    gate_detector
    Threads::Threads
)

# Alias for backwards compatibility with script name
add_executable(advanced_vision_testing AdvancedVisionTesting.cpp)
target_link_libraries(advanced_vision_testing PRIVATE
    gate_detector
    Threads::Threads
)

# Standalone prototyping and calibration utilities
add_executable(focal_calibration FocalLengthCalibration.cpp)
target_link_libraries(focal_calibration PRIVATE
    shared_contracts
    ${OpenCV_LIBS}
)

add_executable(color_vision_testing ColorVisionTesting.cpp)
target_link_libraries(color_vision_testing PRIVATE
    shared_contracts
    ${OpenCV_LIBS}
)

add_executable(opencv_testing OpenCVTesting.cpp)
target_link_libraries(opencv_testing PRIVATE
    shared_contracts
    ${OpenCV_LIBS}
)

# Automated Unit & Integration Tests
if(TARGET GTest::gtest_main)
    add_executable(test_vision tests/test_vision.cpp)
    target_link_libraries(test_vision PRIVATE
        gate_detector
        GTest::gtest_main
        Threads::Threads
    )
    add_test(NAME test_vision COMMAND test_vision)
endif()
