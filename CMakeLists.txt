cmake_minimum_required(VERSION 3.16)
project(DroneAutonomySystem LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
    -Wall -Wextra -Wpedantic
    -O2
)

find_package(Threads REQUIRED)

enable_testing()

# Shared schemas and data contracts
add_subdirectory(shared)

# Autopilot & Simulator
add_subdirectory(autopilot)

# Vision Processing Subsystem
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    message(STATUS "OpenCV ${OpenCV_VERSION} found: building vision-processing")
    add_subdirectory(vision-processing)
else()
    message(WARNING "OpenCV not found: skipping vision-processing (install libopencv-dev to build)")
endif()

# add_subdirectory(ui)
