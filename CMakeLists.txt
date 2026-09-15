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

# Shared schemas and data contracts
add_subdirectory(shared)

# Autopilot & Simulator
add_subdirectory(autopilot)

# add_subdirectory(vision-processing)
# add_subdirectory(ui)
