# Build System

The Flight Controller relies on a dual-build system to support both hardware deployment and local simulation.

## Environments
1. **ESP-IDF Build**:
   - Compiles the code for the ESP32-S3 microcontroller.
   - Requires the ESP-IDF toolchain.
   - Includes FreeRTOS and ESP-specific driver headers.
2. **Native Desktop Build**:
   - Compiles the code for your local OS (Linux/Mac/Windows) using standard `gcc` or `clang`.
   - Bypasses all ESP32 hardware dependencies by using mock interfaces.

## Structure
The root `CMakeLists.txt` dictates the build flow. Components are registered using `idf_component_register` for the hardware build, and standard `add_library` for the native desktop build.
