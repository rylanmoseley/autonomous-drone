# cmake/radxa_zero_3w_toolchain.cmake
# Toolchain for Radxa Zero 3W (Rockchip RK3566, Quad-Core Cortex-A55, 64-bit ARMv8.2-A)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Target cross-compilers (64-bit aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Hardware-specific optimization flags for Cortex-A55
set(CMAKE_C_FLAGS_INIT   "-mcpu=cortex-a55")
set(CMAKE_CXX_FLAGS_INIT "-mcpu=cortex-a55")

# Search paths constraint
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Emulator for running cross-compiled tests on host
find_program(QEMU_AARCH64_EMULATOR NAMES qemu-aarch64-static qemu-aarch64 PATHS /usr/bin /usr/local/bin)
if(QEMU_AARCH64_EMULATOR)
    set(CMAKE_CROSSCOMPILING_EMULATOR ${QEMU_AARCH64_EMULATOR} -L /usr/aarch64-linux-gnu)
endif()
