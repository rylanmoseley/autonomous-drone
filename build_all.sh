#!/usr/bin/env bash
set -e

TARGET="${1:-pi}"

build_host() {
    echo "=== Building for Host (native) ==="
    cmake -B build \
          -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j$(nproc)
    echo "=== Host build complete (build/autopilot/autopilot_node) ==="
}

build_pi() {
    echo "=== Building for Raspberry Pi Zero 2W (armhf / Cortex-A53) ==="
    cmake -B build-pi \
          -DCMAKE_TOOLCHAIN_FILE=cmake/pi_zero_2w_toolchain.cmake \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_SIMULATOR=OFF
    cmake --build build-pi -j$(nproc)
    echo "=== Pi Zero 2W build complete (build-pi/autopilot/autopilot_node) ==="
}

build_radxa() {
    echo "=== Building for Radxa Zero 3W (aarch64 / Cortex-A55) ==="
    cmake -B build-radxa \
          -DCMAKE_TOOLCHAIN_FILE=cmake/radxa_zero_3w_toolchain.cmake \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_SIMULATOR=OFF
    cmake --build build-radxa -j$(nproc)
    echo "=== Radxa Zero 3W build complete (build-radxa/autopilot/autopilot_node) ==="
}

case "$TARGET" in
    host)
        build_host
        ;;
    pi)
        build_pi
        ;;
    radxa)
        build_radxa
        ;;
    all)
        build_host
        build_pi
        build_radxa
        ;;
    *)
        echo "Usage: $0 [host|pi|radxa|all]"
        exit 1
        ;;
esac
