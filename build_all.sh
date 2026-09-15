# 1. Configure for the Pi using the toolchain
cmake -B build-pi \
      -DCMAKE_TOOLCHAIN_FILE=cmake/pi_zero_2w_toolchain.cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SIMULATOR=OFF

# 2. Compile at full speed using all host CPU cores
cmake --build build-pi -j$(nproc)
