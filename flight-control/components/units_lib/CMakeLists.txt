# Tell CMake 4.x to forgive older dependencies that specify < 3.5 minimums
set(CMAKE_POLICY_VERSION_MINIMUM "3.5" CACHE STRING "Minimum policy version for 3rd party deps")

include(FetchContent)

FetchContent_Declare(
  units
  GIT_REPOSITORY https://github.com/nholthaus/units.git
  GIT_TAG v2.3.3
)
FetchContent_MakeAvailable(units)

if(COMMAND idf_component_register)
    idf_component_register(INCLUDE_DIRS "${units_SOURCE_DIR}/include")
else()
    add_library(units_lib INTERFACE)
    target_include_directories(units_lib INTERFACE "${units_SOURCE_DIR}/include")
endif()
