include(FetchContent)

FetchContent_Declare(
  units
  GIT_REPOSITORY https://github.com/nholthaus/units.git
  GIT_TAG master
)
FetchContent_MakeAvailable(units)

if(COMMAND idf_component_register)
    idf_component_register(INCLUDE_DIRS "${units_SOURCE_DIR}/include")
else()
    add_library(units_lib INTERFACE)
    target_include_directories(units_lib INTERFACE "${units_SOURCE_DIR}/include")
endif()
