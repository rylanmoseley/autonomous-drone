include(FetchContent)

FetchContent_Declare(
  eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(eigen)

if(COMMAND idf_component_register)
    idf_component_register(INCLUDE_DIRS "${eigen_SOURCE_DIR}")
else()
    add_library(eigen_lib INTERFACE)
    target_include_directories(eigen_lib INTERFACE "${eigen_SOURCE_DIR}")
endif()
