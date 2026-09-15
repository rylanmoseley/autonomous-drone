add_library(shared_contracts INTERFACE)

target_include_directories(shared_contracts INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
