add_library(path_planner STATIC
    path_planner.cpp
)

target_include_directories(path_planner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(path_planner PUBLIC
    shared_contracts
)
