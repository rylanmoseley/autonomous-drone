add_library(mavlink_daemon STATIC
    mavlink_daemon.cpp
)

target_include_directories(mavlink_daemon
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/external/c_library_v2
)

target_link_libraries(mavlink_daemon PUBLIC
    shared_contracts
    Threads::Threads
)
