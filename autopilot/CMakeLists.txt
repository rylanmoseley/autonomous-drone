add_subdirectory(mavlink-daemon)
add_subdirectory(goal-estimator)
add_subdirectory(path-planner)

# Physical Flight Executable (Runs on Pi Zero 2 W)
add_executable(autopilot_node
    main.cpp
)

target_link_libraries(autopilot_node PRIVATE
    mavlink_daemon
    goal_estimator
    path_planner
    shared_contracts
    Threads::Threads
    m # Math library
)

# Simulator Executable (Runs on PC or Pi)
option(BUILD_SIMULATOR "Build the mock autopilot testbed" ON)

if(BUILD_SIMULATOR)
    add_executable(autopilot_sim
        sim/mock_runner.cpp
    )

    target_link_libraries(autopilot_sim PRIVATE
        goal_estimator
        path_planner
        shared_contracts
        Threads::Threads
        m
    )
endif()
