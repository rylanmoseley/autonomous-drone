add_library(goal_estimator STATIC
    goal_estimator.cpp
)

target_include_directories(goal_estimator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(goal_estimator PUBLIC
    shared_contracts
)
