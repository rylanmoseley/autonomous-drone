#include "path_planner.h"

PathPlanner::PathPlanner(const PlannerConfig& config)
    : kinematic_filter_(config) {}
