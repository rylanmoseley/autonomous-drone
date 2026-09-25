#include <gtest/gtest.h>
#include "path_planner.h"
#include <cmath>

TEST(PathPlannerTest, HasArrived) {
    PlannerConfig config;
    config.arrival_tolerance = 0.5;
    PathPlanner planner(config);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    Pose target = {0.4, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    EXPECT_TRUE(planner.has_arrived(current, target));
    
    target.x = 0.6;
    EXPECT_FALSE(planner.has_arrived(current, target));
}

TEST(PathPlannerTest, VelocityLimiting) {
    PlannerConfig config;
    config.max_velocity = 1.0;
    config.max_vertical_velocity = 0.25;
    config.position_p_gain = 2.0;
    config.max_yaw_rate = 1.0;
    config.yaw_p_gain = 1.0;
    config.arrival_tolerance = 0.1;
    PathPlanner planner(config);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    Pose target = {10.0, 0.0, 10.0, 0, 0, 0, 0, 1.0};
    
    double dt = 0.1;
    FlightCommand cmd = planner.plan(current, target, dt);
    
    EXPECT_NEAR(cmd.target_x, 0.1, 1e-6);
    EXPECT_NEAR(cmd.target_y, 0.0, 1e-6);
    EXPECT_NEAR(cmd.target_z, 0.025, 1e-6);
}
