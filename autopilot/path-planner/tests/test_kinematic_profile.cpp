#include <gtest/gtest.h>
#include "kinematic_profile_filter.h"
#include <cmath>

TEST(KinematicProfileFilterTest, AccelerationAndVelocityLimits) {
    PlannerConfig cfg;
    cfg.max_velocity = 2.0;
    cfg.max_vertical_velocity = 1.0;
    cfg.max_acceleration = 5.0;
    cfg.max_yaw_rate = 1.5;
    cfg.position_p_gain = 2.0;
    cfg.yaw_p_gain = 2.0;
    cfg.arrival_tolerance = 0.10;

    KinematicProfileFilter filter(cfg);

    Pose current = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose target = {10.0, 0.0, 5.0, 0.0, 0.0, 3.14159, 0.0, 1.0}; // Large step command

    double dt = 0.05;
    FlightCommand cmd = filter.planStep(current, target, dt);

    // Initial step should be limited by acceleration: dv = a_max * dt = 5.0 * 0.05 = 0.25 m/s
    EXPECT_NEAR(filter.getVx(), 0.25, 1e-4);
    EXPECT_NEAR(cmd.target_x, 0.25 * 0.05, 1e-4);

    // Step repeatedly until velocity saturates at max_velocity
    for (int i = 0; i < 50; ++i) {
        cmd = filter.planStep(current, target, dt);
    }
    EXPECT_NEAR(filter.getVx(), cfg.max_velocity, 1e-4);
    EXPECT_NEAR(filter.getVz(), cfg.max_vertical_velocity, 1e-4);

    // Reset should clear velocity states
    filter.reset();
    EXPECT_DOUBLE_EQ(filter.getVx(), 0.0);
    EXPECT_DOUBLE_EQ(filter.getVy(), 0.0);
    EXPECT_DOUBLE_EQ(filter.getVz(), 0.0);
}

TEST(KinematicProfileFilterTest, ArrivalToleranceDetection) {
    PlannerConfig cfg;
    cfg.arrival_tolerance = 0.15;
    KinematicProfileFilter filter(cfg);

    Pose p1 = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose p2 = {1.05, 2.05, 3.05, 0.0, 0.0, 0.0, 0.0, 1.0}; // dist ~ 0.086m < 0.15m
    Pose p3 = {1.20, 2.00, 3.00, 0.0, 0.0, 0.0, 0.0, 1.0}; // dist = 0.20m > 0.15m

    EXPECT_TRUE(filter.hasArrived(p1, p2));
    EXPECT_FALSE(filter.hasArrived(p1, p3));
}
