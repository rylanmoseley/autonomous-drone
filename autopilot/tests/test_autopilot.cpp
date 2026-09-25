#include <gtest/gtest.h>
#include "../autopilot_node.h"
#include <cmath>

TEST(AutopilotTest, UninitializedFlightCommand) {
    AutopilotNode node;
    
    // Initial state is IDLE
    EXPECT_EQ(node.getState(), MissionState::IDLE);
    
    UIGoalSequence seq = {};
    seq.start = true;
    seq.num_goals = 1;
    seq.goals[0] = {1.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    node.setUISequence(seq);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    
    // First tick consumes start_mission and transitions to TAKEOFF
    node.tick(current, 0.05);
    EXPECT_EQ(node.getState(), MissionState::TAKEOFF);
    
    // Second tick executes TAKEOFF and outputs command
    FlightCommand cmd = node.tick(current, 0.05);
    
    // Verify target_yaw is valid (not NaN or uninitialized garbage)
    EXPECT_FALSE(std::isnan(cmd.target_yaw));
    EXPECT_TRUE(std::isfinite(cmd.target_yaw));
    // During TAKEOFF, target_yaw should equal current_yaw
    EXPECT_EQ(cmd.target_yaw, current.yaw);
}

TEST(AutopilotTest, ZGoalPropagation) {
    AutopilotNode node;
    
    UIGoalSequence seq = {};
    seq.start = true;
    seq.num_goals = 1;
    // Goal Z is 1.0
    seq.goals[0] = {1.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    node.setUISequence(seq);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    node.tick(current, 0.05); // IDLE -> TAKEOFF
    
    // Satisfy TAKEOFF condition by reaching 1.0 Z
    current.z = 1.0;
    node.tick(current, 0.05); // TAKEOFF computes command
    node.tick(current, 0.05); // TAKEOFF condition met -> NAVIGATING
    EXPECT_EQ(node.getState(), MissionState::NAVIGATING);
    
    // Now in NAVIGATING, phase 0
    FlightCommand cmd = node.tick(current, 0.05);
    
    // Z should be 1.0! Wait, planner.plan outputs SETPOINT based on velocity.
    // If current.z is 1.0, and goal is 1.0, vz = 0, target_z = 1.0.
    EXPECT_DOUBLE_EQ(cmd.target_z, 1.0);
}
