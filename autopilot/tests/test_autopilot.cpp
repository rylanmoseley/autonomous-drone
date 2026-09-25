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
    seq.goals[0] = {1, 1.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
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
    seq.goals[0] = {1, 1.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    node.setUISequence(seq);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    node.tick(current, 0.05); // IDLE -> TAKEOFF
    
    // Satisfy TAKEOFF condition by reaching 1.0 Z
    current.z = 1.0;
    node.tick(current, 0.05); // TAKEOFF computes command
    node.tick(current, 0.05); // TAKEOFF condition met -> NAVIGATING
    EXPECT_EQ(node.getState(), MissionState::NAVIGATING);
    
    // Now in NAVIGATING, phase 0
    // Wait, the estimator will NOT have goal 1!
    // So the drone will SPIN to search for it!
    // The target z should be current.z (1.0).
    FlightCommand cmd = node.tick(current, 0.05);
    EXPECT_DOUBLE_EQ(cmd.target_z, 1.0);
}

TEST(AutopilotTest, FullMissionSimulation) {
    AutopilotNode node;
    UIGoalSequence seq = {};
    seq.start = true;
    seq.num_goals = 3;
    seq.goals[0] = {1, 1.0, -0.5, 1.0, 0, 0, 0, 0, 1.0};
    seq.goals[1] = {2, 1.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    seq.goals[2] = {3, 1.0, 0.5, 1.0, 0, 0, 0, 0, 1.0};
    node.setUISequence(seq);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    double dt = 0.05;
    
    // First tick consumes start_mission and transitions to TAKEOFF
    node.tick(current, dt);
    
    int max_ticks = 2000;
    int ticks = 0;
    while (node.getState() != MissionState::IDLE && ticks < max_ticks) {
        // Inject perfect vision estimates for all goals
        std::vector<VisionGoalEstimate> estimates;
        for (int i = 0; i < 3; ++i) {
            VisionGoalEstimate est;
            est.goal_id = i + 1;
            est.pose = {seq.goals[i].x, seq.goals[i].y, seq.goals[i].z, 0, 0, seq.goals[i].yaw, 0, 1.0};
            est.determinate = true;
            estimates.push_back(est);
        }
        double current_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        node.getEstimator().updateEstimates(estimates, current_time);
        
        FlightCommand cmd = node.tick(current, dt);
        
        // Simple kinematic mock simulator (ignores yaw dynamics for simplicity)
        if (cmd.enable) {
            current.x = cmd.target_x;
            current.y = cmd.target_y;
            current.z = cmd.target_z;
            current.yaw = cmd.target_yaw;
        }
        ticks++;
    }
    
    EXPECT_LT(ticks, max_ticks) << "Autopilot locked up! Failed to complete mission in " << max_ticks << " ticks.";
    EXPECT_EQ(node.getState(), MissionState::IDLE);
    std::cout << "Mission completed in " << ticks << " ticks." << std::endl;
}
