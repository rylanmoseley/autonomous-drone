#include <gtest/gtest.h>
#include "../autopilot_node.h"
#include <cmath>

TEST(AutopilotTest, UninitializedFlightCommand) {
    AutopilotNode node;
    
    // Initial state is STANDBY_DISARM
    EXPECT_EQ(node.getState(), FlightState::FLIGHT_STATE_STANDBY_DISARM);
    
    UICommandPacket cmd_pkt = {};
    cmd_pkt.msg_type = MSG_TYPE_UI_CMD;
    cmd_pkt.command = UI_CMD_START_MISSION;
    cmd_pkt.sequence_length = 1;
    cmd_pkt.goal_sequence[0] = SHAPE_CIRCLE;
    node.handleCommand(cmd_pkt);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    
    // First tick executes TAKEOFF and outputs command
    FlightCommand cmd = node.tick(current, 0.05);
    EXPECT_EQ(node.getState(), FlightState::FLIGHT_STATE_TAKEOFF);
    
    // Verify target_yaw is valid (not NaN or uninitialized garbage)
    EXPECT_FALSE(std::isnan(cmd.target_yaw));
    EXPECT_TRUE(std::isfinite(cmd.target_yaw));
    // During TAKEOFF, target_yaw should equal current.yaw
    EXPECT_EQ(cmd.target_yaw, current.yaw);
}

TEST(AutopilotTest, ZGoalPropagation) {
    AutopilotNode node;
    
    UICommandPacket cmd_pkt = {};
    cmd_pkt.msg_type = MSG_TYPE_UI_CMD;
    cmd_pkt.command = UI_CMD_START_MISSION;
    cmd_pkt.sequence_length = 1;
    cmd_pkt.goal_sequence[0] = SHAPE_CIRCLE;
    node.handleCommand(cmd_pkt);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    node.tick(current, 0.05); // In TAKEOFF
    
    // Satisfy TAKEOFF condition by reaching 1.0 Z
    current.z = 1.0;
    node.tick(current, 0.05); // TAKEOFF condition met -> transitions
    EXPECT_EQ(node.getState(), FlightState::FLIGHT_STATE_PATH_FOLLOWING);
    
    // Now in PATH_FOLLOWING / GOAL_SEEK
    // The estimator does not have SHAPE_CIRCLE yet, so drone will search for it
    FlightCommand cmd = node.tick(current, 0.05);
    EXPECT_DOUBLE_EQ(cmd.target_z, 1.0);
}

TEST(AutopilotTest, FullMissionSimulation) {
    AutopilotNode node;
    UICommandPacket cmd_pkt = {};
    cmd_pkt.msg_type = MSG_TYPE_UI_CMD;
    cmd_pkt.command = UI_CMD_START_MISSION;
    cmd_pkt.sequence_length = 3;
    cmd_pkt.goal_sequence[0] = SHAPE_CIRCLE;
    cmd_pkt.goal_sequence[1] = SHAPE_TRIANGLE;
    cmd_pkt.goal_sequence[2] = SHAPE_SQUARE;
    cmd_pkt.end_action = MISSION_END_LAND;
    node.handleCommand(cmd_pkt);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    double dt = 0.05;
    
    float goal_y[3] = {-0.5f, 0.0f, 0.5f};
    uint8_t shapes[3] = {SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE};
    
    int max_ticks = 2000;
    int ticks = 0;
    while (node.getState() != FlightState::FLIGHT_STATE_ROTOR_STOPPED && 
           node.getState() != FlightState::FLIGHT_STATE_STANDBY_DISARM && 
           ticks < max_ticks) {
        // Inject perfect vision estimates for all goals
        VisionPacket v_pkt = {};
        v_pkt.msg_type = MSG_TYPE_VISION;
        v_pkt.num_goals_reported = 3;
        for (int i = 0; i < 3; ++i) {
            v_pkt.goals[i].shape_type = shapes[i];
            v_pkt.goals[i].is_detected = 1;
            v_pkt.goals[i].num_hypotheses = 1;
            v_pkt.goals[i].hypotheses[0].position_cam = {1.0f, goal_y[i], 1.0f};
            v_pkt.goals[i].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
            v_pkt.goals[i].hypotheses[0].confidence = 1.0f;
        }
        double current_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        node.getEstimator().updateEstimates(v_pkt, current_time);
        
        FlightCommand cmd = node.tick(current, dt);
        
        // Simple kinematic mock simulator
        if (cmd.enable) {
            current.x = cmd.target_x;
            current.y = cmd.target_y;
            current.z = cmd.target_z;
            current.yaw = cmd.target_yaw;
        }
        ticks++;
    }
    
    EXPECT_LT(ticks, max_ticks) << "Autopilot locked up! Failed to complete mission in " << max_ticks << " ticks.";
    EXPECT_TRUE(node.getState() == FlightState::FLIGHT_STATE_ROTOR_STOPPED || 
                node.getState() == FlightState::FLIGHT_STATE_STANDBY_DISARM);
    std::cout << "Mission completed in " << ticks << " ticks." << std::endl;
}
