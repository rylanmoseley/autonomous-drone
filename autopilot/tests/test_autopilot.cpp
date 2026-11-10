#include <gtest/gtest.h>
#include "../autopilot_node.h"
#include "generated_surveyed_tags.h"
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

TEST(AutopilotTest, CompileTimeSurveyedTagMap) {
    // 1. Verify compile-time precomputed ROM entries
    EXPECT_EQ(COMPILED_SURVEYED_TAG_COUNT, 4u);
    EXPECT_STREQ(COMPILED_ARENA_NAME, "Competition Arena Alpha");

    // Fast O(log N) binary search lookup in ROM table
    const auto* tag1 = findCompiledSurveyedTag(1);
    ASSERT_NE(tag1, nullptr);
    EXPECT_EQ(tag1->tag_id, 1);
    EXPECT_FLOAT_EQ(tag1->world_pos.x, 1.5f);
    EXPECT_FLOAT_EQ(tag1->world_pos.y, -0.3f);
    EXPECT_FLOAT_EQ(tag1->world_pos.z, 1.0f);
    EXPECT_FLOAT_EQ(tag1->size_m, 0.16f);
    EXPECT_STREQ(tag1->label, "Gate 1 Entrance Marker");

    const auto* tag4 = findCompiledSurveyedTag(4);
    ASSERT_NE(tag4, nullptr);
    EXPECT_EQ(tag4->tag_id, 4);
    EXPECT_FLOAT_EQ(tag4->world_pos.y, 2.0f);
    EXPECT_FLOAT_EQ(tag4->world_pos.z, 0.5f);
    // 90 deg yaw quaternion: w=0.7071, z=0.7071
    EXPECT_NEAR(tag4->world_att.w, 0.7071f, 1e-3);
    EXPECT_NEAR(tag4->world_att.z, 0.7071f, 1e-3);

    // Non-existent tag returns nullptr
    EXPECT_EQ(findCompiledSurveyedTag(999), nullptr);
}

TEST(AutopilotTest, AutopilotNodePreloadedWithCompiledTags) {
    AutopilotNode node;

    // Node boots automatically initialized with compiled tag map from JSON
    EXPECT_TRUE(node.hasSurveyedTag(1));
    EXPECT_TRUE(node.hasSurveyedTag(2));
    EXPECT_TRUE(node.hasSurveyedTag(3));
    EXPECT_TRUE(node.hasSurveyedTag(4));
    EXPECT_FALSE(node.hasSurveyedTag(5));

    SurveyedTagPose tag1{};
    EXPECT_TRUE(node.getSurveyedTag(1, tag1));
    EXPECT_FLOAT_EQ(tag1.world_pos.x, 1.5f);
    EXPECT_FLOAT_EQ(tag1.world_pos.y, -0.3f);
    EXPECT_FLOAT_EQ(tag1.world_pos.z, 1.0f);

    // Dynamic runtime additions or overrides can still be registered
    node.registerSurveyedTag(5, {10.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, 0.25f);
    EXPECT_TRUE(node.hasSurveyedTag(5));

    // Clear registry
    node.clearSurveyedTags();
    EXPECT_FALSE(node.hasSurveyedTag(1));
    EXPECT_FALSE(node.hasSurveyedTag(5));
}

TEST(AutopilotTest, RetainGoalsWhenNewDataSent) {
    AutopilotNode node;
    
    UICommandPacket cmd_pkt = {};
    cmd_pkt.msg_type = MSG_TYPE_UI_CMD;
    cmd_pkt.command = UI_CMD_START_MISSION;
    cmd_pkt.sequence_length = 3;
    cmd_pkt.goal_sequence[0] = SHAPE_CIRCLE;
    cmd_pkt.goal_sequence[1] = SHAPE_TRIANGLE;
    cmd_pkt.goal_sequence[2] = SHAPE_SQUARE;
    cmd_pkt.end_action = MISSION_END_HOVER;
    node.handleCommand(cmd_pkt);

    Pose current = {0.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    double dt = 0.05;

    uint8_t shapes[3] = {SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE};
    float goal_x[3] = {1.5f, 3.0f, 4.5f};
    float goal_y[3] = {-0.4f, 0.0f, 0.4f};
    float goal_z[3] = {1.0f, 1.2f, 1.0f};

    for (int step = 0; step < 10; ++step) {
        VisionPacket v_pkt = {};
        v_pkt.msg_type = MSG_TYPE_VISION;
        v_pkt.num_goals_reported = 3;
        for (int i = 0; i < 3; ++i) {
            v_pkt.goals[i].shape_type = shapes[i];
            v_pkt.goals[i].is_detected = 1;
            v_pkt.goals[i].num_hypotheses = 1;
            v_pkt.goals[i].hypotheses[0].position_cam = {goal_x[i], goal_y[i], goal_z[i]};
            v_pkt.goals[i].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
            v_pkt.goals[i].hypotheses[0].confidence = 0.95f;
        }
        double now = 100.0 + step * dt;
        node.getEstimator().updateEstimates(v_pkt, now);
        node.tick(current, dt);
    }

    // Verify all 3 goals are in memory
    TrackedGoal g_circle, g_tri, g_sq;
    EXPECT_TRUE(node.getEstimator().getLastKnownEstimate(SHAPE_CIRCLE, g_circle));
    EXPECT_TRUE(node.getEstimator().getLastKnownEstimate(SHAPE_TRIANGLE, g_tri));
    EXPECT_TRUE(node.getEstimator().getLastKnownEstimate(SHAPE_SQUARE, g_sq));

    // Upload new mission sequence: Square -> Circle
    UICommandPacket new_data_pkt = {};
    new_data_pkt.msg_type = MSG_TYPE_UI_CMD;
    new_data_pkt.command = UI_CMD_SET_SEQUENCE;
    new_data_pkt.sequence_length = 2;
    new_data_pkt.goal_sequence[0] = SHAPE_SQUARE;
    new_data_pkt.goal_sequence[1] = SHAPE_CIRCLE;
    new_data_pkt.end_action = MISSION_END_HOVER;
    node.handleCommand(new_data_pkt);

    // Goal memory must be retained
    TrackedGoal g_check;
    EXPECT_TRUE(node.getEstimator().getLastKnownEstimate(SHAPE_CIRCLE, g_check));
    EXPECT_NEAR(g_check.pose.x, 1.5, 0.1);
    EXPECT_TRUE(node.getEstimator().getLastKnownEstimate(SHAPE_SQUARE, g_check));
    EXPECT_NEAR(g_check.pose.x, 4.5, 0.1);

    // Drone targets the new sequence goal (Square)
    FlightCommand next_cmd = node.tick(current, dt);
    EXPECT_TRUE(next_cmd.enable);
    EXPECT_GT(next_cmd.target_x, 3.0);
}

TEST(AutopilotTest, TraversalStrictlyAlongGateNormalRegardlessOfStartingPose) {
    AutopilotNode node;

    // Mission with 2 gates arranged in reverse spatial order (Gate 1 at X=3.0, Gate 2 at X=1.5)
    // Both gates have yaw = 0.0 (normal vector points in +X direction)
    UICommandPacket cmd_pkt = {};
    cmd_pkt.msg_type = MSG_TYPE_UI_CMD;
    cmd_pkt.command = UI_CMD_START_MISSION;
    cmd_pkt.sequence_length = 2;
    cmd_pkt.goal_sequence[0] = SHAPE_CIRCLE;
    cmd_pkt.goal_sequence[1] = SHAPE_TRIANGLE;
    cmd_pkt.end_action = MISSION_END_HOVER;
    node.handleCommand(cmd_pkt);

    // Starting pose is at X=4.0 (BEHIND Gate 1 at X=3.0)
    Pose current = {4.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    double dt = 0.05;

    VisionPacket v_pkt = {};
    v_pkt.msg_type = MSG_TYPE_VISION;
    v_pkt.num_goals_reported = 2;

    v_pkt.goals[0].shape_type = SHAPE_CIRCLE;
    v_pkt.goals[0].is_detected = 1;
    v_pkt.goals[0].num_hypotheses = 1;
    v_pkt.goals[0].hypotheses[0].position_cam = {3.0f, 0.0f, 1.0f};
    v_pkt.goals[0].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f}; // yaw = 0.0
    v_pkt.goals[0].hypotheses[0].confidence = 1.0f;

    v_pkt.goals[1].shape_type = SHAPE_TRIANGLE;
    v_pkt.goals[1].is_detected = 1;
    v_pkt.goals[1].num_hypotheses = 1;
    v_pkt.goals[1].hypotheses[0].position_cam = {1.5f, 0.0f, 1.0f};
    v_pkt.goals[1].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f}; // yaw = 0.0
    v_pkt.goals[1].hypotheses[0].confidence = 1.0f;

    // Takeoff to PATH_FOLLOWING
    node.getEstimator().updateEstimates(v_pkt, 1.0);
    node.tick(current, dt); // Takeoff -> PATH_FOLLOWING
    node.tick(current, dt); // Executes PATH_FOLLOWING for Gate 1

    // Gate 1 entrance standoff is at X = 3.0 - 0.75 = 2.25
    // Normal approach must be strictly +X (dx=1.0, dy=0.0), regardless of starting at X=4.0
    EXPECT_NEAR(node.approach_dx, 1.0, 1e-3);
    EXPECT_NEAR(node.approach_dy, 0.0, 1e-3);

    // Run kinematic mock until Gate 1 is completed and Gate 2 is targeted
    int ticks = 0;
    while (node.current_seq_index == 0 && ticks < 200) {
        node.getEstimator().updateEstimates(v_pkt, 1.0 + ticks * dt);
        FlightCommand cmd = node.tick(current, dt);
        if (cmd.enable) {
            current.x = cmd.target_x;
            current.y = cmd.target_y;
            current.z = cmd.target_z;
            current.yaw = cmd.target_yaw;
        }
        ticks++;
    }

    EXPECT_EQ(node.current_seq_index, 1u);

    // Now targeting Gate 2 (Triangle at X=1.5).
    // Gate 2 is behind Gate 1 (X=1.5 < 3.0).
    // In old code, seq_dx = 1.5 - 3.0 = -1.5, causing dot_seq < 0, flipping approach_dx to -1.0.
    // In new code, starting pose is irrelevant: Gate 2 normal is yaw=0 -> approach_dx = +1.0!
    EXPECT_NEAR(node.approach_dx, 1.0, 1e-3);
    EXPECT_NEAR(node.approach_dy, 0.0, 1e-3);
}

TEST(AutopilotTest, NonTargetGoalRepulsionZoneDeflection) {
    AutopilotNode node;

    // Mission sequence: Target is Square (shape 3)
    UICommandPacket cmd_pkt = {};
    cmd_pkt.msg_type = MSG_TYPE_UI_CMD;
    cmd_pkt.command = UI_CMD_START_MISSION;
    cmd_pkt.sequence_length = 1;
    cmd_pkt.goal_sequence[0] = SHAPE_SQUARE;
    cmd_pkt.end_action = MISSION_END_HOVER;
    node.handleCommand(cmd_pkt);

    // Drone position at takeoff
    Pose current = {0.0, 0.0, 1.0, 0, 0, 0, 0, 1.0};
    double dt = 0.05;

    // Target goal (Square) at (3.5, 0.0, 1.0), yaw = 0
    // Non-target obstacle goal (Circle) at (1.8, 0.05, 1.0), directly in the flight corridor!
    VisionPacket v_pkt = {};
    v_pkt.msg_type = MSG_TYPE_VISION;
    v_pkt.num_goals_reported = 2;

    v_pkt.goals[0].shape_type = SHAPE_CIRCLE; // Non-target obstacle
    v_pkt.goals[0].is_detected = 1;
    v_pkt.goals[0].num_hypotheses = 1;
    v_pkt.goals[0].hypotheses[0].position_cam = {1.8f, 0.05f, 1.0f};
    v_pkt.goals[0].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    v_pkt.goals[0].hypotheses[0].confidence = 0.95f;

    v_pkt.goals[1].shape_type = SHAPE_SQUARE; // Target goal
    v_pkt.goals[1].is_detected = 1;
    v_pkt.goals[1].num_hypotheses = 1;
    v_pkt.goals[1].hypotheses[0].position_cam = {3.5f, 0.0f, 1.0f};
    v_pkt.goals[1].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    v_pkt.goals[1].hypotheses[0].confidence = 0.95f;

    // Update estimates so both goals are known
    node.getEstimator().updateEstimates(v_pkt, 1.0);

    // First tick takeoff -> PATH_FOLLOWING
    node.tick(current, dt);

    // Place drone near the non-target obstacle: X = 1.3, Y = 0.0
    // Desired path without repulsion would be straight along Y = 0.0, which clips Circle at (1.8, 0.05)
    current.x = 1.3;
    current.y = 0.0;
    current.z = 1.0;

    // Even if Circle is now out of camera view (simulated by is_detected = 0 for Circle)
    v_pkt.goals[0].is_detected = 0;
    node.getEstimator().updateEstimates(v_pkt, 1.1);

    // Tick the node. Repulsion from Circle must actively deflect the commanded target in -Y
    FlightCommand cmd = node.tick(current, dt);
    EXPECT_TRUE(cmd.enable);

    // Commanded target must be deflected laterally away from Circle (Y < -0.15)
    EXPECT_LT(cmd.target_y, -0.15);

    // Verify distance from commanded waypoint to the obstacle center is safely outside the physical frame
    double dist_cmd_to_obs = std::hypot(cmd.target_x - 1.8, cmd.target_y - 0.05);
    EXPECT_GE(dist_cmd_to_obs, 0.50);
}
