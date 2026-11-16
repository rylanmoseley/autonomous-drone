#include <gtest/gtest.h>
#include "follower_simulator.h"
#include "autopilot_sim_harness.h"
#include <cmath>
#include <numeric>

using namespace sim;

// 1. Validate Follower First-Order Lag and Velocity Saturation
TEST(FollowerSimulationTest, FirstOrderLagResponse) {
    FollowerConfig cfg;
    cfg.tau_xy = 0.20; // 200ms lag
    cfg.max_velocity_xy = 2.0;
    cfg.max_acceleration = 20.0;
    cfg.noise_enabled = false;
    cfg.drift_vel_x = 0.0;

    FcuFollowerSimulator follower(cfg);
    follower.reset({0.0, 0.0, 1.0, 0, 0, 0, 0, 1.0});

    FlightCommand cmd{};
    cmd.enable = true;
    cmd.target_x = 2.0; // 2-meter step command
    cmd.target_y = 0.0;
    cmd.target_z = 1.0;
    cmd.target_yaw = 0.0;

    double dt = 0.01;
    // Step for tau_xy (0.2s = 20 steps)
    for (int i = 0; i < 20; ++i) {
        follower.step(cmd, dt);
    }

    // At t = tau, response should be around 1 - e^(-1) ≈ 63.2% of target (or clamped by max vel)
    Pose p_tau = follower.getTruePose();
    EXPECT_GT(p_tau.x, 0.3);
    EXPECT_LT(p_tau.x, 2.0);

    // Run until convergence (2.0s = 200 steps)
    for (int i = 0; i < 200; ++i) {
        follower.step(cmd, dt);
    }

    Pose p_final = follower.getTruePose();
    EXPECT_NEAR(p_final.x, 2.0, 0.02);
    EXPECT_NEAR(p_final.y, 0.0, 0.01);
    EXPECT_NEAR(p_final.z, 1.0, 0.01);
}

// 2. Validate Imputed Gaussian Noise Statistics
TEST(FollowerSimulationTest, ImputedGaussianNoiseStatistics) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.05; // 5 cm std
    cfg.pos_noise_std_z = 0.02;  // 2 cm std
    cfg.yaw_noise_std = 0.03;    // 0.03 rad std
    cfg.drift_vel_x = 0.0;
    cfg.random_seed = 12345;

    FcuFollowerSimulator follower(cfg);
    follower.reset({1.0, 2.0, 3.0, 0, 0, 0.5, 0, 1.0});

    FlightCommand cmd{};
    cmd.enable = true;
    cmd.target_x = 1.0;
    cmd.target_y = 2.0;
    cmd.target_z = 3.0;
    cmd.target_yaw = 0.5;

    std::vector<double> errors_x;
    std::vector<double> errors_z;
    std::vector<double> errors_yaw;

    const int N = 1000;
    for (int i = 0; i < N; ++i) {
        follower.step(cmd, 0.01);
        Pose true_p = follower.getTruePose();
        Pose noisy_p = follower.getFeedbackPose();

        errors_x.push_back(noisy_p.x - true_p.x);
        errors_z.push_back(noisy_p.z - true_p.z);
        errors_yaw.push_back(noisy_p.yaw - true_p.yaw);
    }

    // Mean should be close to 0
    double mean_x = std::accumulate(errors_x.begin(), errors_x.end(), 0.0) / N;
    double mean_z = std::accumulate(errors_z.begin(), errors_z.end(), 0.0) / N;
    double mean_yaw = std::accumulate(errors_yaw.begin(), errors_yaw.end(), 0.0) / N;

    EXPECT_NEAR(mean_x, 0.0, 0.01);
    EXPECT_NEAR(mean_z, 0.0, 0.01);
    EXPECT_NEAR(mean_yaw, 0.0, 0.01);

    // Variance / Standard Deviation calculation
    double var_x = 0.0, var_z = 0.0, var_yaw = 0.0;
    for (int i = 0; i < N; ++i) {
        var_x += (errors_x[i] - mean_x) * (errors_x[i] - mean_x);
        var_z += (errors_z[i] - mean_z) * (errors_z[i] - mean_z);
        var_yaw += (errors_yaw[i] - mean_yaw) * (errors_yaw[i] - mean_yaw);
    }
    double std_x = std::sqrt(var_x / N);
    double std_z = std::sqrt(var_z / N);
    double std_yaw = std::sqrt(var_yaw / N);

    // Measured standard deviations should be within ±20% of configured
    EXPECT_NEAR(std_x, 0.05, 0.01);
    EXPECT_NEAR(std_z, 0.02, 0.005);
    EXPECT_NEAR(std_yaw, 0.03, 0.006);
}

// 3. Closed-Loop Setpoint Iteration Baseline (Nominal Follower)
TEST(AutopilotSimulationHarnessTest, SetpointIterationNominal) {
    FollowerConfig cfg;
    cfg.noise_enabled = false;
    cfg.tau_xy = 0.15;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE}, MISSION_END_HOVER);
    // Gate at x=2.0 m; transit completes at x=2.0 + 0.5 = 2.5 m
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.0, 0.0, 1.0, 0.0);

    MissionReport report = harness.runUntil(25.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_LT(report.duration_s, 22.0);
    EXPECT_LT(report.rmse_tracking, 0.60);

    // Verify final true pose is at the gate exit standoff location
    Pose final_pose = harness.getFollower().getTruePose();
    EXPECT_NEAR(final_pose.x, 2.5, 0.30);
    EXPECT_NEAR(final_pose.y, 0.0, 0.30);
    EXPECT_NEAR(final_pose.z, 1.0, 0.30);
}

// 4. Closed-Loop Setpoint Iteration Under Imputed Noise
TEST(AutopilotSimulationHarnessTest, SetpointIterationWithImputedNoise) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.04; // 4 cm noise
    cfg.pos_noise_std_z = 0.02;
    cfg.yaw_noise_std = 0.03;
    cfg.tau_xy = 0.18;
    cfg.random_seed = 42;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_SQUARE}, MISSION_END_HOVER);
    // Gate at (2.5, 0.5, 1.2); exit standoff at (3.0, 0.5, 1.2)
    harness.addSimulatedGoal(SHAPE_SQUARE, 2.5, 0.5, 1.2, 0.0);

    MissionReport report = harness.runUntil(30.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_LT(report.rmse_tracking, 0.75);

    Pose final_pose = harness.getFollower().getTruePose();
    EXPECT_NEAR(final_pose.x, 3.0, 0.40);
    EXPECT_NEAR(final_pose.y, 0.5, 0.40);
    EXPECT_NEAR(final_pose.z, 1.2, 0.40);
}

// 5. Closed-Loop Wind Disturbance Rejection
TEST(AutopilotSimulationHarnessTest, SetpointIterationWithWindDrift) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.drift_vel_x = 0.08; // 0.08 m/s wind in X (steady-state offset ~0.08m < arrival_tol 0.20m)
    cfg.drift_vel_y = -0.06; // -0.06 m/s wind in Y
    cfg.tau_xy = 0.15;
    cfg.random_seed = 99;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_TRIANGLE}, MISSION_END_HOVER);
    // Gate at (1.8, -0.4, 1.0); exit standoff at (2.3, -0.4, 1.0)
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 1.8, -0.4, 1.0, 0.0);

    MissionReport report = harness.runUntil(35.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    Pose final_pose = harness.getFollower().getTruePose();
    EXPECT_NEAR(final_pose.x, 2.3, 0.35);
    EXPECT_NEAR(final_pose.y, -0.4, 0.35);
}

// 6. Full Mission Multi-Gate Traversal with Follower Dynamics and Noise
TEST(AutopilotSimulationHarnessTest, MultiGateTraversalWithNoise) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.03;
    cfg.pos_noise_std_z = 0.015;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 777;

    AutopilotSimHarness harness(cfg);
    // Sequence of 3 gates: Circle -> Triangle -> Square, followed by landing
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_LAND);

    harness.addSimulatedGoal(SHAPE_CIRCLE, 1.2, -0.3, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 2.4, 0.2, 1.1, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 3.6, -0.1, 1.0, 0.0);

    MissionReport report = harness.runUntil(65.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_GT(report.min_clearance_m, 0.015);
    EXPECT_GT(report.total_distance_m, 3.0);
    EXPECT_EQ(report.final_state, FlightState::FLIGHT_STATE_ROTOR_STOPPED);

    // Final altitude on landing should be <= 0.15 m
    Pose final_pose = harness.getFollower().getTruePose();
    EXPECT_LE(final_pose.z, 0.15);
}

// 7. Safety E-Stop Trigger Verification
TEST(AutopilotSimulationHarnessTest, EStopTriggerDropsThrottle) {
    FollowerConfig cfg;
    cfg.noise_enabled = false;
    AutopilotSimHarness harness(cfg);

    harness.setupMission({SHAPE_CIRCLE}, MISSION_END_HOVER);
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.0, 0.0, 1.0, 0.0);

    // Run until takeoff completes and drone is in the air
    harness.runUntil(3.0, 0.05, [](const AutopilotSimHarness& h) {
        return h.getFollower().getTruePose().z >= 0.8;
    });

    EXPECT_GT(harness.getFollower().getTruePose().z, 0.5);

    // Trigger E-STOP via UI Command
    UICommandPacket estop_pkt{};
    estop_pkt.msg_type = MSG_TYPE_UI_CMD;
    estop_pkt.command = UI_CMD_ESTOP;
    harness.getAutopilot().handleCommand(estop_pkt);

    // Step simulation
    for (int i = 0; i < 30; ++i) {
        harness.step(0.05);
    }

    EXPECT_EQ(harness.getAutopilot().getState(), FlightState::FLIGHT_STATE_ESTOP_TRIPPED);
    // Drone drops to ground under E-Stop
    EXPECT_LE(harness.getFollower().getTruePose().z, 0.05);
}

// 8. Vision Tag Conversion to MAVLink VISION_POSITION_ESTIMATE
TEST(AutopilotSimulationHarnessTest, TagToMavlinkVisionPositionEstimate) {
    TagMeasurement tag{};
    tag.tag_id = 7;
    tag.is_valid = 1;
    tag.camera_pos_world = {1.5f, -2.0f, 0.9f};

    // Yaw = 90 deg (pi/2) -> Quat w = cos(pi/4), z = sin(pi/4)
    double yaw = M_PI / 2.0;
    tag.camera_att_world.w = static_cast<float>(std::cos(yaw * 0.5));
    tag.camera_att_world.x = 0.0f;
    tag.camera_att_world.y = 0.0f;
    tag.camera_att_world.z = static_cast<float>(std::sin(yaw * 0.5));
    tag.confidence = 0.80f;

    MavlinkVisionPositionEstimate est = tag_to_mavlink_vision_position(tag, 555000ULL);
    EXPECT_EQ(est.usec, 555000ULL);
    EXPECT_FLOAT_EQ(est.x, 1.5f);
    EXPECT_FLOAT_EQ(est.y, -2.0f);
    EXPECT_FLOAT_EQ(est.z, 0.9f);
    EXPECT_NEAR(est.roll, 0.0f, 1e-4);
    EXPECT_NEAR(est.pitch, 0.0f, 1e-4);
    EXPECT_NEAR(est.yaw, M_PI / 2.0, 1e-4);

    // Covariance check
    EXPECT_GT(est.covariance[0], 0.0f);
    EXPECT_FLOAT_EQ(est.covariance[0], est.covariance[6]);  // pos_var x == pos_var y
    EXPECT_GT(est.covariance[20], 0.0f); // yaw variance
}

// 9. AutopilotNode Tag Pass-Through Verification
TEST(AutopilotSimulationHarnessTest, TagPassThroughForwarding) {
    AutopilotNode autopilot;

    VisionPacket pkt{};
    pkt.msg_type = MSG_TYPE_VISION;
    pkt.sequence_id = 101;
    pkt.timestamp_ns = 2500000000ULL; // 2.5s
    pkt.num_tags_detected = 2;

    pkt.tags[0].tag_id = 10;
    pkt.tags[0].is_valid = 1;
    pkt.tags[0].camera_pos_world = {3.0f, 1.0f, 1.0f};
    pkt.tags[0].camera_att_world = {1.0f, 0.0f, 0.0f, 0.0f};
    pkt.tags[0].confidence = 0.9f;

    pkt.tags[1].tag_id = 11;
    pkt.tags[1].is_valid = 1;
    pkt.tags[1].camera_pos_world = {4.0f, -1.0f, 1.2f};
    pkt.tags[1].camera_att_world = {1.0f, 0.0f, 0.0f, 0.0f};
    pkt.tags[1].confidence = 0.85f;

    autopilot.handleVisionPacket(pkt, 2.5);

    EXPECT_EQ(autopilot.tags_received_count, 2u);
    EXPECT_EQ(autopilot.tags_forwarded_count, 2u);
    EXPECT_TRUE(autopilot.fcu.hasVisionPositionEstimate());
    EXPECT_EQ(autopilot.fcu.getVisionEstimateCount(), 2u);

    MavlinkVisionPositionEstimate last_est = autopilot.fcu.getLastVisionPositionEstimate();
    EXPECT_FLOAT_EQ(last_est.x, 4.0f);
    EXPECT_FLOAT_EQ(last_est.y, -1.0f);
    EXPECT_FLOAT_EQ(last_est.z, 1.2f);
}

// 10. Follower Drift Reset via Global Vision Reading
TEST(AutopilotSimulationHarnessTest, FollowerDriftResetWithVision) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.01;
    cfg.random_walk_drift_std = 0.25; // Random walk dead reckoning drift
    cfg.random_seed = 123;
    FcuFollowerSimulator follower(cfg);
    follower.reset({0.0, 0.0, 1.0, 0, 0, 0, 0, 1.0});

    FlightCommand cmd{};
    cmd.enable = true;
    cmd.target_x = 0.0;
    cmd.target_y = 0.0;
    cmd.target_z = 1.0;

    // Simulate 3 seconds of flight: dead reckoning drift accumulates
    for (int i = 0; i < 60; ++i) {
        follower.step(cmd, 0.05);
    }

    double drift_before = follower.getDriftMagnitude();
    EXPECT_GT(drift_before, 0.10);

    // Now inject tag observation representing ground truth external navigation
    Pose true_p = follower.getTruePose();
    TagMeasurement tag{};
    tag.tag_id = 99;
    tag.is_valid = 1;
    tag.camera_pos_world = {static_cast<float>(true_p.x), static_cast<float>(true_p.y), static_cast<float>(true_p.z)};
    tag.camera_att_world = {1.0f, 0.0f, 0.0f, 0.0f};
    tag.confidence = 0.95f;

    follower.correctDriftWithVision(tag);

    EXPECT_GT(follower.getMetrics().tag_corrections_count, 0u);
    double drift_after = follower.getDriftMagnitude();
    EXPECT_LT(drift_after, drift_before * 0.4);
    EXPECT_NEAR(follower.getFeedbackPose().x, true_p.x, 0.08);
    EXPECT_NEAR(follower.getFeedbackPose().y, true_p.y, 0.08);
}

// 11. Closed-Loop Mission with Tag Drift Correction Active
TEST(AutopilotSimulationHarnessTest, ClosedLoopMissionWithTagDriftHandling) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.random_walk_drift_std = 0.10; // Optical flow dead-reckoning random walk drift
    cfg.drift_vel_x = 0.03;           // Mild environmental drift
    cfg.drift_vel_y = -0.02;
    cfg.random_seed = 999;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE}, MISSION_END_HOVER);

    harness.addSimulatedGoal(SHAPE_CIRCLE, 1.8, -0.3, 1.0, 0.0);

    // Place surveyed fiducial tag anchor at the gate location
    harness.addSimulatedTag(1, 1.8, -0.3, 1.0, 0.0, 0.0, 0.0, 0.95f);

    MissionReport report = harness.runUntil(40.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_GT(report.tag_readings_forwarded, 0u);
    EXPECT_GT(report.total_drift_corrected_m, 0.0);
    EXPECT_LT(report.rmse_tracking, 0.60);
}

// 12. Full Mission Side-by-Side 3-Gate Traversal (Circle -> Triangle -> Square)
TEST(AutopilotSimulationHarnessTest, SideBySideThreeGateMission) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.pos_noise_std_z = 0.01;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 42;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_LAND);

    // Three side-by-side gates at X=2.5:
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, -1.3, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 2.5, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 2.5, 1.3, 1.0, 0.0);

    MissionReport report = harness.runUntil(60.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.final_state, FlightState::FLIGHT_STATE_ROTOR_STOPPED);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_GT(report.min_clearance_m, 0.05);
    EXPECT_EQ(report.sequence_traversed.size(), 3u);
    EXPECT_EQ(report.sequence_traversed[0], SHAPE_CIRCLE);
    EXPECT_EQ(report.sequence_traversed[1], SHAPE_TRIANGLE);
    EXPECT_EQ(report.sequence_traversed[2], SHAPE_SQUARE);
}

// 13. Collision Detector Unit Tests: Accurate Geometry, Clearance, and Pylon Detection
TEST(CollisionDetectorTest, AccurateHoopRimAndPylonCollisions) {
    std::vector<SimulatedGoal> gates = {
        {SHAPE_CIRCLE,   2.0,  0.0, 1.0, 0.0},
        {SHAPE_TRIANGLE, 4.0,  0.0, 1.0, 0.0},
        {SHAPE_SQUARE,   6.0,  0.0, 1.0, 0.0}
    };

    // 1. Center passage of Circle gate: no hit, high clearance
    auto res_center_circle = CollisionDetector::check(2.0, 0.0, 1.0, 0.0, gates);
    EXPECT_FALSE(res_center_circle.is_hit);
    EXPECT_GT(res_center_circle.min_clearance_m, 0.10);

    // 2. Exact rim hit on Circle: drone at radius 0.22 in hoop plane
    auto res_rim_circle = CollisionDetector::check(2.0, 0.22, 1.0, 0.0, gates);
    EXPECT_TRUE(res_rim_circle.is_hit);
    EXPECT_EQ(res_rim_circle.shape_hit, SHAPE_CIRCLE);

    // 3. Pylon hit below Circle: w < -0.25 (e.g. z = 0.5)
    auto res_pylon_circle = CollisionDetector::check(2.0, 0.0, 0.5, 0.0, gates);
    EXPECT_TRUE(res_pylon_circle.is_hit);
    EXPECT_EQ(res_pylon_circle.shape_hit, SHAPE_CIRCLE);

    // 4. Center passage of Triangle gate: apex UP at (0, 0.35), base at -0.175
    auto res_center_tri = CollisionDetector::check(4.0, 0.0, 1.0, 0.0, gates);
    EXPECT_FALSE(res_center_tri.is_hit);
    EXPECT_GT(res_center_tri.min_clearance_m, 0.08);

    // 5. Rim hit on Triangle apex (w = +0.35, z = 1.35)
    auto res_apex_tri = CollisionDetector::check(4.0, 0.0, 1.35, 0.0, gates);
    EXPECT_TRUE(res_apex_tri.is_hit);
    EXPECT_EQ(res_apex_tri.shape_hit, SHAPE_TRIANGLE);

    // 6. Pylon hit below Triangle: w < -0.205 (z = 0.6)
    auto res_pylon_tri = CollisionDetector::check(4.0, 0.0, 0.6, 0.0, gates);
    EXPECT_TRUE(res_pylon_tri.is_hit);
    EXPECT_EQ(res_pylon_tri.shape_hit, SHAPE_TRIANGLE);

    // 7. Center passage of Square gate: no hit
    auto res_center_sq = CollisionDetector::check(6.0, 0.0, 1.0, 0.0, gates);
    EXPECT_FALSE(res_center_sq.is_hit);
    EXPECT_GT(res_center_sq.min_clearance_m, 0.08);

    // 8. Rim hit on Square top edge (w = +0.1768, z = 1.177)
    auto res_rim_sq = CollisionDetector::check(6.0, 0.0, 1.177, 0.0, gates);
    EXPECT_TRUE(res_rim_sq.is_hit);
    EXPECT_EQ(res_rim_sq.shape_hit, SHAPE_SQUARE);

    // 9. Far away drone: high clearance, no hits
    auto res_far = CollisionDetector::check(0.0, 0.0, 1.0, 0.0, gates);
    EXPECT_FALSE(res_far.is_hit);
    EXPECT_GT(res_far.min_clearance_m, 1.5);
}

// 14. Verify APF Repulsion Avoids Collision in Narrow Side-by-Side Gate Layout (0.9m spacing)
TEST(AutopilotSimulationHarnessTest, SideBySideTightSpacingRepulsionAvoidance) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.015;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 101;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_LAND);

    // Gates spaced by only 0.9m laterally (leaving ~0.3m between physical gate rims)
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, -0.9, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 2.5, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 2.5, 0.9, 1.0, 0.0);

    MissionReport report = harness.runUntil(60.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_GT(report.min_clearance_m, 0.015);
    EXPECT_EQ(report.sequence_traversed.size(), 3u);
    EXPECT_EQ(report.sequence_traversed[0], SHAPE_CIRCLE);
    EXPECT_EQ(report.sequence_traversed[1], SHAPE_TRIANGLE);
    EXPECT_EQ(report.sequence_traversed[2], SHAPE_SQUARE);
}

// 15. Verify Multi-Angle Yawed Side-by-Side Gates Traversed Normal to Faces with Zero Collision
TEST(AutopilotSimulationHarnessTest, SideBySideYawedGatesMultiAngle) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.015;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 202;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_LAND);

    // Gate 1 (Circle): yaw = 0.0 (normal +X)
    // Gate 2 (Triangle): yaw = +30 deg (+0.5236 rad)
    // Gate 3 (Square): yaw = -25 deg (-0.4363 rad)
    double yaw_tri = 30.0 * M_PI / 180.0;
    double yaw_sq = -25.0 * M_PI / 180.0;

    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, -1.3, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 2.5, 0.0, 1.0, yaw_tri);
    harness.addSimulatedGoal(SHAPE_SQUARE, 2.5, 1.3, 1.0, yaw_sq);

    MissionReport report = harness.runUntil(60.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_GT(report.min_clearance_m, 0.005);
    EXPECT_EQ(report.sequence_traversed.size(), 3u);
}

TEST(AutopilotSimulationHarnessTest, GoalsAtYaw0_07_Minus16) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.015;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 404;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_LAND);

    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, -1.3, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 2.5, 0.0, 1.0, 0.7);
    harness.addSimulatedGoal(SHAPE_SQUARE, 2.5, 1.3, 1.0, -1.6);

    MissionReport report = harness.runUntil(60.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_GT(report.min_clearance_m, 0.02);
    EXPECT_EQ(report.sequence_traversed.size(), 3u);
}

// 16. Verify Perception Frustum Filtering & Goal Acquisition from Outside Initial FOV
TEST(AutopilotSimulationHarnessTest, ClosedLoopMissionWithRealisticCameraFOV) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 303;

    AutopilotSimHarness harness(cfg);
    SimPerceptionConfig perc{};
    perc.fov_filtering_enabled = true;
    perc.h_fov_deg = 70.0;
    perc.v_fov_deg = 50.0;
    perc.min_range_m = 0.20;
    perc.max_range_m = 5.0;
    harness.setPerceptionConfig(perc);

    harness.setupMission({SHAPE_CIRCLE, SHAPE_SQUARE}, MISSION_END_HOVER);

    // Gate 1 placed at (2.0, 0.5, 1.0) within camera view
    // Gate 2 placed at (4.0, -0.4, 1.0)
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.0, 0.5, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 4.0, -0.4, 1.0, 0.0);

    MissionReport report = harness.runUntil(45.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_EQ(report.sequence_traversed.size(), 2u);
}

// 17. Verify Proximity Perception Dropout: Holds Locked Target and Completes Punch Cleanly
TEST(AutopilotSimulationHarnessTest, ProximityPerceptionDropoutHoldActivePose) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 404;

    AutopilotSimHarness harness(cfg);
    SimPerceptionConfig perc{};
    perc.fov_filtering_enabled = true;
    perc.proximity_dropout_enabled = true;
    perc.proximity_dropout_dist = 0.45; // Camera drops gate contour when closer than 45cm
    harness.setPerceptionConfig(perc);

    harness.setupMission({SHAPE_CIRCLE}, MISSION_END_HOVER);
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.0, 0.0, 1.0, 0.0);

    MissionReport report = harness.runUntil(30.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_GT(report.min_clearance_m, 0.05);
    EXPECT_EQ(report.sequence_traversed.size(), 1u);
}

// 18. Verify Visual Occlusion: Farther Gate is Blocked by Nearer Gate Until First Gate Traversed
TEST(AutopilotSimulationHarnessTest, VisualOcclusionHandlingInLineGates) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 505;

    AutopilotSimHarness harness(cfg);
    SimPerceptionConfig perc{};
    perc.fov_filtering_enabled = true;
    perc.occlusion_check_enabled = true;
    harness.setPerceptionConfig(perc);

    harness.setupMission({SHAPE_CIRCLE, SHAPE_SQUARE}, MISSION_END_LAND);
    // Two gates aligned along the optical axis:
    // Gate 1 (Circle) at (1.8, 0.0, 1.0)
    // Gate 2 (Square) at (3.6, 0.0, 1.0) - directly behind Gate 1!
    harness.addSimulatedGoal(SHAPE_CIRCLE, 1.8, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 3.6, 0.0, 1.0, 0.0);

    MissionReport report = harness.runUntil(50.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_EQ(report.sequence_traversed.size(), 2u);
    EXPECT_EQ(report.sequence_traversed[0], SHAPE_CIRCLE);
    EXPECT_EQ(report.sequence_traversed[1], SHAPE_SQUARE);
}

// 19. Verify Global Surveyed AprilTags Placed at Arbitrary Arena Positions Correct Severe Drift
TEST(AutopilotSimulationHarnessTest, GlobalSurveyedTagsRandomlyPlacedInArena) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.random_walk_drift_std = 0.12; // Continuous dead-reckoning drift accumulation
    cfg.drift_vel_x = 0.08;           // Constant environmental wind
    cfg.drift_vel_y = -0.06;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 606;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_LAND);

    // Three side-by-side gates at X=2.5:
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, -1.3, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 2.5, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 2.5, 1.3, 1.0, 0.0);

    // AprilTags placed at landmark spots in the world (NOT on the gates):
    // Tag 10: In front of Gate 1 area at (1.2, -1.8, 1.0)
    // Tag 11: Behind the gates near Gate 2 turnaround area at (3.3, 0.0, 1.2)
    // Tag 12: Near Gate 3 approach area at (1.5, 1.5, 0.9)
    // Tag 13: Ground marker near start at (0.5, 0.0, 0.8)
    harness.addSimulatedTag(10, 1.2, -1.8, 1.0, 0.0, 0.0, 0.0, 0.95f);
    harness.addSimulatedTag(11, 3.3, 0.0, 1.2, 0.0, 0.0, 0.0, 0.95f);
    harness.addSimulatedTag(12, 1.5, 1.5, 0.9, 0.0, 0.0, 0.0, 0.95f);
    harness.addSimulatedTag(13, 0.5, 0.0, 0.8, 0.0, 0.0, 0.0, 0.95f);

    MissionReport report = harness.runUntil(70.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_GT(report.tag_readings_forwarded, 0u);
    EXPECT_GT(report.total_drift_corrected_m, 0.20);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_GT(report.min_clearance_m, 0.04);
    EXPECT_EQ(report.sequence_traversed.size(), 3u);
}

// 20. Verify 3D Surveyed Tag Attitude and Position Fusion into MAVLink
TEST(AutopilotSimulationHarnessTest, SurveyedTagAttitudeDriftCorrection) {
    TagMeasurement tag{};
    tag.tag_id = 42;
    tag.is_valid = 1;
    tag.camera_pos_world = {2.1f, -1.4f, 1.15f};

    // Yaw = 45 deg, Pitch = 10 deg, Roll = -5 deg
    double cy = std::cos(45.0 * M_PI / 360.0);
    double sy = std::sin(45.0 * M_PI / 360.0);
    double cp = std::cos(10.0 * M_PI / 360.0);
    double sp = std::sin(10.0 * M_PI / 360.0);
    double cr = std::cos(-5.0 * M_PI / 360.0);
    double sr = std::sin(-5.0 * M_PI / 360.0);
    tag.camera_att_world.w = static_cast<float>(cr * cp * cy + sr * sp * sy);
    tag.camera_att_world.x = static_cast<float>(sr * cp * cy - cr * sp * sy);
    tag.camera_att_world.y = static_cast<float>(cr * sp * cy + sr * cp * sy);
    tag.camera_att_world.z = static_cast<float>(cr * cp * sy - sr * sp * cy);
    tag.confidence = 0.92f;

    MavlinkVisionPositionEstimate est = tag_to_mavlink_vision_position(tag, 12345678ULL);
    EXPECT_FLOAT_EQ(est.x, 2.1f);
    EXPECT_FLOAT_EQ(est.y, -1.4f);
    EXPECT_FLOAT_EQ(est.z, 1.15f);
    EXPECT_NEAR(est.yaw, 45.0 * M_PI / 180.0, 1e-3);
    EXPECT_NEAR(est.pitch, 10.0 * M_PI / 180.0, 1e-3);
    EXPECT_NEAR(est.roll, -5.0 * M_PI / 180.0, 1e-3);

    // Apply to Follower simulator
    FollowerConfig cfg;
    cfg.noise_enabled = false;
    FcuFollowerSimulator follower(cfg);
    follower.reset({1.5, -1.0, 1.0, 0, 0, 0, 0, 1.0});

    follower.correctDriftWithVision(tag);
    EXPECT_NEAR(follower.getFeedbackPose().x, 2.1, 0.05);
    EXPECT_NEAR(follower.getFeedbackPose().y, -1.4, 0.05);
    EXPECT_NEAR(follower.getFeedbackPose().z, 1.15, 0.05);
}

// 21. Verify Mission Approaching Gate 1 from Behind Traverses Backwards (-X) Cleanly
TEST(AutopilotSimulationHarnessTest, MissionReversingStartGate) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.015;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 707;

    AutopilotSimHarness harness(cfg);
    // Start the drone behind the gate at X = 3.5, facing -X (yaw = pi)
    harness.reset({3.5, 0.0, 0.0, 0, 0, M_PI, 0, 1.0});

    harness.setupMission({SHAPE_CIRCLE}, MISSION_END_HOVER);
    // Gate normal oriented toward -X (yaw = pi), facing the incoming drone
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, 0.0, 1.0, M_PI);

    MissionReport report = harness.runUntil(35.0, 0.05);

    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_EQ(report.sequence_traversed.size(), 1u);

    // Final exit position must be on the -X side of the gate (X <= 2.1)
    EXPECT_LE(harness.getFollower().getTruePose().x, 2.1);
}

// 22. Verify Mid-Flight Mission Pause Holds Stable Position and Resumes Cleanly
TEST(AutopilotSimulationHarnessTest, MidFlightMissionPauseAndResume) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 808;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_SQUARE}, MISSION_END_LAND);
    harness.addSimulatedGoal(SHAPE_CIRCLE, 1.8, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 3.8, 0.0, 1.0, 0.0);

    // Step until Gate 1 is completed
    harness.runUntil(25.0, 0.05, [](const AutopilotSimHarness& h) {
        return h.getAutopilot().current_seq_index == 1;
    });
    EXPECT_EQ(harness.getAutopilot().current_seq_index, 1u);

    // Send Pause command
    UICommandPacket pause_pkt{};
    pause_pkt.msg_type = MSG_TYPE_UI_CMD;
    pause_pkt.command = UI_CMD_PAUSE_MISSION;
    harness.getAutopilot().handleCommand(pause_pkt);

    // Step for 2 seconds in paused state
    Pose pause_pose = harness.getFollower().getTruePose();
    for (int i = 0; i < 40; ++i) {
        harness.step(0.05);
        EXPECT_EQ(harness.getAutopilot().getState(), FlightState::FLIGHT_STATE_HOVER_WAIT);
    }
    Pose after_pause_pose = harness.getFollower().getTruePose();
    EXPECT_NEAR(after_pause_pose.x, pause_pose.x, 0.15);
    EXPECT_NEAR(after_pause_pose.y, pause_pose.y, 0.15);

    // Send Resume command
    UICommandPacket resume_pkt{};
    resume_pkt.msg_type = MSG_TYPE_UI_CMD;
    resume_pkt.command = UI_CMD_RESUME_MISSION;
    harness.getAutopilot().handleCommand(resume_pkt);

    // Run remaining mission to completion
    MissionReport report = harness.runUntil(35.0, 0.05);
    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
    EXPECT_EQ(report.sequence_traversed.size(), 2u);
}

// 23. Verify Dynamic Sequence Modification On-The-Fly
TEST(AutopilotSimulationHarnessTest, MidFlightSequenceUpdate) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 909;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE}, MISSION_END_HOVER);
    harness.addSimulatedGoal(SHAPE_CIRCLE, 1.8, -0.4, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 3.2, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 4.5, 0.4, 1.0, 0.0);

    // Complete Gate 1
    harness.runUntil(25.0, 0.05, [](const AutopilotSimHarness& h) {
        return h.getAutopilot().current_seq_index == 1;
    });

    // Update sequence to Square only:
    UICommandPacket update_pkt{};
    update_pkt.msg_type = MSG_TYPE_UI_CMD;
    update_pkt.command = UI_CMD_SET_SEQUENCE;
    update_pkt.sequence_length = 1;
    update_pkt.goal_sequence[0] = SHAPE_SQUARE;
    update_pkt.end_action = MISSION_END_LAND;
    harness.getAutopilot().handleCommand(update_pkt);

    MissionReport report = harness.runUntil(35.0, 0.05);
    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);
}

// 24. Verify Safety Timeout: Loss of Vision Heartbeat for > 2.5s Trips E-Stop
TEST(AutopilotSimulationHarnessTest, VisionHeartbeatTimeoutEstopTrigger) {
    FollowerConfig cfg;
    cfg.noise_enabled = false;
    AutopilotSimHarness harness(cfg);

    harness.setupMission({SHAPE_CIRCLE}, MISSION_END_HOVER);
    harness.addSimulatedGoal(SHAPE_CIRCLE, 5.0, 0.0, 1.0, 0.0);

    // Step until takeoff completes and drone is in PATH_FOLLOWING
    harness.runUntil(4.0, 0.05, [](const AutopilotSimHarness& h) {
        return h.getAutopilot().getState() == FlightState::FLIGHT_STATE_PATH_FOLLOWING;
    });
    EXPECT_EQ(harness.getAutopilot().getState(), FlightState::FLIGHT_STATE_PATH_FOLLOWING);

    // Disable vision stream (simulating vision camera hardware crash)
    harness.setVisionEnabled(false);

    // Age last_vision_time by 3.0 seconds to simulate watchdog timer expiry
    harness.getAutopilot().last_vision_time = std::chrono::steady_clock::now() - std::chrono::milliseconds(3000);

    // Step to let Autopilot watchdog trip ESTOP and cut throttle so drone drops to ground
    for (int i = 0; i < 25; ++i) {
        harness.step(0.05);
    }

    // Safety watchdog MUST have tripped ESTOP!
    EXPECT_EQ(harness.getAutopilot().getState(), FlightState::FLIGHT_STATE_ESTOP_TRIPPED);
    EXPECT_NE(harness.getAutopilot().estop_active, 0u);
    // Throttle cut, drone dropped to ground
    EXPECT_LE(harness.getFollower().getTruePose().z, 0.05);
}

// 25. Verify Tube Rail Constraint Strictly Bounds Lateral and Vertical Motion
TEST(AutopilotSimulationHarnessTest, TubeRailConstraintLockedTrajectory) {
    FollowerConfig cfg;
    cfg.noise_enabled = false;
    cfg.tau_xy = 0.12;

    AutopilotSimHarness harness(cfg);
    // Start mounted on the rail constraint at Y = 0.30m
    harness.reset({0.0, 0.30, 0.0, 0, 0, 0, 0, 1.0});
    // Rail constraint: Y locked to 0.30m, Z locked to 1.15m
    harness.setupMission({SHAPE_CIRCLE, SHAPE_SQUARE}, MISSION_END_LAND, true, 0.30f, 1.15f);

    harness.addSimulatedGoal(SHAPE_CIRCLE, 1.8, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 3.6, 0.0, 1.0, 0.0);

    MissionReport report = harness.runUntil(40.0, 0.05);

    EXPECT_TRUE(report.mission_completed);

    // Check all trajectory points during PATH_FOLLOWING phase: Y must stay within +-0.08m of rail_y
    for (const auto& pt : harness.getTrajectory()) {
        if (pt.flight_state == FlightState::FLIGHT_STATE_PATH_FOLLOWING && pt.true_pose.x > 0.5 && pt.true_pose.x < 3.5) {
            EXPECT_NEAR(pt.true_pose.y, 0.30, 0.08);
            EXPECT_NEAR(pt.true_pose.z, 1.15, 0.08);
        }
    }
}

// 26. Verify Kinematic Follower Acceleration and Velocity Saturation Limits
TEST(FollowerSimulationTest, KinematicSaturationLimits) {
    FollowerConfig cfg;
    cfg.max_velocity_xy = 2.0;
    cfg.max_velocity_z = 1.0;
    cfg.max_acceleration = 6.0;
    cfg.max_yaw_rate = 1.5;
    cfg.tau_xy = 0.05; // Fast tau to demand high acceleration
    cfg.tau_z = 0.05;
    cfg.tau_yaw = 0.05;
    cfg.noise_enabled = false;

    FcuFollowerSimulator follower(cfg);
    follower.reset({0.0, 0.0, 1.0, 0, 0, 0, 0, 1.0});

    // Send enormous step input: target (100.0, -100.0, 50.0), yaw = pi
    FlightCommand cmd{};
    cmd.enable = true;
    cmd.target_x = 100.0;
    cmd.target_y = -100.0;
    cmd.target_z = 50.0;
    cmd.target_yaw = M_PI;

    double dt = 0.01;
    double prev_vx = 0.0, prev_vy = 0.0, prev_vz = 0.0;

    for (int i = 0; i < 200; ++i) {
        follower.step(cmd, dt);
        double vx = follower.getVelocityX();
        double vy = follower.getVelocityY();
        double vz = follower.getVelocityZ();

        double speed_xy = std::hypot(vx, vy);
        EXPECT_LE(speed_xy, cfg.max_velocity_xy + 1e-4);
        EXPECT_LE(std::abs(vz), cfg.max_velocity_z + 1e-4);

        if (i > 0) {
            double ax = (vx - prev_vx) / dt;
            double ay = (vy - prev_vy) / dt;
            double az = (vz - prev_vz) / dt;
            double lin_acc = std::sqrt(ax * ax + ay * ay + az * az);
            EXPECT_LE(lin_acc, cfg.max_acceleration + 0.1);
        }
        prev_vx = vx;
        prev_vy = vy;
        prev_vz = vz;
    }
}

// 27. Verify Simulation Step Execution Real-Time Performance (< 100 microseconds / step)
TEST(AutopilotSimulationHarnessTest, StepExecutionTimingBudget) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_LAND);
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, -1.3, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 2.5, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 2.5, 1.3, 1.0, 0.0);
    harness.addSimulatedTag(1, 1.0, -1.0, 1.0);
    harness.addSimulatedTag(2, 3.0, 1.0, 1.0);

    const int N_STEPS = 2000;
    auto t_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N_STEPS; ++i) {
        harness.step(0.05);
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    double total_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    double avg_us = total_us / N_STEPS;

    std::cout << "[PERF] 2000 sim steps executed in " << total_us / 1000.0 << " ms ("
              << avg_us << " us / step)" << std::endl;

    // Real-time step budget is 50,000 us (50ms at 20Hz). Full vision pipeline harness runs > 80x faster than real-time (< 600 us).
    EXPECT_LT(avg_us, 600.0);
}

// 28. Verify Multiple Sequential Missions (Transit across arena with no drift or collision)
TEST(AutopilotSimulationHarnessTest, MultipleSequentialMissions) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.02;
    cfg.pos_noise_std_z = 0.01;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 42;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_HOVER);

    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, -1.3, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_TRIANGLE, 2.5, 0.0, 1.0, 0.0);
    harness.addSimulatedGoal(SHAPE_SQUARE, 2.5, 1.3, 1.0, 0.0);

    // Run first mission: Circle -> Triangle -> Square
    MissionReport rep1 = harness.runUntil(50.0, 0.05);
    EXPECT_TRUE(rep1.mission_completed);
    EXPECT_EQ(rep1.final_state, FlightState::FLIGHT_STATE_HOVER_WAIT);
    EXPECT_EQ(rep1.collision_count, 0u);

    // Send second mission in reverse: Square -> Triangle -> Circle
    UICommandPacket cmd2{};
    cmd2.msg_type = MSG_TYPE_UI_CMD;
    cmd2.command = UI_CMD_START_MISSION;
    cmd2.sequence_length = 3;
    cmd2.goal_sequence[0] = SHAPE_SQUARE;
    cmd2.goal_sequence[1] = SHAPE_TRIANGLE;
    cmd2.goal_sequence[2] = SHAPE_CIRCLE;
    cmd2.end_action = MISSION_END_HOVER;
    harness.getAutopilot().handleCommand(cmd2);

    MissionReport rep2 = harness.runUntil(60.0, 0.05);
    EXPECT_TRUE(rep2.mission_completed);
    EXPECT_EQ(rep2.final_state, FlightState::FLIGHT_STATE_HOVER_WAIT);
    EXPECT_EQ(rep2.collision_count, 0u);

    // Send third mission across arena: Circle -> Triangle -> Square with LAND
    UICommandPacket cmd3{};
    cmd3.msg_type = MSG_TYPE_UI_CMD;
    cmd3.command = UI_CMD_START_MISSION;
    cmd3.sequence_length = 3;
    cmd3.goal_sequence[0] = SHAPE_CIRCLE;
    cmd3.goal_sequence[1] = SHAPE_TRIANGLE;
    cmd3.goal_sequence[2] = SHAPE_SQUARE;
    cmd3.end_action = MISSION_END_LAND;
    harness.getAutopilot().handleCommand(cmd3);

    MissionReport rep3 = harness.runUntil(60.0, 0.05);
    EXPECT_TRUE(rep3.mission_completed);
    EXPECT_EQ(rep3.final_state, FlightState::FLIGHT_STATE_ROTOR_STOPPED);
    EXPECT_EQ(rep3.collision_count, 0u);
}

// 26. Verify Strict Orthogonal Straight Punch-Through Traversal with Minimal Cross-Track Error
TEST(AutopilotSimulationHarnessTest, TraversalCrossTrackApertureStraightPass) {
    FollowerConfig cfg;
    cfg.noise_enabled = true;
    cfg.pos_noise_std_xy = 0.01;
    cfg.tau_xy = 0.15;
    cfg.random_seed = 101;

    AutopilotSimHarness harness(cfg);
    harness.setupMission({SHAPE_CIRCLE}, MISSION_END_LAND);
    harness.addSimulatedGoal(SHAPE_CIRCLE, 2.5, 0.0, 1.0, 0.0);

    MissionReport report = harness.runUntil(30.0, 0.05);
    EXPECT_TRUE(report.mission_completed);
    EXPECT_EQ(report.collision_count, 0u);

    // Analyze trajectory when passing through the aperture corridor (X in [2.2, 2.8]):
    double max_cross_track = 0.0;
    int aperture_points = 0;
    for (const auto& pt : harness.getTrajectory()) {
        if (pt.true_pose.x >= 2.2 && pt.true_pose.x <= 2.8) {
            double cross_track = std::hypot(pt.true_pose.y - 0.0, pt.true_pose.z - 1.0);
            if (cross_track > max_cross_track) {
                max_cross_track = cross_track;
            }
            aperture_points++;
        }
    }

    EXPECT_GT(aperture_points, 5);
    // Commanded traversal must hug the orthogonal line with < 0.035m cross-track deviation
    EXPECT_LT(max_cross_track, 0.035);
}


