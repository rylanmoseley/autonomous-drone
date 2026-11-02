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
    EXPECT_LT(report.rmse_tracking, 0.25);

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
    EXPECT_LT(report.rmse_tracking, 0.35);

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
