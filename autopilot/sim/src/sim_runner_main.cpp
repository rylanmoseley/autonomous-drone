#include "autopilot_sim_harness.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --scenario <name>    Scenario: takeoff, single_gate, multi_gate, drift (default: multi_gate)\n"
              << "  --noise <val>        Position noise standard deviation in meters (default: 0.03)\n"
              << "  --yaw-noise <val>    Yaw noise standard deviation in radians (default: 0.02)\n"
              << "  --drift-x <val>      Wind drift velocity X in m/s (default: 0.0)\n"
              << "  --drift-y <val>      Wind drift velocity Y in m/s (default: 0.0)\n"
              << "  --tau <val>          Follower response time constant in seconds (default: 0.15)\n"
              << "  --seed <val>         PRNG seed for repeatability (default: 42)\n"
              << "  --dt <val>           Simulation timestep in seconds (default: 0.05)\n"
              << "  --csv <filename>     Export trajectory points to CSV\n"
              << "  --help               Show this help message\n";
}

int main(int argc, char** argv) {
    std::string scenario = "multi_gate";
    std::string csv_file = "";
    sim::FollowerConfig cfg{};
    double dt = 0.05;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenario = argv[++i];
        } else if (arg == "--noise" && i + 1 < argc) {
            cfg.pos_noise_std_xy = std::stod(argv[++i]);
            cfg.pos_noise_std_z = cfg.pos_noise_std_xy * 0.5;
            cfg.noise_enabled = (cfg.pos_noise_std_xy > 0.0);
        } else if (arg == "--yaw-noise" && i + 1 < argc) {
            cfg.yaw_noise_std = std::stod(argv[++i]);
        } else if (arg == "--drift-x" && i + 1 < argc) {
            cfg.drift_vel_x = std::stod(argv[++i]);
        } else if (arg == "--drift-y" && i + 1 < argc) {
            cfg.drift_vel_y = std::stod(argv[++i]);
        } else if (arg == "--tau" && i + 1 < argc) {
            cfg.tau_xy = std::stod(argv[++i]);
            cfg.tau_z = cfg.tau_xy * 1.33;
        } else if (arg == "--seed" && i + 1 < argc) {
            cfg.random_seed = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--dt" && i + 1 < argc) {
            dt = std::stod(argv[++i]);
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_file = argv[++i];
        } else if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
    }

    std::cout << "========================================================\n"
              << "         AUTOPILOT CLOSED-LOOP SIMULATOR                \n"
              << "========================================================\n"
              << " Scenario:           " << scenario << "\n"
              << " Follower Lag (tau): " << cfg.tau_xy << " s\n"
              << " Position Noise std: " << cfg.pos_noise_std_xy << " m\n"
              << " Yaw Noise std:      " << cfg.yaw_noise_std << " rad\n"
              << " Wind Drift (X,Y):   [" << cfg.drift_vel_x << ", " << cfg.drift_vel_y << "] m/s\n"
              << " Random Seed:        " << cfg.random_seed << "\n"
              << " Timestep (dt):      " << dt << " s\n"
              << "--------------------------------------------------------\n";

    sim::AutopilotSimHarness harness(cfg);

    if (scenario == "takeoff") {
        harness.setupMission({SHAPE_CIRCLE}, MISSION_END_HOVER);
        harness.addSimulatedGoal(SHAPE_CIRCLE, 0.0, 0.0, 1.0, 0.0);
    } else if (scenario == "single_gate") {
        harness.setupMission({SHAPE_SQUARE}, MISSION_END_HOVER);
        harness.addSimulatedGoal(SHAPE_SQUARE, 2.0, 0.5, 1.0, 0.0);
    } else if (scenario == "drift") {
        cfg.drift_vel_x = 0.20; // 0.2 m/s lateral wind
        cfg.drift_vel_y = -0.15;
        harness.getFollower().setConfig(cfg);
        harness.setupMission({SHAPE_CIRCLE}, MISSION_END_HOVER);
        harness.addSimulatedGoal(SHAPE_CIRCLE, 2.0, 0.0, 1.0, 0.0);
    } else { // "multi_gate"
        harness.setupMission({SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE}, MISSION_END_LAND);
        harness.addSimulatedGoal(SHAPE_CIRCLE, 1.5, -0.5, 1.0, 0.0);
        harness.addSimulatedGoal(SHAPE_TRIANGLE, 3.0, 0.0, 1.2, 0.1);
        harness.addSimulatedGoal(SHAPE_SQUARE, 4.5, 0.5, 1.0, 0.0);
    }

    std::cout << "[SIM] Running scenario...\n";
    sim::MissionReport report = harness.runUntil(45.0, dt);

    std::cout << "--------------------------------------------------------\n"
              << " RESULTS: " << report.summary << "\n"
              << " Trajectory Points:     " << harness.getTrajectory().size() << "\n"
              << " Total Distance:        " << report.total_distance_m << " m\n"
              << " Max Tracking Error:    " << report.max_tracking_error << " m\n"
              << " Tracking RMSE:         " << report.rmse_tracking << " m\n"
              << " Status:                " << (report.mission_completed ? "[SUCCESS / PASS]" : "[INCOMPLETE / FAIL]") << "\n"
              << "========================================================\n";

    if (!csv_file.empty()) {
        std::ofstream csv(csv_file);
        if (csv.is_open()) {
            csv << "time,true_x,true_y,true_z,true_yaw,noisy_x,noisy_y,noisy_z,cmd_x,cmd_y,cmd_z,cmd_yaw,state\n";
            for (const auto& pt : harness.getTrajectory()) {
                csv << pt.timestamp << ","
                    << pt.true_pose.x << "," << pt.true_pose.y << "," << pt.true_pose.z << "," << pt.true_pose.yaw << ","
                    << pt.feedback_pose.x << "," << pt.feedback_pose.y << "," << pt.feedback_pose.z << ","
                    << pt.command.target_x << "," << pt.command.target_y << "," << pt.command.target_z << "," << pt.command.target_yaw << ","
                    << static_cast<int>(pt.flight_state) << "\n";
            }
            std::cout << "[SIM] Exported trajectory to " << csv_file << std::endl;
        }
    }

    return report.mission_completed ? 0 : 1;
}
