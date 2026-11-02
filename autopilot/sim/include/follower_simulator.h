#pragma once

#include "flight_control_interface.h"
#include "vision_packet.h"
#include "mavlink_protocol.h"
#include <random>
#include <cmath>
#include <vector>
#include <string>

namespace sim {

struct FollowerConfig {
    // Kinematic tracking response dynamics (first-order lag time constants)
    double tau_xy = 0.15;           // Response lag in XY plane (seconds)
    double tau_z  = 0.20;           // Response lag in Z axis (seconds)
    double tau_yaw = 0.10;          // Response lag in Yaw (seconds)

    // Velocity & Acceleration saturation limits
    double max_velocity_xy = 2.5;   // Max horizontal speed (m/s)
    double max_velocity_z  = 1.5;   // Max vertical speed (m/s)
    double max_acceleration = 8.0;  // Max linear acceleration (m/s^2)
    double max_yaw_rate    = 2.0;   // Max yaw angular rate (rad/s)

    // Imputed noise configuration
    bool noise_enabled = true;
    double pos_noise_std_xy = 0.03; // Gaussian standard deviation on X/Y feedback (meters)
    double pos_noise_std_z  = 0.015;// Gaussian standard deviation on Z feedback (meters)
    double yaw_noise_std    = 0.02; // Gaussian standard deviation on Yaw feedback (radians)

    // Environmental disturbances / drift (e.g. constant wind vector)
    double drift_vel_x = 0.0;       // Constant external drift in X (m/s)
    double drift_vel_y = 0.0;       // Constant external drift in Y (m/s)
    double drift_vel_z = 0.0;       // Constant external drift in Z (m/s)

    // Random walk sensor bias standard deviation (meters/sqrt(s))
    double random_walk_drift_std = 0.0;

    // PRNG seed for deterministic test reproducibility (0 = seed from clock)
    uint32_t random_seed = 42;
};

struct SimulationMetrics {
    double total_sim_time{0.0};
    double total_distance_traveled{0.0};
    double max_tracking_error{0.0};
    double sum_squared_error{0.0};
    double rmse_position{0.0};
    size_t sample_count{0};
    bool arrived_at_target{false};
    double time_to_arrival{0.0};

    // Global drift handling metrics
    size_t tag_corrections_count{0};
    double total_drift_corrected_m{0.0};
    double max_drift_error{0.0};

    void recordSample(double tracking_error, double dt, bool arrived);
    void reset();
};

class FcuFollowerSimulator {
public:
    explicit FcuFollowerSimulator(const FollowerConfig& config = FollowerConfig());

    void reset(const Pose& initial_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    // Advance the follower simulation by dt seconds given the current autopilot FlightCommand
    void step(const FlightCommand& cmd, double dt);

    // Apply global vision position estimate (from AprilTag/ChArUco pass-through) to arrest FCU drift
    void correctDriftWithVision(const TagMeasurement& tag);
    void correctDriftWithVisionEstimate(const MavlinkVisionPositionEstimate& est);

    // True physical pose of the drone (ground truth)
    const Pose& getTruePose() const { return true_pose_; }

    // Feedback pose (true pose corrupted by imputed noise and bias) reported to autopilot
    const Pose& getFeedbackPose() const { return feedback_pose_; }

    // Accumulated dead reckoning drift (distance between feedback pose and true pose)
    double getDriftMagnitude() const {
        double dx = feedback_pose_.x - true_pose_.x;
        double dy = feedback_pose_.y - true_pose_.y;
        double dz = feedback_pose_.z - true_pose_.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    double getBiasX() const { return bias_x_; }
    double getBiasY() const { return bias_y_; }
    double getBiasZ() const { return bias_z_; }

    // Pack feedback pose into standard Telemetry datagram for autopilot / MAVLink
    Telemetry getTelemetry(int battery_level = 95, bool error_state = false) const;

    // Simulation metrics & statistics
    const SimulationMetrics& getMetrics() const { return metrics_; }

    // Configuration accessors
    const FollowerConfig& getConfig() const { return config_; }
    void setConfig(const FollowerConfig& config);

private:
    FollowerConfig config_;
    Pose true_pose_{};
    Pose feedback_pose_{};
    double vx_{0.0};
    double vy_{0.0};
    double vz_{0.0};
    double bias_x_{0.0};
    double bias_y_{0.0};
    double bias_z_{0.0};

    SimulationMetrics metrics_{};

    std::mt19937 prng_;
    std::normal_distribution<double> normal_dist_{0.0, 1.0};

    static double clamp(double value, double min_val, double max_val);
    static double wrapAnglePi(double angle_rad);
};

} // namespace sim
