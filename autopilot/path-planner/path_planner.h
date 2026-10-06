#pragma once
#include "flight_control_interface.h"

struct PlannerConfig {
    double max_velocity; // m/s
    double max_vertical_velocity; // m/s
    double max_acceleration; // m/s^2
    double max_yaw_rate; // rad/s
    double position_p_gain;
    double yaw_p_gain;
    double arrival_tolerance; // m
};

#include <mutex>

class PathPlanner {
public:
    PathPlanner(const PlannerConfig& config);
    FlightCommand plan(const Pose& current_pose, const Pose& target_pose, double dt, double apf_vx = 0.0, double apf_vy = 0.0);
    bool has_arrived(const Pose& current_pose, const Pose& target_pose) const;
    PlannerConfig getConfig() const {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        return config_;
    }
    void updateConfig(const PlannerConfig& new_cfg) {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        config_ = new_cfg;
    }

private:
    PlannerConfig config_;
    mutable std::mutex cfg_mutex_;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vz_ = 0.0;
    double clamp(double value, double min_val, double max_val) const;
};
