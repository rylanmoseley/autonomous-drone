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

class PathPlanner {
public:
    PathPlanner(const PlannerConfig& config);
    FlightCommand plan(const Pose& current_pose, const Pose& target_pose, double dt);
    bool has_arrived(const Pose& current_pose, const Pose& target_pose) const;
    PlannerConfig& getConfig() { return config_; }

private:
    PlannerConfig config_;
    double clamp(double value, double min_val, double max_val) const;
};
