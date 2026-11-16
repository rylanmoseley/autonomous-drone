#pragma once
#include "flight_control_interface.h"
#include <mutex>

struct PlannerConfig {
    double max_velocity{2.0}; // m/s
    double max_vertical_velocity{1.0}; // m/s
    double max_acceleration{4.0}; // m/s^2
    double max_yaw_rate{1.5}; // rad/s
    double position_p_gain{1.2};
    double yaw_p_gain{1.5};
    double arrival_tolerance{0.15}; // m
};

class KinematicProfileFilter {
public:
    explicit KinematicProfileFilter(const PlannerConfig& config);

    FlightCommand planStep(
        const Pose& current_pose,
        const Pose& target_pose,
        double dt,
        double apf_vx = 0.0,
        double apf_vy = 0.0);

    bool hasArrived(const Pose& current_pose, const Pose& target_pose) const;

    PlannerConfig getConfig() const;
    void updateConfig(const PlannerConfig& new_cfg);
    void reset();

    double getVx() const { return vx_; }
    double getVy() const { return vy_; }
    double getVz() const { return vz_; }

private:
    PlannerConfig config_;
    mutable std::mutex cfg_mutex_;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vz_ = 0.0;

    double clamp(double value, double min_val, double max_val) const;
};
