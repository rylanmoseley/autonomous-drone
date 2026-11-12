#pragma once
#include "flight_control_interface.h"
#include <mutex>
#include <vector>

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
    struct Obstacle {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        uint8_t shape_type{0};
    };

    struct ApproachDirection {
        double dx{1.0};
        double dy{0.0};
        double dz{0.0};
        double sign{1.0};
    };

    struct StandoffPlan {
        Pose target_pose{};
        Pose entrance_standoff{};
        double dot_fwd{0.0};
        double lat{0.0};
        bool is_avoidance_active{false};
    };

    struct OrthogonalPath {
        Pose entry_point{};       // Starting standoff point (D distance from center)
        Pose exit_point{};        // Ending standoff point (D distance on opposite side)
        Pose gate_center{};       // Center of the goal (widest point)
        double normal_x{1.0};     // Traversal unit vector (entry -> center -> exit)
        double normal_y{0.0};
        double normal_z{0.0};
        double standoff_dist{0.75};
        double total_length{1.50};
        double traversal_sign{1.0};
    };

    explicit PathPlanner(const PlannerConfig& config);

    // --- Trajectory / Kinematic Setpoint Filter ---
    FlightCommand plan(const Pose& current_pose, const Pose& target_pose, double dt, double apf_vx = 0.0, double apf_vy = 0.0);
    bool has_arrived(const Pose& current_pose, const Pose& target_pose) const;

    // --- Approach Geometry & Corridor Establishment ---
    ApproachDirection planApproachDirection(
        const Pose& target_gate,
        const Pose& last_completed_gate,
        double last_approach_dx,
        double last_approach_dy,
        bool is_subsequent_goal) const;

    void alignApproachWithCorridor(
        const Pose& target_gate,
        double& inout_dx,
        double& inout_dy) const;

    // --- Gate Standoff & Clearance Waypoint Planning ---
    StandoffPlan planStandoffTarget(
        const Pose& current_pose,
        const Pose& raw_target,
        double approach_dx,
        double approach_dy,
        double standoff_dist = 0.75,
        double lateral_clearance = 0.85) const;

    // --- Traversal Target Planning ---
    Pose planTraversalTarget(
        const Pose& current_pose,
        const Pose& raw_target,
        double approach_dx,
        double approach_dy,
        double lookahead = 0.35,
        double max_punch = 0.55) const;

    Pose getPunchThroughPose(
        const Pose& raw_target,
        double approach_dx,
        double approach_dy,
        double punch_dist = 0.55) const;

    // --- Orthogonal Goal Traversal Path Generation & Tracking ---
    OrthogonalPath generateOrthogonalPath(
        const Pose& goal_pose,
        const Pose& reference_pose,
        const std::vector<Obstacle>& obstacles = {},
        double standoff_dist = 0.75,
        double preferred_sign = 0.0) const;

    bool checkSegmentExclusionIntersection(
        const Pose& p1,
        const Pose& p2,
        const std::vector<Obstacle>& obstacles,
        double exclusion_radius = 0.55) const;

    Pose planOrthogonalTraversalTarget(
        const Pose& current_pose,
        const OrthogonalPath& path,
        double lookahead_dist = 0.35) const;

    bool isOrthogonalTraversalCompleted(
        const Pose& current_pose,
        const OrthogonalPath& path,
        int traverse_ticks) const;

    // --- Obstacle Avoidance (APF) ---
    void computeObstacleRepulsion(
        const Pose& current_pose,
        const std::vector<Obstacle>& obstacles,
        double des_vx,
        double des_vy,
        double& out_apf_vx,
        double& out_apf_vy,
        double& out_offset_x,
        double& out_offset_y,
        double r_nogo = 0.55,
        double r_influence = 0.70) const;

    // --- Phase Arrival & Alignment Evaluators ---
    bool isStandoffReached(
        const Pose& current_pose,
        const Pose& entrance_standoff,
        const Pose& adjusted_target,
        double approach_dx,
        double approach_dy,
        double dot_fwd,
        int standoff_ticks) const;

    bool isBoresightAligned(
        const Pose& current_pose,
        const Pose& raw_target,
        const Pose& adjusted_target,
        double approach_dx,
        double approach_dy,
        int align_ticks) const;

    bool isTraversalCompleted(
        const Pose& current_pose,
        const Pose& raw_target,
        const Pose& traverse_target,
        double approach_dx,
        double approach_dy,
        int traverse_ticks) const;

    PlannerConfig getConfig() const {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        return config_;
    }
    void updateConfig(const PlannerConfig& new_cfg) {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        config_ = new_cfg;
    }
    void reset() {
        vx_ = 0.0;
        vy_ = 0.0;
        vz_ = 0.0;
    }

private:
    PlannerConfig config_;
    mutable std::mutex cfg_mutex_;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vz_ = 0.0;
    double clamp(double value, double min_val, double max_val) const;
};
