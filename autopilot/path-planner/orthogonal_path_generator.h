#pragma once
#include "flight_control_interface.h"
#include "obstacle_avoidance_potential_field.h"
#include <vector>

class OrthogonalPathGenerator {
public:
    using Obstacle = ObstacleAvoidancePotentialField::Obstacle;

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

    OrthogonalPath generateOrthogonalPath(
        const Pose& goal_pose,
        const Pose& reference_pose,
        const std::vector<Obstacle>& obstacles = {},
        double standoff_dist = 0.75,
        double preferred_sign = 0.0) const;

    Pose planOrthogonalTraversalTarget(
        const Pose& current_pose,
        const OrthogonalPath& path,
        double lookahead_dist = 0.35) const;

    bool isOrthogonalTraversalCompleted(
        const Pose& current_pose,
        const OrthogonalPath& path,
        int traverse_ticks) const;

private:
    ObstacleAvoidancePotentialField avoidance_;
};
