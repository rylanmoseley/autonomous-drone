#pragma once
#include "flight_control_interface.h"

class ApproachCorridorPlanner {
public:
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

    StandoffPlan planStandoffTarget(
        const Pose& current_pose,
        const Pose& raw_target,
        double approach_dx,
        double approach_dy,
        double standoff_dist = 0.75,
        double lateral_clearance = 0.85) const;
};
