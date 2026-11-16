#pragma once
#include "flight_control_interface.h"

class PhaseAlignmentEvaluator {
public:
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
        int traverse_ticks,
        double arrival_tolerance = 0.15) const;

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
};
