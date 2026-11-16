#pragma once
#include "kinematic_profile_filter.h"
#include "approach_corridor_planner.h"
#include "orthogonal_path_generator.h"
#include "obstacle_avoidance_potential_field.h"
#include "phase_alignment_evaluator.h"
#include <vector>

class PathPlanner {
public:
    using Obstacle = ObstacleAvoidancePotentialField::Obstacle;
    using ApproachDirection = ApproachCorridorPlanner::ApproachDirection;
    using StandoffPlan = ApproachCorridorPlanner::StandoffPlan;
    using OrthogonalPath = OrthogonalPathGenerator::OrthogonalPath;

    explicit PathPlanner(const PlannerConfig& config);

    // Sub-component Accessors for targeted development and inspection
    KinematicProfileFilter& getKinematicFilter() { return kinematic_filter_; }
    const KinematicProfileFilter& getKinematicFilter() const { return kinematic_filter_; }

    ApproachCorridorPlanner& getCorridorPlanner() { return corridor_planner_; }
    const ApproachCorridorPlanner& getCorridorPlanner() const { return corridor_planner_; }

    OrthogonalPathGenerator& getOrthogonalGenerator() { return orthogonal_generator_; }
    const OrthogonalPathGenerator& getOrthogonalGenerator() const { return orthogonal_generator_; }

    ObstacleAvoidancePotentialField& getAvoidanceField() { return avoidance_field_; }
    const ObstacleAvoidancePotentialField& getAvoidanceField() const { return avoidance_field_; }

    PhaseAlignmentEvaluator& getPhaseEvaluator() { return phase_evaluator_; }
    const PhaseAlignmentEvaluator& getPhaseEvaluator() const { return phase_evaluator_; }

    // --- Trajectory / Kinematic Setpoint Filter ---
    FlightCommand plan(const Pose& current_pose, const Pose& target_pose, double dt, double apf_vx = 0.0, double apf_vy = 0.0) {
        return kinematic_filter_.planStep(current_pose, target_pose, dt, apf_vx, apf_vy);
    }

    bool has_arrived(const Pose& current_pose, const Pose& target_pose) const {
        return kinematic_filter_.hasArrived(current_pose, target_pose);
    }

    // --- Approach Geometry & Corridor Establishment ---
    ApproachDirection planApproachDirection(
        const Pose& target_gate,
        const Pose& last_completed_gate,
        double last_approach_dx,
        double last_approach_dy,
        bool is_subsequent_goal) const {
        return corridor_planner_.planApproachDirection(target_gate, last_completed_gate, last_approach_dx, last_approach_dy, is_subsequent_goal);
    }

    void alignApproachWithCorridor(
        const Pose& target_gate,
        double& inout_dx,
        double& inout_dy) const {
        corridor_planner_.alignApproachWithCorridor(target_gate, inout_dx, inout_dy);
    }

    // --- Gate Standoff & Clearance Waypoint Planning ---
    StandoffPlan planStandoffTarget(
        const Pose& current_pose,
        const Pose& raw_target,
        double approach_dx,
        double approach_dy,
        double standoff_dist = 0.75,
        double lateral_clearance = 0.85) const {
        return corridor_planner_.planStandoffTarget(current_pose, raw_target, approach_dx, approach_dy, standoff_dist, lateral_clearance);
    }

    // --- Traversal Target Planning ---
    Pose planTraversalTarget(
        const Pose& current_pose,
        const Pose& raw_target,
        double approach_dx,
        double approach_dy,
        double lookahead = 0.35,
        double max_punch = 0.55) const {
        return phase_evaluator_.planTraversalTarget(current_pose, raw_target, approach_dx, approach_dy, lookahead, max_punch);
    }

    Pose getPunchThroughPose(
        const Pose& raw_target,
        double approach_dx,
        double approach_dy,
        double punch_dist = 0.55) const {
        return phase_evaluator_.getPunchThroughPose(raw_target, approach_dx, approach_dy, punch_dist);
    }

    // --- Orthogonal Goal Traversal Path Generation & Tracking ---
    OrthogonalPath generateOrthogonalPath(
        const Pose& goal_pose,
        const Pose& reference_pose,
        const std::vector<Obstacle>& obstacles = {},
        double standoff_dist = 0.75,
        double preferred_sign = 0.0) const {
        return orthogonal_generator_.generateOrthogonalPath(goal_pose, reference_pose, obstacles, standoff_dist, preferred_sign);
    }

    bool checkSegmentExclusionIntersection(
        const Pose& p1,
        const Pose& p2,
        const std::vector<Obstacle>& obstacles,
        double exclusion_radius = 0.55) const {
        return avoidance_field_.checkSegmentExclusionIntersection(p1, p2, obstacles, exclusion_radius);
    }

    Pose planOrthogonalTraversalTarget(
        const Pose& current_pose,
        const OrthogonalPath& path,
        double lookahead_dist = 0.35) const {
        return orthogonal_generator_.planOrthogonalTraversalTarget(current_pose, path, lookahead_dist);
    }

    bool isOrthogonalTraversalCompleted(
        const Pose& current_pose,
        const OrthogonalPath& path,
        int traverse_ticks) const {
        return orthogonal_generator_.isOrthogonalTraversalCompleted(current_pose, path, traverse_ticks);
    }

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
        double r_influence = 0.70) const {
        avoidance_field_.computeObstacleRepulsion(current_pose, obstacles, des_vx, des_vy, out_apf_vx, out_apf_vy, out_offset_x, out_offset_y, r_nogo, r_influence);
    }

    // --- Phase Arrival & Alignment Evaluators ---
    bool isStandoffReached(
        const Pose& current_pose,
        const Pose& entrance_standoff,
        const Pose& adjusted_target,
        double approach_dx,
        double approach_dy,
        double dot_fwd,
        int standoff_ticks) const {
        return phase_evaluator_.isStandoffReached(current_pose, entrance_standoff, adjusted_target, approach_dx, approach_dy, dot_fwd, standoff_ticks);
    }

    bool isBoresightAligned(
        const Pose& current_pose,
        const Pose& raw_target,
        const Pose& adjusted_target,
        double approach_dx,
        double approach_dy,
        int align_ticks,
        double approach_dz = 0.0) const {
        return phase_evaluator_.isBoresightAligned(current_pose, raw_target, adjusted_target, approach_dx, approach_dy, align_ticks, approach_dz);
    }

    bool isTraversalCompleted(
        const Pose& current_pose,
        const Pose& raw_target,
        const Pose& traverse_target,
        double approach_dx,
        double approach_dy,
        int traverse_ticks) const {
        return phase_evaluator_.isTraversalCompleted(current_pose, raw_target, traverse_target, approach_dx, approach_dy, traverse_ticks, kinematic_filter_.getConfig().arrival_tolerance);
    }

    PlannerConfig getConfig() const {
        return kinematic_filter_.getConfig();
    }
    void updateConfig(const PlannerConfig& new_cfg) {
        kinematic_filter_.updateConfig(new_cfg);
    }
    void reset() {
        kinematic_filter_.reset();
    }

private:
    KinematicProfileFilter kinematic_filter_;
    ApproachCorridorPlanner corridor_planner_;
    OrthogonalPathGenerator orthogonal_generator_;
    ObstacleAvoidancePotentialField avoidance_field_;
    PhaseAlignmentEvaluator phase_evaluator_;
};
