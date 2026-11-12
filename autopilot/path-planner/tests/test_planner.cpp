#include <gtest/gtest.h>
#include "path_planner.h"
#include <cmath>

TEST(PathPlannerTest, HasArrived) {
    PlannerConfig config;
    config.arrival_tolerance = 0.5;
    PathPlanner planner(config);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    Pose target = {0.4, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    EXPECT_TRUE(planner.has_arrived(current, target));
    
    target.x = 0.6;
    EXPECT_FALSE(planner.has_arrived(current, target));
}

TEST(PathPlannerTest, VelocityLimiting) {
    PlannerConfig config = {};
    config.max_velocity = 1.0;
    config.max_vertical_velocity = 0.25;
    config.max_acceleration = 100.0; // Instantly reach target velocity for the test
    config.position_p_gain = 2.0;
    config.max_yaw_rate = 1.0;
    config.yaw_p_gain = 1.0;
    config.arrival_tolerance = 0.1;
    PathPlanner planner(config);
    
    Pose current = {0.0, 0.0, 0.0, 0, 0, 0, 0, 1.0};
    Pose target = {10.0, 0.0, 10.0, 0, 0, 0, 0, 1.0};
    
    double dt = 0.1;
    FlightCommand cmd = planner.plan(current, target, dt);
    
    EXPECT_NEAR(cmd.target_x, 0.1, 1e-6);
    EXPECT_NEAR(cmd.target_y, 0.0, 1e-6);
    EXPECT_NEAR(cmd.target_z, 0.025, 1e-6);
}

// 3. Plan Approach Direction for Sequential In-Line Gates
TEST(PathPlannerTest, PlanApproachDirectionSequentialGates) {
    PlannerConfig config{};
    PathPlanner planner(config);

    // Gate 1: traversed along +X (approach dx=1, dy=0)
    Pose gate1 = {2.0, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};
    // Gate 2: sequential along +X at X=4.5, Y=0.0
    Pose gate2 = {4.5, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};

    auto dir = planner.planApproachDirection(gate2, gate1, 1.0, 0.0, true);
    EXPECT_DOUBLE_EQ(dir.sign, 1.0);
    EXPECT_NEAR(dir.dx, 1.0, 1e-5);
    EXPECT_NEAR(dir.dy, 0.0, 1e-5);
}

// 4. Plan Approach Direction for Side-By-Side Gates (Reverse Traversal)
TEST(PathPlannerTest, PlanApproachDirectionSideBySideGates) {
    PlannerConfig config{};
    PathPlanner planner(config);

    // Gate 1 at (2.5, -0.6, 1.0), traversed forward (+X)
    Pose gate1 = {2.5, -0.6, 1.0, 0, 0, 0.0, 0, 1.0};
    // Gate 2 side-by-side at (2.5, +0.6, 1.0)
    Pose gate2 = {2.5, +0.6, 1.0, 0, 0, 0.0, 0, 1.0};

    auto dir = planner.planApproachDirection(gate2, gate1, 1.0, 0.0, true);
    // Side-by-side reverse traversal: approach sign should be -1.0 (backwards through Gate 2)
    EXPECT_DOUBLE_EQ(dir.sign, -1.0);
    EXPECT_NEAR(dir.dx, -1.0, 1e-5);
    EXPECT_NEAR(dir.dy, 0.0, 1e-5);
}

// 5. Corridor Normal Symmetry Alignment
TEST(PathPlannerTest, AlignApproachWithCorridor) {
    PlannerConfig config{};
    PathPlanner planner(config);

    // Gate normal reported as -X (yaw = pi), but drone has established approach along +X
    Pose gate = {3.0, 0.0, 1.0, 0, 0, M_PI, 0, 1.0};
    double dx = 1.0, dy = 0.0;
    planner.alignApproachWithCorridor(gate, dx, dy);

    // Half-plane projection should flip normal to align with established corridor (+X)
    EXPECT_NEAR(dx, 1.0, 1e-5);
    EXPECT_NEAR(dy, 0.0, 1e-5);
}

// 6. Standoff Target Calculation Without Obstruction
TEST(PathPlannerTest, PlanStandoffTargetWithoutObstruction) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {3.0, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};
    Pose drone = {1.0, 0.0, 1.0, 0, 0, 0.0, 0, 1.0}; // Safely in front of the gate

    auto plan = planner.planStandoffTarget(drone, gate, 1.0, 0.0, 0.75, 0.85);

    EXPECT_FALSE(plan.is_avoidance_active);
    EXPECT_NEAR(plan.target_pose.x, 2.25, 1e-5); // 3.0 - 0.75
    EXPECT_NEAR(plan.target_pose.y, 0.0, 1e-5);
    EXPECT_NEAR(plan.target_pose.z, 1.0, 1e-5);
    EXPECT_LT(plan.dot_fwd, 0.0);
}

// 7. Standoff Target Calculation With Gate-Frame Avoidance
TEST(PathPlannerTest, PlanStandoffTargetGateFrameAvoidance) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {2.5, 0.0, 1.0, 0, 0, 0.0, 0, 1.0}; // Gate at (2.5, 0.0) facing +X
    // Drone is behind the gate at (3.5, 0.1), attempting to fly to entrance standoff (1.75, 0.0)
    Pose drone = {3.5, 0.1, 1.0, 0, 0, 0.0, 0, 1.0};

    auto plan = planner.planStandoffTarget(drone, gate, 1.0, 0.0, 0.75, 0.85);

    EXPECT_TRUE(plan.is_avoidance_active);
    // Avoidance waypoint must maintain full 0.75m standoff and lateral clearance 0.85m
    EXPECT_NEAR(plan.target_pose.x, 1.75, 1e-5);
    EXPECT_GE(std::abs(plan.target_pose.y), 0.80);
}

// 8. Obstacle Repulsion (APF)
TEST(PathPlannerTest, ComputeObstacleRepulsion) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose drone = {1.5, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};
    // Non-target obstacle located at (1.8, 0.0, 1.0) directly ahead in the flight path
    std::vector<PathPlanner::Obstacle> obstacles = {{1.8, 0.0, 1.0, 1}};

    double apf_vx = 0.0, apf_vy = 0.0, off_x = 0.0, off_y = 0.0;
    planner.computeObstacleRepulsion(drone, obstacles, 1.0, 0.0, apf_vx, apf_vy, off_x, off_y, 0.55, 0.70);

    // Obstacle is 0.3m ahead; should generate significant lateral deflection
    double tot_offset = std::hypot(off_x, off_y);
    EXPECT_GT(tot_offset, 0.10);
    EXPECT_LE(tot_offset, 1.20);
}

// 9. Traversal Target & Punch Pose Generation
TEST(PathPlannerTest, TraversalTargetAndPunchPose) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {2.0, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};
    Pose drone = {2.1, 0.0, 1.0, 0, 0, 0.0, 0, 1.0}; // Traversing through gate

    Pose step_target = planner.planTraversalTarget(drone, gate, 1.0, 0.0, 0.35, 0.55);
    EXPECT_NEAR(step_target.x, 2.45, 1e-5); // dot_fwd=0.1 + 0.35 = 0.45m -> x = 2.45

    Pose punch_pose = planner.getPunchThroughPose(gate, 1.0, 0.0, 0.55);
    EXPECT_NEAR(punch_pose.x, 2.55, 1e-5); // 2.0 + 0.55
}

// 10. Phase Arrival & Alignment Evaluators
TEST(PathPlannerTest, PhaseTransitionEvaluators) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {2.5, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};
    Pose entrance_standoff = {1.75, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};

    // Standoff arrived: drone at (1.76, 0.01, 1.0)
    Pose drone_standoff = {1.76, 0.01, 1.0, 0, 0, 0.0, 0, 1.0};
    EXPECT_TRUE(planner.isStandoffReached(drone_standoff, entrance_standoff, entrance_standoff, 1.0, 0.0, -0.74, 10));

    // Boresight aligned
    EXPECT_TRUE(planner.isBoresightAligned(drone_standoff, gate, entrance_standoff, 1.0, 0.0, 10));

    // Traversal completed: drone at (3.05, 0.0, 1.0)
    Pose drone_exited = {3.05, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};
    Pose punch_target = {3.05, 0.0, 1.0, 0, 0, 0.0, 0, 1.0};
    EXPECT_TRUE(planner.isTraversalCompleted(drone_exited, gate, punch_target, 1.0, 0.0, 10));
}

// 11. Orthogonal Path Generation - Nominal Gate
TEST(PathPlannerTest, GenerateOrthogonalPathNominal) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {2.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}; // Yaw=0, pitch=0
    Pose drone_ref = {1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    auto path = planner.generateOrthogonalPath(gate, drone_ref, {}, 0.75);

    // Entry point: (1.75, 0.0, 1.0)
    EXPECT_NEAR(path.entry_point.x, 1.75, 1e-4);
    EXPECT_NEAR(path.entry_point.y, 0.0, 1e-4);
    EXPECT_NEAR(path.entry_point.z, 1.0, 1e-4);

    // Center point (widest point of goal): (2.5, 0.0, 1.0)
    EXPECT_NEAR(path.gate_center.x, 2.5, 1e-4);
    EXPECT_NEAR(path.gate_center.y, 0.0, 1e-4);
    EXPECT_NEAR(path.gate_center.z, 1.0, 1e-4);

    // Exit point: (3.25, 0.0, 1.0)
    EXPECT_NEAR(path.exit_point.x, 3.25, 1e-4);
    EXPECT_NEAR(path.exit_point.y, 0.0, 1e-4);
    EXPECT_NEAR(path.exit_point.z, 1.0, 1e-4);

    // Unit normal vector and path dimensions
    EXPECT_NEAR(path.normal_x, 1.0, 1e-4);
    EXPECT_NEAR(path.normal_y, 0.0, 1e-4);
    EXPECT_NEAR(path.normal_z, 0.0, 1e-4);
    EXPECT_NEAR(path.standoff_dist, 0.75, 1e-4);
    EXPECT_NEAR(path.total_length, 1.50, 1e-4);
}

// 12. Orthogonal Path Generation - Arbitrary 3D Orientations (Yaw=0.7, -1.6, Pitched)
TEST(PathPlannerTest, GenerateOrthogonalPathYawedAndPitched) {
    PlannerConfig config{};
    PathPlanner planner(config);

    // 1. Gate at yaw = 0.7 rad
    Pose gate_yaw07 = {2.5, 0.0, 1.0, 0.0, 0.0, 0.7, 0.0, 1.0};
    Pose drone_ref = {1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    auto path07 = planner.generateOrthogonalPath(gate_yaw07, drone_ref, {}, 0.75);

    double exp_nx07 = std::cos(0.7);
    double exp_ny07 = std::sin(0.7);
    EXPECT_NEAR(path07.normal_x, exp_nx07, 1e-4);
    EXPECT_NEAR(path07.normal_y, exp_ny07, 1e-4);
    EXPECT_NEAR(path07.normal_z, 0.0, 1e-4);
    // Center is widest point of gate
    EXPECT_NEAR(path07.gate_center.x, 2.5, 1e-4);
    EXPECT_NEAR(path07.gate_center.y, 0.0, 1e-4);

    // 2. Gate at yaw = -1.6 rad
    Pose gate_yaw_neg16 = {2.5, 1.3, 1.0, 0.0, 0.0, -1.6, 0.0, 1.0};
    auto path16 = planner.generateOrthogonalPath(gate_yaw_neg16, drone_ref, {}, 0.75);
    double norm16 = std::hypot(path16.normal_x, path16.normal_y);
    EXPECT_NEAR(norm16, 1.0, 1e-4);

    // 3. Pitched Gate (pitch = 0.20 rad, yaw = 0.0)
    Pose gate_pitched = {2.0, 0.0, 1.0, 0.0, 0.20, 0.0, 0.0, 1.0};
    auto path_pitch = planner.generateOrthogonalPath(gate_pitched, drone_ref, {}, 0.75);
    double n3d_len = std::hypot(std::hypot(path_pitch.normal_x, path_pitch.normal_y), path_pitch.normal_z);
    EXPECT_NEAR(n3d_len, 1.0, 1e-4);
    EXPECT_NEAR(path_pitch.normal_x, std::cos(0.20), 1e-4);
    EXPECT_NEAR(path_pitch.normal_z, -std::sin(0.20), 1e-4);
}

// 13. Closest End Selection
TEST(PathPlannerTest, GenerateOrthogonalPathClosestEndSelection) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {2.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    // Case A: Drone is at X = 1.0 (closer to End 1 at X = 1.75 than End 2 at X = 3.25)
    Pose drone_near_end1 = {1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    auto path_a = planner.generateOrthogonalPath(gate, drone_near_end1, {}, 0.75);
    EXPECT_NEAR(path_a.entry_point.x, 1.75, 1e-4);
    EXPECT_NEAR(path_a.exit_point.x, 3.25, 1e-4);
    EXPECT_NEAR(path_a.normal_x, 1.0, 1e-4);

    // Case B: Drone is at X = 3.6 (closer to End 2 at X = 3.25 than End 1 at X = 1.75)
    Pose drone_near_end2 = {3.6, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    auto path_b = planner.generateOrthogonalPath(gate, drone_near_end2, {}, 0.75);
    EXPECT_NEAR(path_b.entry_point.x, 3.25, 1e-4);
    EXPECT_NEAR(path_b.exit_point.x, 1.75, 1e-4);
    EXPECT_NEAR(path_b.normal_x, -1.0, 1e-4);
}

// 14. Exclusion Area Intersection Avoidance
TEST(PathPlannerTest, GenerateOrthogonalPathExclusionAreaAvoidance) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {2.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    // Drone is at (1.0, 1.2, 1.0) (closer to End 1 at (1.75, 0.0) than End 2 at (3.25, 0.0))
    Pose drone = {1.0, 1.2, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    // Obstacle is placed directly blocking End 1 at (1.75, 0.0, 1.0)
    std::vector<PathPlanner::Obstacle> obstacles = {{1.75, 0.0, 1.0, 1}};

    // Because End 1 is inside the obstacle exclusion radius, planner must choose End 2!
    auto path = planner.generateOrthogonalPath(gate, drone, obstacles, 0.75);
    EXPECT_NEAR(path.entry_point.x, 3.25, 1e-4);
    EXPECT_NEAR(path.exit_point.x, 1.75, 1e-4);
    EXPECT_NEAR(path.normal_x, -1.0, 1e-4);
}

// 15. Precise Orthogonal Line Tracking Through Widest Point
TEST(PathPlannerTest, OrthogonalPathTraversalTracking) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {2.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose drone = {1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    auto path = planner.generateOrthogonalPath(gate, drone, {}, 0.75);

    // Drone at entry point (1.75, 0.0, 1.0)
    Pose pos_entry = {1.75, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose target_step1 = planner.planOrthogonalTraversalTarget(pos_entry, path, 0.35);
    EXPECT_NEAR(target_step1.x, 1.75 + 0.35, 1e-4);
    EXPECT_NEAR(target_step1.y, 0.0, 1e-4);
    EXPECT_NEAR(target_step1.z, 1.0, 1e-4);

    // Drone passing through gate center (widest point at X = 2.5)
    Pose pos_center = {2.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose target_center = planner.planOrthogonalTraversalTarget(pos_center, path, 0.35);
    EXPECT_NEAR(target_center.x, 2.85, 1e-4);
    EXPECT_NEAR(target_center.y, 0.0, 1e-4);
    EXPECT_NEAR(target_center.z, 1.0, 1e-4);

    // Drone with lateral error at (2.2, 0.15, 1.08)
    Pose pos_perturbed = {2.2, 0.15, 1.08, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose target_perturbed = planner.planOrthogonalTraversalTarget(pos_perturbed, path, 0.35);
    // Commanded target must be strictly on the orthogonal line (Y=0, Z=1.0)
    EXPECT_NEAR(target_perturbed.y, 0.0, 1e-4);
    EXPECT_NEAR(target_perturbed.z, 1.0, 1e-4);
    EXPECT_GT(target_perturbed.x, 2.2);
}

// 16. Traversal Completion Evaluation
TEST(PathPlannerTest, OrthogonalPathTraversalCompletion) {
    PlannerConfig config{};
    PathPlanner planner(config);

    Pose gate = {2.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose drone = {1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    auto path = planner.generateOrthogonalPath(gate, drone, {}, 0.75);

    // Still in front of gate
    Pose drone_before = {2.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    EXPECT_FALSE(planner.isOrthogonalTraversalCompleted(drone_before, path, 10));

    // At gate center
    Pose drone_center = {2.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    EXPECT_FALSE(planner.isOrthogonalTraversalCompleted(drone_center, path, 10));

    // Reached exit standoff point (3.25, 0.0, 1.0)
    Pose drone_exit = {3.22, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    EXPECT_TRUE(planner.isOrthogonalTraversalCompleted(drone_exit, path, 10));
}
