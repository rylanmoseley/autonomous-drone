#include <gtest/gtest.h>
#include "orthogonal_path_generator.h"
#include <cmath>

TEST(OrthogonalPathGeneratorTest, TraversalNormalFrom3DOrientation) {
    OrthogonalPathGenerator gen;

    // Gate facing positive X (yaw = 0, pitch = 0)
    Pose goal_x = {2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose ref = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}; // Approaching from -X

    auto path_x = gen.generateOrthogonalPath(goal_x, ref, {}, 0.75);
    EXPECT_NEAR(path_x.normal_x, 1.0, 1e-4);
    EXPECT_NEAR(path_x.normal_y, 0.0, 1e-4);
    EXPECT_NEAR(path_x.normal_z, 0.0, 1e-4);
    EXPECT_NEAR(path_x.entry_point.x, 1.25, 1e-4);
    EXPECT_NEAR(path_x.exit_point.x, 2.75, 1e-4);
    EXPECT_DOUBLE_EQ(path_x.traversal_sign, 1.0);

    // Gate pitched downwards by 30 deg (pitch = -M_PI/6)
    Pose goal_pitched = {2.0, 0.0, 1.0, 0.0, -M_PI / 6.0, 0.0, 0.0, 1.0};
    auto path_p = gen.generateOrthogonalPath(goal_pitched, ref, {}, 0.75);
    EXPECT_GT(path_p.normal_z, 0.1); // nz = -sp > 0
}

TEST(OrthogonalPathGeneratorTest, ClosestEndSelectionAndSignLocking) {
    OrthogonalPathGenerator gen;
    Pose goal = {2.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose ref_behind = {3.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}; // Approaching from behind (+X)

    // With no obstacles, closest end is end2 (at x = 3.25)
    auto path = gen.generateOrthogonalPath(goal, ref_behind, {}, 0.75);
    EXPECT_NEAR(path.entry_point.x, 3.25, 1e-4);
    EXPECT_NEAR(path.exit_point.x, 1.75, 1e-4);
    EXPECT_DOUBLE_EQ(path.traversal_sign, -1.0);

    // If preferred_sign is locked to +1.0, it must adhere to end1
    auto path_locked = gen.generateOrthogonalPath(goal, ref_behind, {}, 0.75, 1.0);
    EXPECT_NEAR(path_locked.entry_point.x, 1.75, 1e-4);
    EXPECT_NEAR(path_locked.exit_point.x, 3.25, 1e-4);
    EXPECT_DOUBLE_EQ(path_locked.traversal_sign, 1.0);
}

TEST(OrthogonalPathGeneratorTest, TraversalLineLookaheadProjection) {
    OrthogonalPathGenerator gen;
    Pose goal = {2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose ref = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    auto path = gen.generateOrthogonalPath(goal, ref, {}, 0.75);

    // Drone at entry point (1.25, 0.0, 1.0)
    Pose drone = {1.25, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose target = gen.planOrthogonalTraversalTarget(drone, path, 0.35);

    EXPECT_NEAR(target.x, 1.25 + 0.35, 1e-4);
    EXPECT_NEAR(target.y, 0.0, 1e-4);
    EXPECT_NEAR(target.z, 1.0, 1e-4);

    // Completion check: when drone passes exit point
    Pose drone_exit = {2.75, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    EXPECT_TRUE(gen.isOrthogonalTraversalCompleted(drone_exit, path, 10));
}
