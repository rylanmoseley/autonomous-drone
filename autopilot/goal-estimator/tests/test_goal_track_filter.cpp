#include <gtest/gtest.h>
#include <cmath>
#include "goal_track_filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST(GoalTrackFilterTest, InitializationAndYawExtraction) {
    PoseHypothesis hyp;
    hyp.position_cam = {1.0f, 2.0f, 3.0f};
    // Yaw = 90 deg = pi/2: w = cos(pi/4) = 0.7071, z = sin(pi/4) = 0.7071
    hyp.orientation_cam = {0.7071068f, 0.0f, 0.0f, 0.7071068f};
    hyp.confidence = 0.95f;

    TrackedGoal tg = GoalTrackFilter::initializeTrackedGoal(SHAPE_CIRCLE, hyp, 5.0);
    EXPECT_EQ(tg.shape_type, SHAPE_CIRCLE);
    EXPECT_EQ(tg.state, TRACK_STATE_CONVERGING);
    EXPECT_NEAR(tg.pose.x, 1.0, 1e-5);
    EXPECT_NEAR(tg.pose.y, 2.0, 1e-5);
    EXPECT_NEAR(tg.pose.z, 3.0, 1e-5);
    EXPECT_NEAR(tg.pose.yaw, M_PI / 2.0, 1e-3);
    EXPECT_DOUBLE_EQ(tg.last_update_time, 5.0);
}

TEST(GoalTrackFilterTest, OutlierRejectionAndRelocation) {
    GoalTrackFilter filter(0.5);
    PoseHypothesis hyp;
    hyp.position_cam = {2.0f, 0.0f, 1.0f};
    hyp.orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    hyp.confidence = 0.95f;

    TrackedGoal tg = GoalTrackFilter::initializeTrackedGoal(SHAPE_CIRCLE, hyp, 1.0);
    // Move to locked
    bool accepted = filter.updateGoalTrack(tg, hyp, 1.1, false);
    EXPECT_TRUE(accepted);
    EXPECT_EQ(tg.state, TRACK_STATE_LOCKED);
    EXPECT_NEAR(tg.pose.x, 2.0, 1e-4);

    // Frame 3: Single-frame spatial glitch (jump > 0.35m to X = 3.5m)
    hyp.position_cam.x = 3.5f;
    accepted = filter.updateGoalTrack(tg, hyp, 1.2, false);
    EXPECT_FALSE(accepted);
    EXPECT_NEAR(tg.pose.x, 2.0, 1e-4);
    EXPECT_EQ(filter.getOutlierJumpCount(SHAPE_CIRCLE), 1);

    // Frame 4: Second anomalous frame
    accepted = filter.updateGoalTrack(tg, hyp, 1.3, false);
    EXPECT_FALSE(accepted);
    EXPECT_NEAR(tg.pose.x, 2.0, 1e-4);
    EXPECT_EQ(filter.getOutlierJumpCount(SHAPE_CIRCLE), 2);

    // Frame 5: Third consecutive anomalous frame (confirms intentional relocation)
    accepted = filter.updateGoalTrack(tg, hyp, 1.4, false);
    EXPECT_TRUE(accepted);
    EXPECT_NEAR(tg.pose.x, 3.5, 1e-4);
    EXPECT_EQ(filter.getOutlierJumpCount(SHAPE_CIRCLE), 0);
}

TEST(GoalTrackFilterTest, ConfidenceDamping) {
    GoalTrackFilter filter(0.5);
    PoseHypothesis hyp;
    hyp.position_cam = {2.0f, 0.0f, 1.0f};
    hyp.orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    hyp.confidence = 0.95f;

    TrackedGoal tg = GoalTrackFilter::initializeTrackedGoal(SHAPE_TRIANGLE, hyp, 1.0);
    filter.updateGoalTrack(tg, hyp, 1.1, false);

    // Feed target with low confidence (< 0.85)
    hyp.position_cam.x = 2.2f;
    hyp.confidence = 0.60f;
    bool accepted = filter.updateGoalTrack(tg, hyp, 1.2, false);
    EXPECT_TRUE(accepted);
    // With damped alpha = 0.5 * 0.25 = 0.125:
    // Expected x = 0.125 * 2.2 + 0.875 * 2.0 = 2.025m
    EXPECT_NEAR(tg.pose.x, 2.025, 1e-3);
}

TEST(GoalTrackFilterTest, PlanarYawSymmetry) {
    GoalTrackFilter filter(0.5);
    PoseHypothesis hyp;
    hyp.position_cam = {2.0f, 0.0f, 1.0f};
    hyp.orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    hyp.confidence = 0.95f;

    TrackedGoal tg = GoalTrackFilter::initializeTrackedGoal(SHAPE_SQUARE, hyp, 1.0);
    filter.updateGoalTrack(tg, hyp, 1.1, false);

    // Invert normal vector by ~180 degrees (yaw = pi - 0.05 rad)
    double flip_yaw = M_PI - 0.05;
    hyp.orientation_cam = {
        static_cast<float>(std::cos(flip_yaw * 0.5)),
        0.0f, 0.0f,
        static_cast<float>(std::sin(flip_yaw * 0.5))
    };
    filter.updateGoalTrack(tg, hyp, 1.2, false);

    // Symmetry projection should map it close to 0.0, NOT flip to 180 deg
    EXPECT_NEAR(tg.pose.yaw, -0.025, 0.05);
}
