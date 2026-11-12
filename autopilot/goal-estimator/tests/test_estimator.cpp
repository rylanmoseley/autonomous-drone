#include <gtest/gtest.h>
#include <cmath>
#include "goal_estimator.h"

TEST(GoalEstimatorTest, UpdateAndTimeout) {
    GoalEstimator estimator(0.5, 2.0);
    
    VisionPacket pkt = {};
    pkt.msg_type = MSG_TYPE_VISION;
    pkt.num_goals_reported = 1;
    pkt.goals[0].shape_type = SHAPE_CIRCLE;
    pkt.goals[0].is_detected = 1;
    pkt.goals[0].num_hypotheses = 1;
    pkt.goals[0].hypotheses[0].position_cam = {1.0f, 2.0f, 3.0f};
    pkt.goals[0].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    pkt.goals[0].hypotheses[0].confidence = 1.0f;
    
    estimator.updateEstimates(pkt, 10.0);
    
    auto current = estimator.getCurrentEstimates(10.5);
    ASSERT_EQ(current.size(), 1);
    EXPECT_EQ(current[0].shape_type, SHAPE_CIRCLE);
    EXPECT_NEAR(current[0].pose.x, 1.0, 1e-5);
    EXPECT_NEAR(current[0].pose.y, 2.0, 1e-5);
    EXPECT_NEAR(current[0].pose.z, 3.0, 1e-5);
    
    pkt.goals[0].hypotheses[0].position_cam.x = 3.0f;
    estimator.updateEstimates(pkt, 11.0);
    
    current = estimator.getCurrentEstimates(11.5);
    ASSERT_EQ(current.size(), 1);
    EXPECT_NEAR(current[0].pose.x, 2.0, 1e-5); 
    
    current = estimator.getCurrentEstimates(14.0);
    EXPECT_EQ(current.size(), 0);
}

// 2. Spatial Outlier Glitch Filter Rejection & Relocation Confirmation
TEST(GoalEstimatorTest, SpatialOutlierGlitchFilter) {
    GoalEstimator estimator(0.5, 2.0);

    VisionPacket pkt = {};
    pkt.msg_type = MSG_TYPE_VISION;
    pkt.num_goals_reported = 1;
    pkt.goals[0].shape_type = SHAPE_CIRCLE;
    pkt.goals[0].is_detected = 1;
    pkt.goals[0].num_hypotheses = 1;
    pkt.goals[0].hypotheses[0].position_cam = {2.0f, 0.0f, 1.0f};
    pkt.goals[0].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    pkt.goals[0].hypotheses[0].confidence = 0.95f;

    // Frame 1: Establish track
    estimator.updateEstimates(pkt, 1.0);
    // Frame 2: Lock track
    estimator.updateEstimates(pkt, 1.1);

    auto est = estimator.getCurrentEstimates(1.1);
    ASSERT_EQ(est.size(), 1u);
    EXPECT_EQ(est[0].state, TRACK_STATE_LOCKED);
    EXPECT_NEAR(est[0].pose.x, 2.0, 1e-4);

    // Frame 3: Single-frame spatial glitch (jump > 0.35m to X = 3.5m)
    pkt.goals[0].hypotheses[0].position_cam.x = 3.5f;
    estimator.updateEstimates(pkt, 1.2);

    // Glitch must be rejected! Position remains at 2.0m
    est = estimator.getCurrentEstimates(1.2);
    ASSERT_EQ(est.size(), 1u);
    EXPECT_NEAR(est[0].pose.x, 2.0, 1e-4);

    // Frame 4: Second anomalous frame
    estimator.updateEstimates(pkt, 1.3);
    est = estimator.getCurrentEstimates(1.3);
    EXPECT_NEAR(est[0].pose.x, 2.0, 1e-4);

    // Frame 5: Third consecutive anomalous frame (confirms intentional relocation)
    estimator.updateEstimates(pkt, 1.4);
    est = estimator.getCurrentEstimates(1.4);
    // Adopts new relocated position
    EXPECT_NEAR(est[0].pose.x, 3.5, 1e-4);
}

// 3. Confidence Damped Updating for Marginal Contours
TEST(GoalEstimatorTest, ConfidenceDampedUpdating) {
    GoalEstimator estimator(0.5, 2.0);

    VisionPacket pkt = {};
    pkt.msg_type = MSG_TYPE_VISION;
    pkt.num_goals_reported = 1;
    pkt.goals[0].shape_type = SHAPE_TRIANGLE;
    pkt.goals[0].is_detected = 1;
    pkt.goals[0].num_hypotheses = 1;
    pkt.goals[0].hypotheses[0].position_cam = {2.0f, 0.0f, 1.0f};
    pkt.goals[0].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    pkt.goals[0].hypotheses[0].confidence = 0.95f;

    estimator.updateEstimates(pkt, 1.0);
    estimator.updateEstimates(pkt, 1.1); // Locked state

    // Feed a slightly moved target with low confidence (< 0.85)
    // Within jump threshold (0.2m < 0.35m), but confidence is 0.60
    pkt.goals[0].hypotheses[0].position_cam.x = 2.2f;
    pkt.goals[0].hypotheses[0].confidence = 0.60f;
    estimator.updateEstimates(pkt, 1.2);

    // With damped alpha = 0.5 * 0.25 = 0.125:
    // Expected x = 0.125 * 2.2 + 0.875 * 2.0 = 0.275 + 1.75 = 2.025m
    auto est = estimator.getCurrentEstimates(1.2);
    ASSERT_EQ(est.size(), 1u);
    EXPECT_NEAR(est[0].pose.x, 2.025, 1e-3);
}

// 4. Planar Yaw Symmetry Projector (+-180 deg equivalence)
TEST(GoalEstimatorTest, PlanarYawSymmetry) {
    GoalEstimator estimator(0.5, 2.0);

    VisionPacket pkt = {};
    pkt.msg_type = MSG_TYPE_VISION;
    pkt.num_goals_reported = 1;
    pkt.goals[0].shape_type = SHAPE_SQUARE;
    pkt.goals[0].is_detected = 1;
    pkt.goals[0].num_hypotheses = 1;
    pkt.goals[0].hypotheses[0].position_cam = {2.0f, 0.0f, 1.0f};
    pkt.goals[0].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f}; // yaw = 0.0
    pkt.goals[0].hypotheses[0].confidence = 0.95f;

    estimator.updateEstimates(pkt, 1.0);
    estimator.updateEstimates(pkt, 1.1); // Locked state

    // Invert normal vector by 180 degrees (yaw = pi - 0.05 rad ≈ 3.09 rad)
    double flip_yaw = M_PI - 0.05;
    Quatf q_flip = {
        static_cast<float>(std::cos(flip_yaw * 0.5)),
        0.0f, 0.0f,
        static_cast<float>(std::sin(flip_yaw * 0.5))
    };
    pkt.goals[0].hypotheses[0].orientation_cam = q_flip;
    estimator.updateEstimates(pkt, 1.2);

    auto est = estimator.getCurrentEstimates(1.2);
    ASSERT_EQ(est.size(), 1u);
    // Symmetry projection should map it close to 0.0, NOT flip to 180 deg!
    EXPECT_NEAR(est[0].pose.yaw, -0.025, 0.05);
}

// 5. Telemetry Occluded State and Memory Retention
TEST(GoalEstimatorTest, OccludedTelemetryState) {
    GoalEstimator estimator(0.5, 1.0); // 1-second timeout

    VisionPacket pkt = {};
    pkt.msg_type = MSG_TYPE_VISION;
    pkt.num_goals_reported = 1;
    pkt.goals[0].shape_type = SHAPE_CIRCLE;
    pkt.goals[0].is_detected = 1;
    pkt.goals[0].num_hypotheses = 1;
    pkt.goals[0].hypotheses[0].position_cam = {3.0f, 1.5f, 1.0f};
    pkt.goals[0].hypotheses[0].orientation_cam = {1.0f, 0.0f, 0.0f, 0.0f};
    pkt.goals[0].hypotheses[0].confidence = 0.95f;

    estimator.updateEstimates(pkt, 10.0);

    // Active tracking
    auto telem = estimator.getTelemetry(10.5);
    ASSERT_EQ(telem.size(), 3u); // Telemetry reports circle, triangle, square slots
    EXPECT_EQ(telem[0].state, TRACK_STATE_CONVERGING);

    // Timeout occurs (> 1.0s elapsed)
    telem = estimator.getTelemetry(12.0);
    EXPECT_EQ(telem[0].state, TRACK_STATE_OCCLUDED);
    EXPECT_NEAR(telem[0].world_position.x, 3.0f, 1e-4);
    EXPECT_NEAR(telem[0].world_position.y, 1.5f, 1e-4);
    EXPECT_NEAR(telem[0].spatial_variance, 0.05f, 1e-4);

    // Triangle was never detected
    EXPECT_EQ(telem[1].state, TRACK_STATE_SEARCHING);
}

// 6. Lifecycle Management: Clear and Reset
TEST(GoalEstimatorTest, LifecycleApis) {
    GoalEstimator estimator(0.5, 2.0);

    VisionPacket pkt = {};
    pkt.msg_type = MSG_TYPE_VISION;
    pkt.num_goals_reported = 2;
    pkt.goals[0].shape_type = SHAPE_CIRCLE;
    pkt.goals[0].is_detected = 1;
    pkt.goals[0].num_hypotheses = 1;
    pkt.goals[0].hypotheses[0].position_cam = {1.0f, 0.0f, 1.0f};
    pkt.goals[0].hypotheses[0].confidence = 0.9f;

    pkt.goals[1].shape_type = SHAPE_TRIANGLE;
    pkt.goals[1].is_detected = 1;
    pkt.goals[1].num_hypotheses = 1;
    pkt.goals[1].hypotheses[0].position_cam = {2.0f, 0.0f, 1.0f};
    pkt.goals[1].hypotheses[0].confidence = 0.9f;

    estimator.updateEstimates(pkt, 1.0);
    EXPECT_EQ(estimator.getCurrentEstimates(1.5).size(), 2u);

    // Clear circle
    estimator.clearGoal(SHAPE_CIRCLE);
    EXPECT_EQ(estimator.getCurrentEstimates(1.5).size(), 1u);
    TrackedGoal tg;
    EXPECT_FALSE(estimator.getLastKnownEstimate(SHAPE_CIRCLE, tg));
    EXPECT_TRUE(estimator.getLastKnownEstimate(SHAPE_TRIANGLE, tg));

    // Full reset
    estimator.reset();
    EXPECT_EQ(estimator.getCurrentEstimates(1.5).size(), 0u);
    EXPECT_EQ(estimator.getAllKnownGoals().size(), 0u);
}
