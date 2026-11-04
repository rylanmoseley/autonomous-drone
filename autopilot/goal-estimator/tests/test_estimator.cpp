#include <gtest/gtest.h>
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
