#include <gtest/gtest.h>
#include "goal_estimator.h"

TEST(GoalEstimatorTest, UpdateAndTimeout) {
    GoalEstimator estimator(0.5, 2.0);
    
    std::vector<VisionGoalEstimate> updates;
    VisionGoalEstimate e1;
    e1.goal_id = 1;
    e1.pose = {1.0, 2.0, 3.0, 0, 0, 0, 0, 1.0};
    e1.determinate = true;
    updates.push_back(e1);
    
    estimator.updateEstimates(updates, 10.0);
    
    auto current = estimator.getCurrentEstimates(10.5);
    ASSERT_EQ(current.size(), 1);
    EXPECT_EQ(current[0].goal_id, 1);
    EXPECT_DOUBLE_EQ(current[0].pose.x, 1.0);
    
    e1.pose.x = 3.0;
    updates[0] = e1;
    estimator.updateEstimates(updates, 11.0);
    
    current = estimator.getCurrentEstimates(11.5);
    ASSERT_EQ(current.size(), 1);
    EXPECT_DOUBLE_EQ(current[0].pose.x, 2.0); 
    
    current = estimator.getCurrentEstimates(14.0);
    EXPECT_EQ(current.size(), 0);
}
