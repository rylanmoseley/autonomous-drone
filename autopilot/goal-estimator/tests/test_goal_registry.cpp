#include <gtest/gtest.h>
#include "goal_registry.h"

TEST(GoalRegistryTest, StoreAndRetrieve) {
    GoalRegistry registry(2.0);

    TrackedGoal tg;
    tg.shape_type = SHAPE_CIRCLE;
    tg.state = TRACK_STATE_LOCKED;
    tg.pose.x = 1.0;
    tg.pose.y = 2.0;
    tg.pose.z = 3.0;
    tg.last_update_time = 10.0;

    registry.setTrackedGoal(SHAPE_CIRCLE, tg);

    TrackedGoal retrieved;
    EXPECT_TRUE(registry.getTrackedGoal(SHAPE_CIRCLE, retrieved));
    EXPECT_EQ(retrieved.shape_type, SHAPE_CIRCLE);
    EXPECT_NEAR(retrieved.pose.x, 1.0, 1e-5);

    EXPECT_TRUE(registry.getLastKnownGoal(SHAPE_CIRCLE, retrieved));
    EXPECT_NEAR(retrieved.pose.y, 2.0, 1e-5);
}

TEST(GoalRegistryTest, TimeoutAndActiveEstimates) {
    GoalRegistry registry(2.0);

    TrackedGoal tg;
    tg.shape_type = SHAPE_TRIANGLE;
    tg.state = TRACK_STATE_LOCKED;
    tg.last_update_time = 10.0;
    registry.setTrackedGoal(SHAPE_TRIANGLE, tg);

    // Within timeout (10.0 + 1.5 = 11.5 <= 12.0)
    auto active = registry.getActiveEstimates(11.5);
    EXPECT_EQ(active.size(), 1u);

    // After timeout (10.0 + 2.5 = 12.5 > 12.0)
    active = registry.getActiveEstimates(12.5);
    EXPECT_EQ(active.size(), 0u);

    // But last known goal remains!
    TrackedGoal last_known;
    EXPECT_TRUE(registry.getLastKnownGoal(SHAPE_TRIANGLE, last_known));
}

TEST(GoalRegistryTest, ClearAndReset) {
    GoalRegistry registry(2.0);

    TrackedGoal tg1, tg2;
    tg1.shape_type = SHAPE_CIRCLE;
    tg1.last_update_time = 1.0;
    tg2.shape_type = SHAPE_SQUARE;
    tg2.last_update_time = 1.0;

    registry.setTrackedGoal(SHAPE_CIRCLE, tg1);
    registry.setTrackedGoal(SHAPE_SQUARE, tg2);
    EXPECT_EQ(registry.getAllKnownGoals().size(), 2u);

    registry.clearGoal(SHAPE_CIRCLE);
    EXPECT_EQ(registry.getAllKnownGoals().size(), 1u);
    TrackedGoal temp;
    EXPECT_FALSE(registry.getLastKnownGoal(SHAPE_CIRCLE, temp));
    EXPECT_TRUE(registry.getLastKnownGoal(SHAPE_SQUARE, temp));

    registry.reset();
    EXPECT_EQ(registry.getAllKnownGoals().size(), 0u);
}
