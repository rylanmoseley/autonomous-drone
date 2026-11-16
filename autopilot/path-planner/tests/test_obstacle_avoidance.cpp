#include <gtest/gtest.h>
#include "obstacle_avoidance_potential_field.h"
#include <cmath>

TEST(ObstacleAvoidanceTest, SegmentExclusionIntersection) {
    ObstacleAvoidancePotentialField apf;

    std::vector<ObstacleAvoidancePotentialField::Obstacle> obstacles = {
        {2.0, 0.0, 1.0, 1} // Obstacle at (2.0, 0.0, 1.0)
    };

    // Segment passing directly through obstacle from (1.0, 0.0, 1.0) to (3.0, 0.0, 1.0)
    Pose p1 = {1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose p2 = {3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    EXPECT_TRUE(apf.checkSegmentExclusionIntersection(p1, p2, obstacles, 0.55));

    // Clear segment at Y = 1.0
    Pose p_clear1 = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Pose p_clear2 = {3.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    EXPECT_FALSE(apf.checkSegmentExclusionIntersection(p_clear1, p_clear2, obstacles, 0.55));
}

TEST(ObstacleAvoidanceTest, RepulsionForcesAndOffsets) {
    ObstacleAvoidancePotentialField apf;
    std::vector<ObstacleAvoidancePotentialField::Obstacle> obstacles = {
        {2.0, 0.0, 1.0, 1}
    };

    // Drone at (1.5, 0.0, 1.0), heading +X towards obstacle
    Pose current = {1.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    double apf_vx = 0.0, apf_vy = 0.0, off_x = 0.0, off_y = 0.0;

    apf.computeObstacleRepulsion(current, obstacles, 1.0, 0.0, apf_vx, apf_vy, off_x, off_y, 0.45, 0.95);

    // Repulsion should push away from obstacle (apf_vx < 0 or lateral deflection)
    EXPECT_GT(std::hypot(apf_vx, apf_vy), 0.1);
    EXPECT_GT(std::hypot(off_x, off_y), 0.05);
}
