#include <gtest/gtest.h>
#include "sim_camera_perception.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST(SimCameraPerceptionTest, NominalGateProjection) {
    sim::SimCameraPerception perception;
    Pose true_p{};
    true_p.x = 0.0;
    true_p.y = 0.0;
    true_p.z = 1.0;
    true_p.yaw = 0.0; // Facing along +X

    std::vector<sim::SimulatedGoal> goals = {
        {SHAPE_CIRCLE, 3.0, 0.0, 1.0, 0.0}
    };
    std::vector<sim::SimulatedTag> tags;

    VisionPacket pkt = perception.generateVisionPacket(true_p, goals, tags, 1, 0.1);
    EXPECT_EQ(pkt.num_goals_reported, 1);
    EXPECT_EQ(pkt.goals[0].is_detected, 1);
    EXPECT_EQ(pkt.goals[0].shape_type, SHAPE_CIRCLE);

    // Drone is at (0,0,1), gate is at (3,0,1).
    // Camera is mounted forward 0.18m, up 0.04m -> cam is at (0.18, 0, 1.04).
    // dx = 3.0 - 0.18 = 2.82m forward.
    // Cam Z is forward (+Z_cam), Cam X is right (+X_cam), Cam Y is down (+Y_cam).
    // 320x240 image pixel discretization tolerance (+-0.20m at 2.82m range)
    EXPECT_NEAR(pkt.goals[0].hypotheses[0].position_cam.z, 2.82, 0.20); // fwd
    EXPECT_NEAR(pkt.goals[0].hypotheses[0].position_cam.x, 0.0, 0.05);  // lat
}

TEST(SimCameraPerceptionTest, FrustumFovFiltering) {
    sim::SimPerceptionConfig cfg;
    cfg.fov_filtering_enabled = true;
    cfg.h_fov_deg = 70.0; // +-35 deg
    cfg.v_fov_deg = 50.0;
    cfg.min_range_m = 0.15;
    cfg.max_range_m = 6.0;

    sim::SimCameraPerception perception(cfg);
    Pose true_p{};
    true_p.x = 0.0;
    true_p.y = 0.0;
    true_p.z = 1.0;
    true_p.yaw = 0.0;

    // Gate at 2.0m forward, but 3.0m to the right (angle = atan2(3, 2) = 56 deg > 35 deg)
    std::vector<sim::SimulatedGoal> goals = {
        {SHAPE_TRIANGLE, 2.0, 3.0, 1.0, 0.0}
    };
    std::vector<sim::SimulatedTag> tags;

    VisionPacket pkt = perception.generateVisionPacket(true_p, goals, tags, 2, 0.2);
    EXPECT_EQ(pkt.num_goals_reported, 1);
    EXPECT_EQ(pkt.goals[0].is_detected, 0); // Out of FOV
}

TEST(SimCameraPerceptionTest, OcclusionFiltering) {
    sim::SimPerceptionConfig cfg;
    cfg.fov_filtering_enabled = true;
    cfg.occlusion_check_enabled = true;

    sim::SimCameraPerception perception(cfg);
    Pose true_p{};
    true_p.x = 0.0;
    true_p.y = 0.0;
    true_p.z = 1.0;
    true_p.yaw = 0.0;

    // Two gates lined up directly along the optical axis: one at 2.0m, second at 4.0m
    std::vector<sim::SimulatedGoal> goals = {
        {SHAPE_CIRCLE, 2.0, 0.0, 1.0, 0.0},
        {SHAPE_SQUARE, 4.0, 0.0, 1.0, 0.0}
    };
    std::vector<sim::SimulatedTag> tags;

    VisionPacket pkt = perception.generateVisionPacket(true_p, goals, tags, 3, 0.3);
    EXPECT_EQ(pkt.num_goals_reported, 2);
    EXPECT_EQ(pkt.goals[0].is_detected, 1); // Front gate visible
    EXPECT_EQ(pkt.goals[1].is_detected, 0); // Back gate occluded by front gate
}

TEST(SimCameraPerceptionTest, TagMeasurementDetection) {
    sim::SimCameraPerception perception;
    Pose true_p{};
    true_p.x = 1.0;
    true_p.y = 1.0;
    true_p.z = 1.0;
    true_p.yaw = 0.0;

    std::vector<sim::SimulatedGoal> goals;
    std::vector<sim::SimulatedTag> tags = {
        {101, 1.5, 1.0, 1.0, 0.0, 0.0, 0.0, 0.98f}, // Within 3.5m (dist 0.5m)
        {102, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0, 0.95f} // Far away (dist ~12m)
    };

    VisionPacket pkt = perception.generateVisionPacket(true_p, goals, tags, 4, 0.4);
    EXPECT_EQ(pkt.num_tags_detected, 1);
    EXPECT_EQ(pkt.tags[0].tag_id, 101);
    EXPECT_EQ(pkt.tags[0].is_valid, 1);
    EXPECT_FLOAT_EQ(pkt.tags[0].camera_pos_world.x, 1.0f);
}
