#pragma once
#include "flight_control_interface.h"
#include <vector>

class ObstacleAvoidancePotentialField {
public:
    struct Obstacle {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        uint8_t shape_type{0};
    };

    bool checkSegmentExclusionIntersection(
        const Pose& p1,
        const Pose& p2,
        const std::vector<Obstacle>& obstacles,
        double exclusion_radius = 0.55) const;

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
        double r_influence = 0.70) const;
};
