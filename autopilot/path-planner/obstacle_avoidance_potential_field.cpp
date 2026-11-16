#include "obstacle_avoidance_potential_field.h"
#include <cmath>
#include <algorithm>

bool ObstacleAvoidancePotentialField::checkSegmentExclusionIntersection(
    const Pose& p1,
    const Pose& p2,
    const std::vector<Obstacle>& obstacles,
    double exclusion_radius) const {

    double seg_dx = p2.x - p1.x;
    double seg_dy = p2.y - p1.y;
    double seg_dz = p2.z - p1.z;
    double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

    for (const auto& obs : obstacles) {
        double d_p2 = std::hypot(p2.x - obs.x, p2.y - obs.y);
        if (p2.z <= obs.z + 0.35 && d_p2 < exclusion_radius) {
            return true;
        }

        if (seg_len_sq > 1e-4) {
            double t = ((obs.x - p1.x) * seg_dx + (obs.y - p1.y) * seg_dy) / seg_len_sq;
            if (t > 0.05 && t <= 1.0) {
                double proj_x = p1.x + t * seg_dx;
                double proj_y = p1.y + t * seg_dy;
                double proj_z = p1.z + t * seg_dz;
                if (proj_z <= obs.z + 0.35) {
                    double dist_to_seg = std::hypot(proj_x - obs.x, proj_y - obs.y);
                    if (dist_to_seg < exclusion_radius) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void ObstacleAvoidancePotentialField::computeObstacleRepulsion(
    const Pose& current_pose,
    const std::vector<Obstacle>& obstacles,
    double des_vx,
    double des_vy,
    double& out_apf_vx,
    double& out_apf_vy,
    double& out_offset_x,
    double& out_offset_y,
    double r_nogo,
    double r_influence) const {

    out_apf_vx = 0.0;
    out_apf_vy = 0.0;
    out_offset_x = 0.0;
    out_offset_y = 0.0;

    for (const auto& obs : obstacles) {
        if (current_pose.z > obs.z + 0.40) continue;

        double dx = current_pose.x - obs.x;
        double dy = current_pose.y - obs.y;
        double dist_2d = std::hypot(dx, dy);

        if (dist_2d >= r_influence) continue;

        if (dist_2d < 0.01) {
            dx = 1.0; dy = 0.0; dist_2d = 1.0;
        }

        double pen = (r_influence - dist_2d) / (r_influence - r_nogo);
        pen = std::max(0.0, std::min(2.5, pen));

        double ux = dx / dist_2d;
        double uy = dy / dist_2d;

        double cross = dx * des_vy - dy * des_vx;
        double side = (std::abs(cross) > 1e-3) ? ((cross >= 0.0) ? 1.0 : -1.0) : ((dy >= 0.0) ? 1.0 : -1.0);

        double vx = -side * uy;
        double vy = side * ux;

        double dir_x = ux * 0.70 + vx * 0.70;
        double dir_y = uy * 0.70 + vy * 0.70;
        double dir_norm = std::hypot(dir_x, dir_y);
        if (dir_norm > 1e-4) {
            dir_x /= dir_norm;
            dir_y /= dir_norm;
        }

        out_apf_vx += dir_x * pen * 2.5;
        out_apf_vy += dir_y * pen * 2.5;
        out_offset_x += dir_x * pen * 0.45;
        out_offset_y += dir_y * pen * 0.45;
    }

    double tot_off = std::hypot(out_offset_x, out_offset_y);
    if (tot_off > 1.2) {
        out_offset_x = (out_offset_x / tot_off) * 1.2;
        out_offset_y = (out_offset_y / tot_off) * 1.2;
    }
}
