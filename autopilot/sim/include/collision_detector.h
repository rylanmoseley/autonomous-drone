#pragma once

#include "common_types.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace sim {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct SimulatedGoalDef {
    uint8_t shape_type; // SHAPE_CIRCLE (1), SHAPE_TRIANGLE (2), SHAPE_SQUARE (3)
    double x;           // Position in World NED (meters)
    double y;
    double z;
    double yaw;         // Heading (radians)
};

struct CollisionCheckResult {
    bool is_hit{false};
    std::string hit_message;
    double min_clearance_m{1e9};
    uint8_t shape_hit{0};
};

class CollisionDetector {
public:
    // Distance from point (pu, pw) to 2D line segment between (ax, ay) and (bx, by)
    static inline double distToSegment2D(double pu, double pw, double ax, double ay, double bx, double by) {
        double vx = bx - ax;
        double vy = by - ay;
        double l2 = vx * vx + vy * vy;
        if (l2 < 1e-8) {
            return std::hypot(pu - ax, pw - ay);
        }
        double t = std::max(0.0, std::min(1.0, ((pu - ax) * vx + (pw - ay) * vy) / l2));
        return std::hypot(pu - (ax + t * vx), pw - (ay + t * vy));
    }

    // Check collision and calculate obstacle clearance for a drone at (px, py, pz, pyaw)
    template <typename GoalContainer>
    static CollisionCheckResult check(double px, double py, double pz, double pyaw,
                                      const GoalContainer& goals,
                                      double drone_radius = 0.06) {
        (void)pyaw;
        const double tube_radius = 0.03;
        const double collision_dist = tube_radius + drone_radius; // 0.09m

        CollisionCheckResult result{};
        result.min_clearance_m = 1e9;

        for (const auto& g : goals) {
            double rx = px - g.x;
            double ry = py - g.y;
            double rz = pz - g.z;

            double gnx = std::cos(g.yaw);
            double gny = std::sin(g.yaw);
            double h = rx * gnx + ry * gny;          // axial distance along hoop normal
            double u = -rx * gny + ry * gnx;         // horizontal coordinate in hoop plane
            double w = rz;                           // vertical coordinate in hoop plane

            uint8_t shape = g.shape_type;

            // 1. Circle (shape 1): Torus with major radius R = 0.22m, tube r = 0.03m
            if (shape == SHAPE_CIRCLE) {
                double rho = std::hypot(u, w);
                double d_2d = std::abs(rho - 0.22);
                double d_tube = std::hypot(d_2d, h);
                double clearance = d_tube - collision_dist;
                if (clearance < result.min_clearance_m) {
                    result.min_clearance_m = clearance;
                }
                if (d_tube < collision_dist) {
                    result.is_hit = true;
                    result.shape_hit = shape;
                    result.hit_message = "Clipped Circle gate hoop rim";
                }
                // Vertical mounting pylon extending below hoop base: w < -0.25m
                if (w < -0.25 && pz >= 0.0) {
                    double dist_pylon = std::hypot(rx, ry);
                    double pylon_clearance = dist_pylon - (0.015 + drone_radius);
                    if (pylon_clearance < result.min_clearance_m) {
                        result.min_clearance_m = pylon_clearance;
                    }
                    if (dist_pylon < (0.015 + drone_radius)) {
                        result.is_hit = true;
                        result.shape_hit = shape;
                        result.hit_message = "Collided with Circle gate pylon";
                    }
                }
            }
            // 2. Triangle (shape 2): Equilateral triangle, circumradius R = 0.35m, apex UP
            else if (shape == SHAPE_TRIANGLE) {
                // Apex at (0, 0.35), Bottom-Left at (-0.30311, -0.175), Bottom-Right at (+0.30311, -0.175)
                double d_base = distToSegment2D(u, w, -0.30311, -0.175, 0.30311, -0.175);
                double d_left = distToSegment2D(u, w, -0.30311, -0.175, 0.0, 0.35);
                double d_right = distToSegment2D(u, w, 0.30311, -0.175, 0.0, 0.35);
                double d_2d = std::min({d_base, d_left, d_right});
                double d_tube = std::hypot(d_2d, h);
                double clearance = d_tube - collision_dist;
                if (clearance < result.min_clearance_m) {
                    result.min_clearance_m = clearance;
                }
                if (d_tube < collision_dist) {
                    result.is_hit = true;
                    result.shape_hit = shape;
                    result.hit_message = "Clipped Triangle gate hoop rim";
                }
                // Vertical mounting pylon extending below base: w < -0.205m
                if (w < -0.205 && pz >= 0.0) {
                    double dist_pylon = std::hypot(rx, ry);
                    double pylon_clearance = dist_pylon - (0.015 + drone_radius);
                    if (pylon_clearance < result.min_clearance_m) {
                        result.min_clearance_m = pylon_clearance;
                    }
                    if (dist_pylon < (0.015 + drone_radius)) {
                        result.is_hit = true;
                        result.shape_hit = shape;
                        result.hit_message = "Collided with Triangle gate pylon";
                    }
                }
            }
            // 3. Square (shape 3): Upright square, half-width L = 0.17678m
            else if (shape == SHAPE_SQUARE) {
                const double L = 0.17678;
                double d_top = distToSegment2D(u, w, -L, L, L, L);
                double d_bot = distToSegment2D(u, w, -L, -L, L, -L);
                double d_left = distToSegment2D(u, w, -L, -L, -L, L);
                double d_right = distToSegment2D(u, w, L, -L, L, L);
                double d_2d = std::min({d_top, d_bot, d_left, d_right});
                double d_tube = std::hypot(d_2d, h);
                double clearance = d_tube - collision_dist;
                if (clearance < result.min_clearance_m) {
                    result.min_clearance_m = clearance;
                }
                if (d_tube < collision_dist) {
                    result.is_hit = true;
                    result.shape_hit = shape;
                    result.hit_message = "Clipped Square gate hoop rim";
                }
                // Vertical mounting pylon extending below bottom edge: w < -0.207m
                if (w < -0.207 && pz >= 0.0) {
                    double dist_pylon = std::hypot(rx, ry);
                    double pylon_clearance = dist_pylon - (0.015 + drone_radius);
                    if (pylon_clearance < result.min_clearance_m) {
                        result.min_clearance_m = pylon_clearance;
                    }
                    if (dist_pylon < (0.015 + drone_radius)) {
                        result.is_hit = true;
                        result.shape_hit = shape;
                        result.hit_message = "Collided with Square gate pylon";
                    }
                }
            }
        }

        return result;
    }
};

} // namespace sim
