#pragma once

#include "common_types.h"
#include <vector>
#include <string>

namespace sim {

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
    static double distToSegment2D(double pu, double pw, double ax, double ay, double bx, double by);

    // Check collision and calculate obstacle clearance for a drone at (px, py, pz, pyaw)
    static CollisionCheckResult check(double px, double py, double pz, double pyaw,
                                      const std::vector<SimulatedGoalDef>& goals,
                                      double drone_radius = 0.06);

    // Generic container adapter
    template <typename GoalContainer>
    static CollisionCheckResult check(double px, double py, double pz, double pyaw,
                                      const GoalContainer& goals,
                                      double drone_radius = 0.06) {
        std::vector<SimulatedGoalDef> defs;
        defs.reserve(goals.size());
        for (const auto& g : goals) {
            defs.push_back({g.shape_type, g.x, g.y, g.z, g.yaw});
        }
        return check(px, py, pz, pyaw, defs, drone_radius);
    }
};

} // namespace sim
