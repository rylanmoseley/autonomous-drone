#pragma once

#include <cstdint>
#include <string>
#include "flight_control_interface.h"

namespace sim {

struct SimulatedGoal {
    uint8_t shape_type; // SHAPE_CIRCLE, SHAPE_TRIANGLE, SHAPE_SQUARE
    double x;           // Position in World NED
    double y;
    double z;
    double yaw;         // Heading (radians)
};

struct SimulatedTag {
    uint16_t tag_id;
    double x;           // World NED coordinates of tag anchor
    double y;
    double z;
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    float confidence{1.0f};
};

struct TrajectoryPoint {
    double timestamp;
    Pose true_pose;
    Pose feedback_pose;
    FlightCommand command;
    FlightState flight_state;
};

struct CollisionRecord {
    double timestamp{0.0};
    uint8_t shape_type{0};
    std::string description;
    Pose drone_pose{};
    double clearance{0.0};
};

} // namespace sim
