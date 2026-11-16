#pragma once
#include <cstdint>
#include "flight_control_interface.h"
#include "vision_packet.h"
#include "telemetry_packet.h"

struct TrackedGoal {
    uint8_t shape_type = SHAPE_NONE;
    int goal_id = 0;
    uint8_t state = TRACK_STATE_SEARCHING;
    Pose pose = {};
    float spatial_variance = 0.0f;
    double last_update_time = 0.0;
    bool determinate = false;
};
