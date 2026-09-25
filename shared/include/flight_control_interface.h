#pragma once
#include <vector>
#include <cstddef>

struct __attribute__((packed)) Pose {
    double x, y, z;
    double roll, pitch, yaw;
    double timestamp;
    double confidence;
};

struct __attribute__((packed)) FlightCommand {
    double target_x, target_y, target_z;
    double target_yaw;
    bool enable;
    bool estop;
};

struct __attribute__((packed)) Telemetry {
    Pose current_pose;
    int battery_level;
    bool error_state;
};
