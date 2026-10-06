#pragma once
#include <cstddef>
#include <cstdint>

struct __attribute__((packed)) UIGoalPose {
    uint32_t id;
    double x, y, z;
    double roll, pitch, yaw;
    double timestamp;
    double confidence;
};

struct __attribute__((packed)) UIGoalSequence {
    UIGoalPose goals[16];
    size_t num_goals;
    bool start;
    bool stop;
    bool estop;
};

struct UIAckPacket {
    bool received;
};

struct __attribute__((packed)) UIConfigPacket {
    double max_velocity;
    double max_vertical_velocity;
    double max_yaw_rate;
};
