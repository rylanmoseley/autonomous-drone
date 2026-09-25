#pragma once
#include <cstddef>

struct __attribute__((packed)) UIGoalPose {
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
