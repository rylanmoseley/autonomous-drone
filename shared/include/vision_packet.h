#pragma once

struct __attribute__((packed)) VisionPose {
    double x, y, z;
    double roll, pitch, yaw;
    double timestamp;
    double confidence;
};

struct __attribute__((packed)) VisionGoalEstimate {
    VisionPose pose;
    int goal_id;
    bool determinate;
};
