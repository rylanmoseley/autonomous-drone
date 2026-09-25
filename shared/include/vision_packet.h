#pragma once

struct VisionPose {
    double x, y, z;
    double roll, pitch, yaw;
    double timestamp;
    double confidence;
};

struct VisionGoalEstimate {
    VisionPose pose;
    int goal_id;
    bool determinate;
};
