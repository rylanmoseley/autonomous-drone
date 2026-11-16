#pragma once
#include <chrono>

class MavlinkFcuWatchdog {
public:
    static bool isConnected(double time_since_last_telemetry, double timeout_s = 1.0) {
        return time_since_last_telemetry <= timeout_s;
    }
};
