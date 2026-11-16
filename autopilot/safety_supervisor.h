#pragma once
#include <chrono>
#include <mutex>
#include "flight_control_interface.h"

class SafetySupervisor {
public:
    explicit SafetySupervisor(double vision_timeout_s = 2.5);

    void recordVisionPacket();
    double getTimeSinceLastVision() const;

    void setVisionTimeout(double timeout_s) { vision_timeout_s_ = timeout_s; }
    double getVisionTimeout() const { return vision_timeout_s_; }

    void enableFcuWatchdog(bool enable, double timeout_s = 1.0) {
        fcu_watchdog_enabled_ = enable;
        fcu_timeout_s_ = timeout_s;
    }
    bool isFcuWatchdogEnabled() const { return fcu_watchdog_enabled_; }
    double getFcuTimeout() const { return fcu_timeout_s_; }

    // Evaluates whether safety timeout has occurred; returns true if E-Stop should be triggered
    bool shouldTripEstop(FlightState current_state, double time_since_fcu_telemetry) const;

    static FlightCommand createEstopCommand();

    std::chrono::steady_clock::time_point last_vision_time;

private:
    double vision_timeout_s_;
    bool fcu_watchdog_enabled_{false};
    double fcu_timeout_s_{1.0};
    mutable std::mutex mutex_;
};
