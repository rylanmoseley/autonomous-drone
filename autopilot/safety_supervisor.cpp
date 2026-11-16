#include "safety_supervisor.h"

SafetySupervisor::SafetySupervisor(double vision_timeout_s)
    : last_vision_time(std::chrono::steady_clock::now()),
      vision_timeout_s_(vision_timeout_s) {}

void SafetySupervisor::recordVisionPacket() {
    std::lock_guard<std::mutex> lock(mutex_);
    last_vision_time = std::chrono::steady_clock::now();
}

double SafetySupervisor::getTimeSinceLastVision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last_vision_time).count();
}

bool SafetySupervisor::shouldTripEstop(FlightState current_state, double time_since_fcu_telemetry) const {
    if (current_state == FlightState::FLIGHT_STATE_ESTOP_TRIPPED ||
        current_state == FlightState::FLIGHT_STATE_STANDBY_DISARM ||
        current_state == FlightState::FLIGHT_STATE_ROTOR_STOPPED) {
        return false;
    }

    if (getTimeSinceLastVision() > vision_timeout_s_) {
        return true;
    }

    if (fcu_watchdog_enabled_ && time_since_fcu_telemetry > fcu_timeout_s_) {
        return true;
    }

    return false;
}

FlightCommand SafetySupervisor::createEstopCommand() {
    FlightCommand cmd = {};
    cmd.enable = false;
    cmd.estop = true;
    return cmd;
}
