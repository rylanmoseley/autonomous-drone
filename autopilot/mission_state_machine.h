#pragma once
#include "flight_control_interface.h"
#include <mutex>

class MissionStateMachine {
public:
    explicit MissionStateMachine(FlightState initial_state = FlightState::FLIGHT_STATE_STANDBY_DISARM);

    FlightState getState() const;
    void setState(FlightState state);

    bool isArmed() const { return is_armed_; }
    void setArmed(bool armed) { is_armed_ = armed; }

    bool isEstopActive() const { return estop_active_; }
    void setEstopActive(bool estop) { estop_active_ = estop; }

    // State transition helpers triggered by UI or Mission Events
    void handleArmCommand();
    void handleDisarmCommand();
    void handleStartMissionCommand(bool in_air);
    void handlePauseMissionCommand();
    void handleResumeMissionCommand();
    void handleLandCommand();
    void handleEstopCommand();

    // Flight maneuver targets
    static Pose computeTakeoffTarget(const Pose& current_pose, bool use_tube, float tube_rail_z);
    static Pose computeLandingStep(const Pose& current_pose, double dt, bool& out_touchdown);
    static Pose computeHoverHold(const Pose& current_pose, bool has_hover_pose, const Pose& current_hover_pose);

private:
    FlightState state_;
    bool is_armed_{false};
    bool estop_active_{false};
    mutable std::mutex mutex_;
};
