#include "mission_state_machine.h"
#include <cmath>
#include <algorithm>

MissionStateMachine::MissionStateMachine(FlightState initial_state)
    : state_(initial_state), is_armed_(false), estop_active_(false) {}

FlightState MissionStateMachine::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void MissionStateMachine::setState(FlightState state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
}

void MissionStateMachine::handleArmCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    is_armed_ = true;
    estop_active_ = false;
    if (state_ == FlightState::FLIGHT_STATE_STANDBY_DISARM ||
        state_ == FlightState::FLIGHT_STATE_ROTOR_STOPPED ||
        state_ == FlightState::FLIGHT_STATE_ESTOP_TRIPPED) {
        state_ = FlightState::FLIGHT_STATE_ARMED;
    }
}

void MissionStateMachine::handleDisarmCommand(bool force) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!force && (state_ == FlightState::FLIGHT_STATE_TAKEOFF ||
                   state_ == FlightState::FLIGHT_STATE_PATH_FOLLOWING ||
                   state_ == FlightState::FLIGHT_STATE_GOAL_SEEK ||
                   state_ == FlightState::FLIGHT_STATE_HOVER_WAIT ||
                   state_ == FlightState::FLIGHT_STATE_LANDING)) {
        // Disarm rejected while airborne; require explicit LAND or ESTOP
        return;
    }
    is_armed_ = false;
    estop_active_ = false;
    state_ = FlightState::FLIGHT_STATE_STANDBY_DISARM;
}

void MissionStateMachine::handleStartMissionCommand(bool in_air) {
    std::lock_guard<std::mutex> lock(mutex_);
    is_armed_ = true;
    estop_active_ = false;
    if (in_air && state_ == FlightState::FLIGHT_STATE_HOVER_WAIT) {
        state_ = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
    } else {
        state_ = FlightState::FLIGHT_STATE_TAKEOFF;
    }
}

void MissionStateMachine::handlePauseMissionCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == FlightState::FLIGHT_STATE_PATH_FOLLOWING ||
        state_ == FlightState::FLIGHT_STATE_GOAL_SEEK) {
        state_ = FlightState::FLIGHT_STATE_HOVER_WAIT;
    }
}

void MissionStateMachine::handleResumeMissionCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == FlightState::FLIGHT_STATE_HOVER_WAIT) {
        state_ = FlightState::FLIGHT_STATE_PATH_FOLLOWING;
    }
}

void MissionStateMachine::handleLandCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = FlightState::FLIGHT_STATE_LANDING;
}

void MissionStateMachine::handleEstopCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = FlightState::FLIGHT_STATE_ESTOP_TRIPPED;
    estop_active_ = true;
}

Pose MissionStateMachine::computeTakeoffTarget(const Pose& current_pose, bool use_tube, float tube_rail_z) {
    double takeoff_z = (use_tube && tube_rail_z != 0.0f) ? std::abs(tube_rail_z) : 1.0;
    return {current_pose.x, current_pose.y, takeoff_z, 0, 0, current_pose.yaw, 0, 1.0};
}

Pose MissionStateMachine::computeLandingStep(const Pose& current_pose, double dt, bool& out_touchdown) {
    double next_z = std::max(0.0, current_pose.z - 0.40 * dt);
    out_touchdown = (current_pose.z <= 0.05);
    return {current_pose.x, current_pose.y, next_z, 0, 0, current_pose.yaw, 0, 1.0};
}

Pose MissionStateMachine::computeHoverHold(const Pose& current_pose, bool has_hover_pose, const Pose& current_hover_pose) {
    if (has_hover_pose) {
        return current_hover_pose;
    }
    return current_pose;
}
