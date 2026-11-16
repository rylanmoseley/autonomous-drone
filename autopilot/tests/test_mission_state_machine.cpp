#include <gtest/gtest.h>
#include "mission_state_machine.h"

TEST(MissionStateMachineTest, InitialStateAndArmDisarm) {
    MissionStateMachine sm;
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_STANDBY_DISARM);
    EXPECT_FALSE(sm.isArmed());
    EXPECT_FALSE(sm.isEstopActive());

    sm.handleArmCommand();
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_ARMED);
    EXPECT_TRUE(sm.isArmed());

    sm.handleDisarmCommand();
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_STANDBY_DISARM);
    EXPECT_FALSE(sm.isArmed());
}

TEST(MissionStateMachineTest, StartMissionTakeoffVsInAir) {
    MissionStateMachine sm;
    // Ground start -> TAKEOFF
    sm.handleStartMissionCommand(false);
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_TAKEOFF);
    EXPECT_TRUE(sm.isArmed());

    // Pause mission -> HOVER_WAIT
    sm.setState(FlightState::FLIGHT_STATE_PATH_FOLLOWING);
    sm.handlePauseMissionCommand();
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_HOVER_WAIT);

    // Resume mission -> PATH_FOLLOWING
    sm.handleResumeMissionCommand();
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_PATH_FOLLOWING);

    // In-air start from HOVER_WAIT -> PATH_FOLLOWING directly
    sm.setState(FlightState::FLIGHT_STATE_HOVER_WAIT);
    sm.handleStartMissionCommand(true);
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_PATH_FOLLOWING);
}

TEST(MissionStateMachineTest, TakeoffTargetComputation) {
    Pose cur{0.0, 0.0, 0.0, 0, 0, 0.5, 0, 1.0};
    Pose target = MissionStateMachine::computeTakeoffTarget(cur, false, 0.0f);
    EXPECT_FLOAT_EQ(target.x, 0.0f);
    EXPECT_FLOAT_EQ(target.y, 0.0f);
    EXPECT_FLOAT_EQ(target.z, 1.0f); // Default takeoff 1.0m
    EXPECT_FLOAT_EQ(target.yaw, 0.5f);

    // With tube rail constraint at 1.5m
    Pose target_tube = MissionStateMachine::computeTakeoffTarget(cur, true, 1.5f);
    EXPECT_FLOAT_EQ(target_tube.z, 1.5f);
}

TEST(MissionStateMachineTest, LandingRampAndTouchdown) {
    Pose cur{1.0, 2.0, 0.8, 0, 0, 0, 0, 1.0};
    double dt = 0.5; // step = 0.40 * 0.5 = 0.20m
    bool touchdown = false;

    Pose next = MissionStateMachine::computeLandingStep(cur, dt, touchdown);
    EXPECT_NEAR(next.z, 0.6, 1e-4);
    EXPECT_FALSE(touchdown);

    // Near ground touchdown check
    Pose near_ground{1.0, 2.0, 0.04, 0, 0, 0, 0, 1.0};
    next = MissionStateMachine::computeLandingStep(near_ground, dt, touchdown);
    EXPECT_TRUE(touchdown);
}

TEST(MissionStateMachineTest, EstopTripping) {
    MissionStateMachine sm;
    sm.setState(FlightState::FLIGHT_STATE_PATH_FOLLOWING);
    sm.handleEstopCommand();
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_ESTOP_TRIPPED);
    EXPECT_TRUE(sm.isEstopActive());
}

TEST(MissionStateMachineTest, InFlightDisarmRejectedUnlessForced) {
    std::vector<FlightState> in_flight_states = {
        FlightState::FLIGHT_STATE_TAKEOFF,
        FlightState::FLIGHT_STATE_PATH_FOLLOWING,
        FlightState::FLIGHT_STATE_GOAL_SEEK,
        FlightState::FLIGHT_STATE_HOVER_WAIT,
        FlightState::FLIGHT_STATE_LANDING
    };

    for (FlightState state : in_flight_states) {
        MissionStateMachine sm;
        sm.handleArmCommand();
        sm.setState(state);
        EXPECT_TRUE(sm.isArmed());

        // Unforced in-flight disarm command MUST be rejected to prevent motor cut
        sm.handleDisarmCommand(false);
        EXPECT_EQ(sm.getState(), state);
        EXPECT_TRUE(sm.isArmed());

        // Explicit emergency forced disarm MUST be honored
        sm.handleDisarmCommand(true);
        EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_STANDBY_DISARM);
        EXPECT_FALSE(sm.isArmed());
    }
}

TEST(MissionStateMachineTest, DisarmAllowedOnGround) {
    // 1. Armed on ground -> Disarm moves to STANDBY_DISARM
    MissionStateMachine sm;
    sm.handleArmCommand();
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_ARMED);
    EXPECT_TRUE(sm.isArmed());

    sm.handleDisarmCommand(false);
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_STANDBY_DISARM);
    EXPECT_FALSE(sm.isArmed());

    // 2. Rotor stopped on ground -> Disarm remains disarmed
    sm.setState(FlightState::FLIGHT_STATE_ROTOR_STOPPED);
    sm.handleDisarmCommand(false);
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_STANDBY_DISARM);
    EXPECT_FALSE(sm.isArmed());

    // 3. E-stop tripped -> Disarm transitions to STANDBY_DISARM and clears armed
    sm.setState(FlightState::FLIGHT_STATE_ESTOP_TRIPPED);
    sm.handleDisarmCommand(false);
    EXPECT_EQ(sm.getState(), FlightState::FLIGHT_STATE_STANDBY_DISARM);
    EXPECT_FALSE(sm.isArmed());
}
