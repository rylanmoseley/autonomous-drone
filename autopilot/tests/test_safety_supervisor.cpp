#include <gtest/gtest.h>
#include "safety_supervisor.h"
#include <thread>
#include <chrono>

TEST(SafetySupervisorTest, VisionTimeoutEstop) {
    SafetySupervisor safety(0.05); // 50ms timeout
    safety.recordVisionPacket();

    // Immediately after record, should not trip
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_PATH_FOLLOWING, 0.0));

    // Wait 70ms (> 50ms timeout)
    std::this_thread::sleep_for(std::chrono::milliseconds(70));
    EXPECT_TRUE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_PATH_FOLLOWING, 0.0));

    // Standby/Disarm state ignores timeout
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_STANDBY_DISARM, 0.0));
}

TEST(SafetySupervisorTest, FcuWatchdogTimeout) {
    SafetySupervisor safety(2.5); // long vision timeout
    safety.recordVisionPacket();

    safety.enableFcuWatchdog(true, 0.20); // 200ms FCU timeout
    EXPECT_TRUE(safety.isFcuWatchdogEnabled());

    // Within timeout (100ms < 200ms)
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_PATH_FOLLOWING, 0.10));

    // Exceeded timeout (300ms > 200ms)
    EXPECT_TRUE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_PATH_FOLLOWING, 0.30));
}

TEST(SafetySupervisorTest, EstopCommandGeneration) {
    FlightCommand cmd = SafetySupervisor::createEstopCommand();
    EXPECT_FALSE(cmd.enable);
    EXPECT_TRUE(cmd.estop);
}

TEST(SafetySupervisorTest, ArmedLaunchpadStateExemptFromVisionTimeout) {
    SafetySupervisor safety(0.04); // 40ms timeout
    safety.recordVisionPacket();

    std::this_thread::sleep_for(std::chrono::milliseconds(60)); // Vision has timed out

    // FLIGHT_STATE_ARMED (on launchpad before takeoff) MUST be exempt from tripping ESTOP
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_ARMED, 0.0));

    // In flight, timeout triggers ESTOP
    EXPECT_TRUE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_TAKEOFF, 0.0));
    EXPECT_TRUE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_PATH_FOLLOWING, 0.0));
}

TEST(SafetySupervisorTest, RotorStoppedAndEstopStatesExempt) {
    SafetySupervisor safety(0.02); // 20ms timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_ROTOR_STOPPED, 0.0));
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_ESTOP_TRIPPED, 0.0));
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_STANDBY_DISARM, 0.0));
}

TEST(SafetySupervisorTest, FcuWatchdogExemptions) {
    SafetySupervisor safety(5.0); // No vision timeout
    safety.recordVisionPacket();
    safety.enableFcuWatchdog(true, 0.05); // 50ms FCU timeout

    // 10.0 seconds stale telemetry
    double stale_fcu_s = 10.0;

    // Ground and inactive states are exempt from FCU timeout trip
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_STANDBY_DISARM, stale_fcu_s));
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_ARMED, stale_fcu_s));
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_ROTOR_STOPPED, stale_fcu_s));
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_ESTOP_TRIPPED, stale_fcu_s));

    // Airborne states trip ESTOP
    EXPECT_TRUE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_TAKEOFF, stale_fcu_s));
    EXPECT_TRUE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_PATH_FOLLOWING, stale_fcu_s));
}

TEST(SafetySupervisorTest, ResetWatchdogOnPacketArrival) {
    SafetySupervisor safety(0.04);
    safety.recordVisionPacket();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_PATH_FOLLOWING, 0.0));

    // Fresh packet arrives
    safety.recordVisionPacket();
    EXPECT_FALSE(safety.shouldTripEstop(FlightState::FLIGHT_STATE_PATH_FOLLOWING, 0.0));
}
