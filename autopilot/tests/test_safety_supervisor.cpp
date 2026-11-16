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
