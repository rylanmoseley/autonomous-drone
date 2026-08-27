#include <iostream>
#include <thread>
#include <chrono>
#include "mock_hardware.h"
#include "control_system/control_system.h"

int main() {
    std::cout << "Starting Desktop Flight Control Simulation..." << std::endl;

    MockIMU mock_imu;
    MockMotors mock_motors;

    ControlSystem drone(&mock_imu, &mock_motors);
    drone.init();

    std::cout << "Running simulated control loop (Press Ctrl+C to stop)..." << std::endl;
    
    // Simulate a 100Hz loop (10ms)
    while (true) {
        drone.tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
