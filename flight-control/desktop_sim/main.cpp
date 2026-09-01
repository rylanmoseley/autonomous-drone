#include <iostream>
#include <thread>
#include <chrono>
#include "control_system/control_system.h"
#include "mock_hardware.h"
#include "physics_engine.h"

int main() {
    std::cout << "Starting Software-In-The-Loop Simulation..." << std::endl;

    // 1. Setup Drone Hardware Configuration (Runtime Configurable)
    Config::DroneHardwareConfig config;
    config.mass = units::mass::kilogram_t(1.2); // 1.2kg drone
    // TODO: Populate realistic inertia tensor, rotor positions, ESC rates, etc.

    // 2. Instantiate the Physics Engine (Below HAL)
    PhysicsEngine sim_engine(config);

    // 3. Instantiate Mock Hardware Drivers and bind them to the Physics Engine
    // TODO: Pass a reference to `sim_engine` into these mock drivers so they can read/write the TrueState
    MockIMU imu_sensor;
    MockMotors motor_driver;
    
    // 4. Instantiate the Flight Control System (Above HAL)
    ControlSystem flight_controller(&imu_sensor, &motor_driver);
    flight_controller.init();

    // 5. Main Simulation Loop
    units::time::second_t dt(0.001); // 1000Hz internal simulation step
    
    while(true) {
        // Step the physics engine
        sim_engine.step(dt);
        
        // Step the flight controller (which polls the HAL)
        // TODO: In a real environment, the flight loop might run at a different rate than the physics engine.
        // We should manage time/scheduling here to respect Config::ProtocolConfig rates.
        flight_controller.tick();
        
        // Allow external runtime configurations
        // TODO: Poll a UDP socket for configuration changes and call sim_engine.update_config()

        // Sleep to maintain real-time factor (or run as fast as possible for offline ML training)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
