#include <iostream>
#include <thread>
#include <chrono>
#include "control_system/control_system.h"
#include "mock_hardware.h"
#include "physics_engine.h"

#include "udp_telemetry.h"

int main() {
    std::cout << "Starting Software-In-The-Loop Simulation..." << std::endl;

    // 1. Setup Drone Hardware Configuration (Runtime Configurable)
    Config::DroneHardwareConfig config;
    config.mass = units::mass::kilogram_t(1.2); // 1.2kg drone
    
    // Setup a default diagonal inertia tensor (approximate for a quadcopter)
    config.moment_of_inertia = Eigen::Matrix3f::Identity() * 0.05f; 
    
    config.linear_drag_coefficient = 0.5f;
    config.battery_params.nominal_voltage = units::voltage::volt_t(14.8);
    config.motor_params.time_constant = units::time::second_t(0.05);
    config.motor_params.max_rpm = units::angular_velocity::revolutions_per_minute_t(10000);

    // 2. Instantiate the Physics Engine (Below HAL)
    PhysicsEngine sim_engine(config);

    // 3. Instantiate Mock Hardware Drivers and bind them to the Physics Engine
    MockIMU imu_sensor(&sim_engine);
    MockMotors motor_driver(&sim_engine);
    
    // 4. Instantiate the Flight Control System (Above HAL)
    ControlSystem flight_controller(&imu_sensor, &motor_driver);
    flight_controller.init();

    // Setup Telemetry to Node.js proxy (Listening on 9001, Sending to 9000)
    UdpTelemetry telemetry("127.0.0.1", 9000, 9001);

    // 5. Main Simulation Loop
    units::time::second_t dt(0.001); // 1000Hz internal simulation step
    int tick_count = 0;
    
    CommandVector cmd;
    
    while(true) {
        // Allow external runtime configurations
        if (telemetry.poll_commands(config, cmd)) {
            sim_engine.update_config(config);
            flight_controller.set_config(config); // Keep controller in sync with dynamic rotors
        }
        
        flight_controller.set_command(cmd);

        // Step the physics engine
        sim_engine.step(dt);
        
        // Step the flight controller (which polls the HAL)
        flight_controller.tick();
        
        // Broadcast telemetry at ~60Hz (every ~16 ticks at 1000Hz)
        if (tick_count % 16 == 0) {
            telemetry.broadcast_state(sim_engine.get_true_state(), sim_engine.get_config());
        }
        
        // Sleep to maintain real-time factor
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        tick_count++;
    }

    return 0;
}
