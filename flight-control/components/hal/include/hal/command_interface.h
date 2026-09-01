#pragma once
#include <units.h>
#include <Eigen/Dense>

namespace HAL {

enum class ControlMode {
    AUTONOMOUS,
    MANUAL_FALLBACK,
    EMERGENCY_STOP
};

// 6D state representation (e.g., 3D Linear + 3D Angular).
// We use a struct to allow strict typing of the different units (e.g., meters vs radians)
// while conceptually treating it as a 6D spatial vector.
struct SpatialState6D {
    // Depending on the exact control loop, these could represent Position/Attitude or Velocity/Rates.
    // Assuming Velocity/Rates (Twist) as the most common interface for high-level drone autonomy:
    Eigen::Vector3f linear;
    Eigen::Vector3f angular;
};

struct CommandData {
    units::time::millisecond_t timestamp;
    
    ControlMode mode;
    SpatialState6D setpoint;
    
    bool is_connected;
};

class CommandInterface {
public:
    virtual ~CommandInterface() = default;
    virtual void init() = 0;
    
    // Polled by the internal flight loop.
    // Takes in the current Process Variable (PV) to send back to the higher-order controller (telemetry),
    // and populates `new_command` with the latest desired setpoint (from autonomy or manual fallback).
    virtual bool update(const SpatialState6D& current_pv, CommandData& new_command) = 0;
};

} // namespace HAL
