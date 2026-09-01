#pragma once
#include <units.h>
#include <Eigen/Dense>

namespace HAL {

enum class ControlMode {
    AUTONOMOUS,
    MANUAL_FALLBACK,
    EMERGENCY_STOP
};

struct CommandData {
    units::time::millisecond_t timestamp;
    
    ControlMode mode;
    
    // Setpoints (could be velocity, position, or attitude depending on autonomy level)
    Eigen::Matrix<units::velocity::meters_per_second_t, 3, 1> velocity_setpoint;
    units::angle::degree_t yaw_setpoint;
    
    // Fallback direct overrides (if mode == MANUAL_FALLBACK)
    units::percentage::percent_t manual_throttle;
    units::angle::degree_t manual_roll;
    units::angle::degree_t manual_pitch;
    
    bool is_connected;
};

class CommandInterface {
public:
    virtual ~CommandInterface() = default;
    virtual void init() = 0;
    
    virtual bool update(CommandData& data) = 0;
};

} // namespace HAL
