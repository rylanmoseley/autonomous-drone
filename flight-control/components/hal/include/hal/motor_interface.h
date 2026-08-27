#pragma once

#include <units.h>

using namespace units;

namespace HAL {

class MotorInterface {
public:
    virtual ~MotorInterface() = default;
    virtual void init() = 0;
    
    virtual void set_torque(int motor_id, torque::newton_meter_t output) = 0;
    virtual void set_speed(int motor_id, angular_velocity::revolutions_per_minute_t speed) = 0;
    virtual angular_velocity::revolutions_per_minute_t get_speed(int motor_id) = 0;
    virtual current::amperes get_current(int motor_id) = 0;
    virtual temperature::celsius get_temperature(int motor_id) = 0;
};

} // namespace HAL
