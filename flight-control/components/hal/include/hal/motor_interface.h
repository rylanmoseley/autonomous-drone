#pragma once

namespace HAL {

class MotorInterface {
public:
    virtual ~MotorInterface() = default;
    virtual void init() = 0;
    virtual void set_throttle(int motor_index, float percentage) = 0;
};

} // namespace HAL
