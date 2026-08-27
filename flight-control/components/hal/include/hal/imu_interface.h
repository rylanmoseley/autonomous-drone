#pragma once

namespace HAL {

class IMUInterface {
public:
    virtual ~IMUInterface() = default;
    virtual void init() = 0;
    virtual void read_accel(float& x, float& y, float& z) = 0;
    virtual void read_gyro(float& x, float& y, float& z) = 0;
};

} // namespace HAL
