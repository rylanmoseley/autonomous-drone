#pragma once

#include <units.h>
#include <Eigen/Dense>

namespace HAL {

// Strongly typed Eigen vectors
using AccelVector = Eigen::Matrix<units::acceleration::meters_per_second_squared_t, 3, 1>;
using GyroVector = Eigen::Matrix<units::angular_velocity::radians_per_second_t, 3, 1>;
using MagVector = Eigen::Matrix<units::magnetic_field::gauss_t, 3, 1>;

struct IMUData {
    units::time::millisecond_t timestamp;
    
    AccelVector accel;      // X: Forward, Y: Right, Z: Down (NED)
    GyroVector gyro;        // Roll, Pitch, Yaw
    MagVector mag;          // Magnetometer data
    
    units::temperature::celsius_t temperature;
};

class IMUInterface {
public:
    virtual ~IMUInterface() = default;
    virtual void init() = 0;
    
    // Polled by the strictly scheduled internal flight loop
    virtual bool update(IMUData& data) = 0;
    
    // Calibration hooks
    virtual void calibrate_gyro() = 0;
    virtual void calibrate_accel() = 0;
};

} // namespace HAL
