#pragma once

#include <units.h>
#include <Eigen/Dense>

namespace HAL {

// Accelerometer returns m/s^2
using AccelVector = Eigen::Vector3f;

// Gyro returns rad/s
using GyroVector = Eigen::Vector3f;

// Magnetometer returns raw floats (or we can use microteslas if available)
using MagVector = Eigen::Vector3f;

struct IMUData {
    units::time::millisecond_t timestamp;
    
    AccelVector accel;      // Linear acceleration in body frame
    GyroVector gyro;        // Angular velocity in body frame
    MagVector mag;          // Magnetometer data
    
    // Some IMUs compute temperature
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
