#pragma once
#include <units.h>

namespace HAL {

struct ToFData {
    units::time::millisecond_t timestamp;
    
    units::length::meter_t distance;
    units::percentage::percent_t signal_quality; // Confidence in the measurement
};

class ToFSensorInterface {
public:
    virtual ~ToFSensorInterface() = default;
    virtual void init() = 0;
    
    // Polled by the strictly scheduled internal flight loop
    virtual bool update(ToFData& data) = 0;
};

} // namespace HAL
