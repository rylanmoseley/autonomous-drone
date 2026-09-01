#pragma once
#include <units.h>

namespace HAL {

struct BatteryData {
    units::time::millisecond_t timestamp;
    
    units::voltage::volt_t voltage;
    units::current::ampere_t current;
    units::charge::milliampere_hour_t consumed_capacity;
};

class BatteryInterface {
public:
    virtual ~BatteryInterface() = default;
    virtual void init() = 0;
    virtual bool update(BatteryData& data) = 0;
};

} // namespace HAL
