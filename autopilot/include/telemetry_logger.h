#pragma once
#include <string>

class ITelemetryLogger {
public:
    virtual ~ITelemetryLogger() = default;
    virtual void log(const std::string& key, const std::string& value) = 0;
    virtual void log(const std::string& key, double value) = 0;
    virtual void log(const std::string& key, int value) = 0;
    virtual void flush() = 0;
};
