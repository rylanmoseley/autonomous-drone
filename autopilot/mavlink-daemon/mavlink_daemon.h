#pragma once
#include "flight_control_interface.h"
#include <string>
#include <mutex>
#include <thread>
#include <atomic>

class MavlinkDaemon {
public:
    MavlinkDaemon(int local_port, const std::string& remote_ip, int remote_port);
    ~MavlinkDaemon();
    
    void sendFlightCommand(const FlightCommand& cmd);
    Telemetry getTelemetry();
    void start();
    void stop();

private:
    void receiveLoop();

    int local_port_;
    std::string remote_ip_;
    int remote_port_;
    int socket_fd_;
    
    std::atomic<bool> running_;
    std::thread rx_thread_;
    
    Telemetry current_telemetry_;
    std::mutex telem_mutex_;
};
