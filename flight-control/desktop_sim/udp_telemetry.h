#pragma once
#include "physics_engine.h"
#include <string>

class UdpTelemetry {
public:
    UdpTelemetry(const std::string& ip_address, int send_port, int listen_port);
    ~UdpTelemetry();

    // Broadcast the TrueState over UDP as JSON
    void broadcast_state(const PhysicsEngine::TrueState& state, const Config::DroneHardwareConfig& config);
    
    // Non-blocking poll for incoming commands/configs
    // Returns true if a config update was received
    bool poll_commands(Config::DroneHardwareConfig& out_config);

private:
    int sockfd_;
    struct sockaddr_in dest_addr_;
};
