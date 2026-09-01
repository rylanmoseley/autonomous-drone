#pragma once

#include "physics_engine.h"
#include "hal/drone_config.h"
#include <string>
#include <netinet/in.h>

#include "control_system/control_system.h" // For CommandVector

class UdpTelemetry {
public:
    UdpTelemetry(const std::string& node_ip, int listen_port, int send_port);
    ~UdpTelemetry();

    // Broadcast the TrueState over UDP as JSON
    void broadcast_state(const PhysicsEngine::TrueState& state, const Config::DroneHardwareConfig& config);
    
    // Non-blocking poll for incoming commands/configs
    // Returns true if a config update was received
    bool poll_commands(Config::DroneHardwareConfig& out_config, CommandVector& out_cmd);

private:
    int sockfd_;
    struct sockaddr_in dest_addr_;
};
