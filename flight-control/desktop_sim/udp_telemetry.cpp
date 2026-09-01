#include "udp_telemetry.h"
#include "third_party/json.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

using json = nlohmann::json;

UdpTelemetry::UdpTelemetry(const std::string& ip_address, int send_port, int listen_port) {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    
    // Make socket non-blocking
    int flags = fcntl(sockfd_, F_GETFL, 0);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

    // Setup destination address for broadcasting
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(send_port);
    inet_pton(AF_INET, ip_address.c_str(), &dest_addr_.sin_addr);
    
    // Bind to listen port
    struct sockaddr_in listen_addr;
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(listen_port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;
    bind(sockfd_, (struct sockaddr*)&listen_addr, sizeof(listen_addr));
}

UdpTelemetry::~UdpTelemetry() {
    close(sockfd_);
}

void UdpTelemetry::broadcast_state(const PhysicsEngine::TrueState& state, const Config::DroneHardwareConfig& config) {
    json j;
    j["type"] = "telemetry";
    
    j["position"] = {state.position(0), state.position(1), state.position(2)};
    j["velocity"] = {state.velocity(0), state.velocity(1), state.velocity(2)};
    j["euler_angles"] = {state.euler_angles(0), state.euler_angles(1), state.euler_angles(2)};
    j["angular_velocity"] = {state.angular_velocity(0), state.angular_velocity(1), state.angular_velocity(2)};
    
    j["battery_voltage"] = state.battery_voltage.to<float>();
    j["current_draw"] = state.total_current_draw.to<float>();
    j["consumed_capacity"] = state.consumed_capacity.to<float>();
    
    // Include minimal config info if needed for dynamic rendering
    json rotors = json::array();
    for (const auto& r : config.rotors) {
        rotors.push_back({
            {"position", {r.position(0), r.position(1), r.position(2)}}
        });
    }
    j["rotors"] = rotors;

    std::string payload = j.dump();
    sendto(sockfd_, payload.c_str(), payload.length(), 0, (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
}

bool UdpTelemetry::poll_commands(Config::DroneHardwareConfig& out_config, CommandVector& out_cmd) {
    char buffer[4096];
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    
    bool config_updated = false;
    
    while (true) {
        int bytes = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&src_addr, &src_len);
        if (bytes <= 0) break; // No more data
        
        buffer[bytes] = '\0';
        try {
            json j = json::parse(buffer);
            if (j["type"] == "config") {
                if (j["data"].contains("mass")) {
                    out_config.mass = units::mass::kilogram_t(j["data"]["mass"].get<float>());
                }
                if (j["data"].contains("linear_drag_coefficient")) {
                    out_config.linear_drag_coefficient = j["data"]["linear_drag_coefficient"].get<float>();
                }
                if (j["data"].contains("nominal_voltage")) {
                    out_config.battery_params.nominal_voltage = units::voltage::volt_t(j["data"]["nominal_voltage"].get<float>());
                }
                if (j["data"].contains("rotors") && j["data"]["rotors"].is_array()) {
                    out_config.rotors.clear();
                    for (const auto& r : j["data"]["rotors"]) {
                        Config::RotorConfig rc;
                        
                        // Default thrust axis (Z-up, so thrust points down? Wait, thrust pushes UP, so thrust axis is +Z)
                        rc.thrust_axis = Eigen::Vector3f(0, 0, 1);
                        
                        if (r.contains("position") && r["position"].is_array() && r["position"].size() == 3) {
                            rc.position(0) = r["position"][0].get<float>();
                            rc.position(1) = r["position"][1].get<float>();
                            rc.position(2) = r["position"][2].get<float>();
                        }
                        
                        if (r.contains("thrust_coefficient")) rc.thrust_coefficient = r["thrust_coefficient"].get<float>();
                        else rc.thrust_coefficient = 0.0f;

                        if (r.contains("torque_coefficient")) rc.torque_coefficient = r["torque_coefficient"].get<float>();
                        else rc.torque_coefficient = 0.0f;

                        if (r.contains("spins_clockwise")) rc.spins_clockwise = r["spins_clockwise"].get<bool>();
                        else rc.spins_clockwise = true;
                        
                        out_config.rotors.push_back(rc);
                    }
                }
                config_updated = true;
            } else if (j["type"] == "command") {
                if (j["data"].contains("lx")) out_cmd.lx = j["data"]["lx"].get<float>();
                if (j["data"].contains("ly")) out_cmd.ly = j["data"]["ly"].get<float>();
                if (j["data"].contains("lz")) out_cmd.lz = j["data"]["lz"].get<float>();
                if (j["data"].contains("ax")) out_cmd.ax = j["data"]["ax"].get<float>();
                if (j["data"].contains("ay")) out_cmd.ay = j["data"]["ay"].get<float>();
                if (j["data"].contains("az")) out_cmd.az = j["data"]["az"].get<float>();
            }
        } catch (...) {
            // Silently ignore parse errors for UDP packets
        }
    }
    return config_updated;
}
