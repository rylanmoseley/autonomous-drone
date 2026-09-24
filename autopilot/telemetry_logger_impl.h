#pragma once
#include "telemetry_logger.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <map>
#include <sstream>

class UdpTelemetryLogger : public ITelemetryLogger {
    int sock;
    struct sockaddr_in dest;
    std::map<std::string, std::string> buffer;
public:
    UdpTelemetryLogger(const std::string& ip, int port) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        dest.sin_family = AF_INET;
        dest.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &dest.sin_addr);
    }
    ~UdpTelemetryLogger() { if (sock >= 0) close(sock); }
    
    void log(const std::string& key, const std::string& value) override {
        buffer[key] = "\"" + value + "\"";
    }
    void log(const std::string& key, double value) override {
        buffer[key] = std::to_string(value);
    }
    void log(const std::string& key, int value) override {
        buffer[key] = std::to_string(value);
    }
    void flush() override {
        if (buffer.empty()) return;
        std::stringstream ss;
        ss << "{";
        bool first = true;
        for (const auto& [k, v] : buffer) {
            if (!first) ss << ",";
            ss << "\"" << k << "\":" << v;
            first = false;
        }
        ss << "}";
        std::string payload = ss.str();
        sendto(sock, payload.c_str(), payload.size(), 0, (struct sockaddr*)&dest, sizeof(dest));
        buffer.clear();
    }
};
