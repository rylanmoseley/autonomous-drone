#include "autopilot_node.h"
#include <csignal>
#include <atomic>
#include <iostream>
#include <chrono>
#include <thread>

static std::atomic<bool> g_shutdown_requested{false};

void handle_signal(int) {
    g_shutdown_requested = true;
}

int main(int /*argc*/, char** /*argv*/) {
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    
    AutopilotNode node;
    std::cout << "Starting Autopilot Node..." << std::endl;
    node.start();
    
    // Block main thread until node stops or signal received
    while (node.running && !g_shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Stopping Autopilot Node..." << std::endl;
    node.stop();
    
    return 0;
}
