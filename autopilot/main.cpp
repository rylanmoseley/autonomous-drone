#include "autopilot_node.h"
#include <csignal>
#include <iostream>

AutopilotNode* node_ptr = nullptr;

void handle_sigint(int) {
    if (node_ptr) {
        node_ptr->stop();
    }
}

int main(int argc, char** argv) {
    AutopilotNode node;
    node_ptr = &node;
    signal(SIGINT, handle_sigint);
    
    std::cout << "Starting Autopilot Node..." << std::endl;
    node.start();
    
    // Block main thread until node stops
    while(node.running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
