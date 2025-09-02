#include "state_estimator/state_estimator_node.hpp"
#include <lcm/lcm-cpp.hpp>
#include <csignal>
#include <memory>
#include <thread>
#include <chrono>

sig_atomic_t volatile g_request_shutdown = 0;

void mySigintHandler(int sig) {
    (void)sig;
    g_request_shutdown = 1;
}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);

    // Create the LCM instance
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "ERROR: LCM initialization failed" << std::endl;
        return 1;
    }

    // Create your state estimator node
    auto node = std::make_shared<state_estimator::state_estimator_node>(lcm);

    // Initialize plugins (if needed)
    node->init_plugins();

    std::cout << "Starting state estimator (LCM)" << std::endl;

    // Main loop
    while (!g_request_shutdown) {
        // Handle incoming LCM messages (non-blocking)
        lcm.handleTimeout(1);  // timeout in ms

        // Allow your node to do periodic work if needed
        node->spin_some();

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    node->shutdown();
    return 0;
}
