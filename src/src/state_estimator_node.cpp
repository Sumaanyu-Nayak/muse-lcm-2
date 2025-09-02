#include <iostream>
#include <thread>
#include <vector>
#include <cstdlib>   // std::system
#include <string>

void run_process(const std::string &cmd) {
    std::cout << "Starting: " << cmd << std::endl;
    int rc = std::system(cmd.c_str()); // blocks this thread until process finishes
    std::cout << "Finished: " << cmd << " (exit code " << rc << ")\n";
}

int main() {
    // Adjust these to the paths/arguments you need
    std::vector<std::string> cmds = {
        "./attitude_node", 
        "./contact_detection_node", 
        "./leg_odometry_node"
        // "./sensor_fusion_node"
    };

    std::vector<std::thread> threads;
    threads.reserve(cmds.size());

    for (const auto &c : cmds) {
        threads.emplace_back(run_process, c);
    }

    // Join all threads (wait until each child process ends)
    for (auto &t : threads) {
        if (t.joinable()) t.join();
    }

    std::cout << "All processes finished.\n";
    return 0;
}
