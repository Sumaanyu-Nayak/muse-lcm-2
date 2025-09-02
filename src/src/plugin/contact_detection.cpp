#include <lcm/lcm-cpp.hpp>
#include "state_estimator_msgs/contact_detection_t.hpp"   // Generated LCM type
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>

int main(int argc, char ** argv)
{

int sequence_id = 0;

    lcm::LCM lcm;
    if (!lcm.good())
        return 1;

    auto start = std::chrono::steady_clock::now();

    while (true)
    {
        auto now = std::chrono::system_clock::now();
        auto now_us = std::chrono::time_point_cast<std::chrono::microseconds>(now);
        auto epoch_us = now_us.time_since_epoch().count();
        contact_detection_t msg;

        // Fill header
        msg.head.sequence_id = sequence_id++;
        msg.head.sec  = static_cast<int32_t>(epoch_us / 1000000);  // seconds
        msg.head.nsec = static_cast<int32_t>((epoch_us % 1000000) * 1000); // nanoseconds
        msg.head.frame_name = "base"; 

        // Publish all stance as true
        msg.stance_lf = true;
        msg.stance_rf = true;
        msg.stance_lh = true;
        msg.stance_rh = true;

        lcm.publish("/state_estimator/contact_detection", &msg);
        std::cout << "Published CONTACT_DETECTION with all stances = true at time: " 
                   << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 Hz
    }

    return 0;
}
