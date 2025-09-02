#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

#include "../msgs/state_estimator_msgs/imu_t.hpp"
#include "../msgs/state_estimator_msgs/joint_state_with_acceleration_t.hpp"
#include "../msgs/state_estimator_msgs/attitude_t.hpp"
#include "../msgs/state_estimator_msgs/contact_detection_t.hpp"
#include "../msgs/state_estimator_msgs/leg_odometry_t.hpp"

// Global latest data
imu_t latest_imu;
joint_state_with_acceleration_t latest_joints;
attitude_t latest_att;
contact_detection_t latest_contact;

bool imu_ready=false, joints_ready=false, att_ready=false, contact_ready=false;

class Handler {
public:
    lcm::LCM* lcm;

    void handleIMU(const lcm::ReceiveBuffer*, const std::string&, const imu_t* msg) {
        latest_imu = *msg; imu_ready=true;
        try_compute();
    }
    void handleJoints(const lcm::ReceiveBuffer*, const std::string&, const joint_state_with_acceleration_t* msg) {
        latest_joints = *msg; joints_ready=true;
        try_compute();
    }
    void handleAtt(const lcm::ReceiveBuffer*, const std::string&, const attitude_t* msg) {
        latest_att = *msg; att_ready=true;
        try_compute();
    }
    void handleContact(const lcm::ReceiveBuffer*, const std::string&, const contact_detection_t* msg) {
        latest_contact = *msg; contact_ready=true;
        try_compute();
    }

    void try_compute() {
        if(!(imu_ready && joints_ready && att_ready && contact_ready)) return;

        // Convert quaternion to rotation matrix
        Eigen::Quaterniond q(
            latest_att.quaternion[3], // w
            latest_att.quaternion[0], // x
            latest_att.quaternion[1], // y
            latest_att.quaternion[2]  // z
        );
        Eigen::Matrix3d w_R_b = q.toRotationMatrix();

        // Dummy kinematics: base velocity = average of stance legs
        Eigen::Vector3d base_vel(0,0,0);
        int stance_count = 0;

        auto add_leg = [&](bool stance, const Eigen::Vector3d& v){
            if(stance){ base_vel += v; stance_count++; }
        };

        add_leg(latest_contact.stance_lf, Eigen::Vector3d(0,0,0));
        add_leg(latest_contact.stance_rf, Eigen::Vector3d(0,0,0));
        add_leg(latest_contact.stance_lh, Eigen::Vector3d(0,0,0));
        add_leg(latest_contact.stance_rh, Eigen::Vector3d(0,0,0));

        if(stance_count > 0) base_vel /= stance_count;

        // Transform to world frame
        Eigen::Vector3d base_vel_world = w_R_b * base_vel;

        // Fill odometry msg
        leg_odometry_t odom;
        odom.head = latest_imu.head;

        for(int i=0;i<3;i++){
            odom.lin_vel_lf[i] = 0;
            odom.lin_vel_rf[i] = 0;
            odom.lin_vel_lh[i] = 0;
            odom.lin_vel_rh[i] = 0;
        }

        odom.base_velocity[0] = base_vel_world.x();
        odom.base_velocity[1] = base_vel_world.y();
        odom.base_velocity[2] = base_vel_world.z();

        lcm->publish("LEG_ODOMETRY", &odom);
        std::cout << "Published odometry: "
                  << odom.base_velocity[0] << ", "
                  << odom.base_velocity[1] << ", "
                  << odom.base_velocity[2] << std::endl;
    }
};

int main(int argc, char** argv) {
    lcm::LCM lcm;
    if(!lcm.good()) return 1;

    Handler handler;
    handler.lcm = &lcm;

    lcm.subscribe("/sensors/imu", &Handler::handleIMU, &handler);
    lcm.subscribe("/state_estimator/joint_states", &Handler::handleJoints, &handler);
    lcm.subscribe("/state_estimator/attitude", &Handler::handleAtt, &handler);
    lcm.subscribe("/state_estimator/contact_detection", &Handler::handleContact, &handler);

    while(0 == lcm.handle());
    return 0;
}
