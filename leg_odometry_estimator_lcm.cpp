#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// Pinocchio includes
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Include LCM message types
#include "msgs/state_estimator_msgs/imu_t.hpp"
#include "msgs/state_estimator_msgs/joint_state_with_acceleration_t.hpp"
#include "msgs/state_estimator_msgs/contact_detection_t.hpp"
#include "msgs/state_estimator_msgs/attitude_t.hpp"
#include "msgs/state_estimator_msgs/leg_odometry_t.hpp"
#include "msgs/state_estimator_msgs/header_t.hpp"

class LegOdometryEstimatorLCM {
public:
    LegOdometryEstimatorLCM() : 
        lcm_("udp://239.255.76.67:7667?ttl=1"),
        sequence_id_(0),
        model_loaded_(false)
    {
        initialize();
        
        // Subscribe to required messages
        lcm_.subscribe("imu", &LegOdometryEstimatorLCM::handleIMU, this);
        lcm_.subscribe("joint_states", &LegOdometryEstimatorLCM::handleJointStates, this);
        lcm_.subscribe("contact_detection", &LegOdometryEstimatorLCM::handleContactDetection, this);
        lcm_.subscribe("attitude", &LegOdometryEstimatorLCM::handleAttitude, this);
        
        std::cout << "Leg Odometry Estimator LCM initialized" << std::endl;
        std::cout << "Subscribing to: imu, joint_states, contact_detection, attitude" << std::endl;
        std::cout << "Publishing to: leg_odometry" << std::endl;
    }
    
    ~LegOdometryEstimatorLCM() = default;
    
    void run() {
        std::cout << "Starting leg odometry estimation loop..." << std::endl;
        while (true) {
            if (lcm_.handle() != 0) {
                std::cerr << "Error: LCM handle failed" << std::endl;
                break;
            }
        }
    }

private:
    void initialize() {
        // Initialize parameters (hardcoded for simplicity)
        std::string urdf_path = "src/urdfs/aliengo.urdf";  // Relative to workspace
        
        // Base to IMU rotation matrix
        base_R_imu_ << -1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, -1.0;
        
        // Foot frame names (update based on your robot)
        feet_frame_names_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
        // Alternative for Aliengo: {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
        
        // Load URDF model
        try {
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = pinocchio::Data(model_);
            model_loaded_ = true;
            std::cout << "URDF loaded into Pinocchio model successfully from: " << urdf_path << std::endl;
            std::cout << "Model has " << model_.nq << " positions and " << model_.nv << " velocities" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Failed to load URDF: " << e.what() << std::endl;
            std::cerr << "Trying alternative path..." << std::endl;
            
            // Try alternative path
            try {
                urdf_path = "src/urdfs/anymal.urdf";
                pinocchio::urdf::buildModel(urdf_path, model_);
                data_ = pinocchio::Data(model_);
                model_loaded_ = true;
                std::cout << "URDF loaded successfully from: " << urdf_path << std::endl;
            } catch (const std::exception& e2) {
                std::cerr << "Failed to load alternative URDF: " << e2.what() << std::endl;
                model_loaded_ = false;
            }
        }
        
        // Initialize message storage
        last_imu_time_ = 0.0;
        last_joint_time_ = 0.0;
        last_contact_time_ = 0.0;
        last_attitude_time_ = 0.0;
        
        // Initialize stance flags
        stance_lf_ = false;
        stance_rf_ = false;
        stance_lh_ = false;
        stance_rh_ = false;
    }
    
    void handleIMU(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const imu_t* msg) {
        latest_imu_ = *msg;
        last_imu_time_ = msg->head.sec + msg->head.nsec * 1e-9;
        
        // Extract angular velocity
        omega_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        
        // Try to compute leg odometry if we have all required data
        tryComputeLegOdometry();
    }
    
    void handleJointStates(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const joint_state_with_acceleration_t* msg) {
        latest_joint_states_ = *msg;
        last_joint_time_ = msg->head.sec + msg->head.nsec * 1e-9;
        
        // Try to compute leg odometry if we have all required data
        tryComputeLegOdometry();
    }
    
    void handleContactDetection(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const contact_detection_t* msg) {
        latest_contact_ = *msg;
        last_contact_time_ = msg->head.sec + msg->head.nsec * 1e-9;
        
        // Extract stance flags
        stance_lf_ = msg->stance_lf;
        stance_rf_ = msg->stance_rf;
        stance_lh_ = msg->stance_lh;
        stance_rh_ = msg->stance_rh;
        
        // Try to compute leg odometry if we have all required data
        tryComputeLegOdometry();
    }
    
    void handleAttitude(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const attitude_t* msg) {
        latest_attitude_ = *msg;
        last_attitude_time_ = msg->head.sec + msg->head.nsec * 1e-9;
        
        // Try to compute leg odometry if we have all required data
        tryComputeLegOdometry();
    }
    
    void tryComputeLegOdometry() {
        // Check if we have recent data from all sources
        double current_time = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        
        double max_age = 0.1; // 100ms
        
        if (!model_loaded_) {
            return;
        }
        
        if (current_time - last_imu_time_ > max_age ||
            current_time - last_joint_time_ > max_age ||
            current_time - last_contact_time_ > max_age ||
            current_time - last_attitude_time_ > max_age) {
            return; // Data too old
        }
        
        computeLegOdometry();
    }
    
    void computeLegOdometry() {
        // Check joint state size
        if (latest_joint_states_.no_of_joints != model_.nq) {
            std::cerr << "Warning: Mismatch in joint state size. Expected " << model_.nq 
                      << ", got " << latest_joint_states_.no_of_joints << std::endl;
            return;
        }
        
        // Fill joint positions and velocities
        Eigen::VectorXd q(model_.nq);
        Eigen::VectorXd v(model_.nv);
        
        for (int i = 0; i < latest_joint_states_.no_of_joints; ++i) {
            q[i] = latest_joint_states_.position[i];
            v[i] = latest_joint_states_.velocity[i];
        }
        
        // Compute forward kinematics
        pinocchio::forwardKinematics(model_, data_, q, v);
        pinocchio::updateFramePlacements(model_, data_);
        
        std::vector<Eigen::Vector3d> foot_velocities;
        
        // Compute foot velocities for each leg
        for (size_t i = 0; i < feet_frame_names_.size(); ++i) {
            const auto& foot_name = feet_frame_names_[i];
            
            try {
                std::size_t frame_id = model_.getFrameId(foot_name);
                
                // Get spatial velocity in LOCAL_WORLD_ALIGNED frame
                pinocchio::Motion foot_vel_global = pinocchio::getFrameVelocity(
                    model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
                
                // Get position of the foot in the base frame
                Eigen::Vector3d foot_pos_base = data_.oMf[frame_id].translation();
                Eigen::Vector3d omega_rotated = base_R_imu_ * omega_;
                
                // Compute velocity contribution from base angular motion: ω x r
                Eigen::Vector3d omega_cross_r = omega_rotated.cross(foot_pos_base);
                
                // Compute linear velocity of the foot relative to base
                Eigen::Vector3d rel_vel = -(foot_vel_global.linear() - omega_cross_r);
                foot_velocities.push_back(rel_vel);
                
            } catch (const std::exception& e) {
                std::cerr << "Error computing velocity for foot " << foot_name << ": " << e.what() << std::endl;
                foot_velocities.push_back(Eigen::Vector3d::Zero());
            }
        }
        
        // Ensure we have velocities for all 4 feet
        if (foot_velocities.size() != 4) {
            std::cerr << "Error: Expected 4 foot velocities, got " << foot_velocities.size() << std::endl;
            return;
        }
        
        Eigen::Vector3d lin_leg_lf = foot_velocities[0];  // Left Front
        Eigen::Vector3d lin_leg_rf = foot_velocities[1];  // Right Front
        Eigen::Vector3d lin_leg_lh = foot_velocities[2];  // Left Hind
        Eigen::Vector3d lin_leg_rh = foot_velocities[3];  // Right Hind
        
        // Compute weighted average base velocity based on stance
        double sum_stance = stance_lf_ + stance_rf_ + stance_lh_ + stance_rh_;
        Eigen::Vector3d base_velocity = (stance_lf_ * lin_leg_lf + 
                                       stance_rf_ * lin_leg_rf + 
                                       stance_lh_ * lin_leg_lh + 
                                       stance_rh_ * lin_leg_rh) / (sum_stance + 1e-5);
        
        // Transform base velocity to world frame using attitude
        Eigen::Quaterniond quat_est;
        quat_est.w() = latest_attitude_.quaternion[0];
        quat_est.vec() << latest_attitude_.quaternion[1], 
                          latest_attitude_.quaternion[2], 
                          latest_attitude_.quaternion[3];
        
        Eigen::Matrix3d w_R_b = quat_est.toRotationMatrix().transpose();
        base_velocity = w_R_b * base_velocity;
        
        // Publish leg odometry
        publishLegOdometry(lin_leg_lf, lin_leg_rf, lin_leg_lh, lin_leg_rh, base_velocity);
    }
    
    void publishLegOdometry(const Eigen::Vector3d& lin_vel_lf,
                           const Eigen::Vector3d& lin_vel_rf,
                           const Eigen::Vector3d& lin_vel_lh,
                           const Eigen::Vector3d& lin_vel_rh,
                           const Eigen::Vector3d& base_velocity) {
        
        leg_odometry_t leg_odom_msg;
        
        // Header
        leg_odom_msg.head.sequence_id = ++sequence_id_;
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
        
        leg_odom_msg.head.sec = seconds.count();
        leg_odom_msg.head.nsec = nanoseconds.count();
        leg_odom_msg.head.frame_name = "base_link";
        
        // Copy foot velocities
        for (int j = 0; j < 3; j++) {
            leg_odom_msg.lin_vel_lf[j] = lin_vel_lf[j];
            leg_odom_msg.lin_vel_rf[j] = lin_vel_rf[j];
            leg_odom_msg.lin_vel_lh[j] = lin_vel_lh[j];
            leg_odom_msg.lin_vel_rh[j] = lin_vel_rh[j];
            leg_odom_msg.base_velocity[j] = base_velocity[j];
        }
        
        // Publish message
        lcm_.publish("leg_odometry", &leg_odom_msg);
        
        // Debug output
        if (sequence_id_ % 50 == 0) {  // Print every 50 messages
            std::cout << "Leg Odometry [" << sequence_id_ << "]: "
                      << "Base vel=[" << base_velocity.transpose() << "] "
                      << "Stance=[" << stance_lf_ << "," << stance_rf_ 
                      << "," << stance_lh_ << "," << stance_rh_ << "]" << std::endl;
        }
    }

private:
    // LCM communication
    lcm::LCM lcm_;
    
    // Pinocchio model and data
    pinocchio::Model model_;
    pinocchio::Data data_;
    bool model_loaded_;
    
    // Robot configuration
    std::vector<std::string> feet_frame_names_;
    Eigen::Matrix3d base_R_imu_;
    
    // Latest received messages
    imu_t latest_imu_;
    joint_state_with_acceleration_t latest_joint_states_;
    contact_detection_t latest_contact_;
    attitude_t latest_attitude_;
    
    // Message timestamps
    double last_imu_time_;
    double last_joint_time_;
    double last_contact_time_;
    double last_attitude_time_;
    
    // Current state
    Eigen::Vector3d omega_;
    bool stance_lf_, stance_rf_, stance_lh_, stance_rh_;
    
    // Sequence tracking
    int32_t sequence_id_;
};

int main(int argc, char** argv) {
    std::cout << "=== Leg Odometry Estimator LCM ===" << std::endl;
    std::cout << "Converting ROS2 plugin to standalone LCM implementation" << std::endl;
    std::cout << "Using Pinocchio for kinematics computation" << std::endl << std::endl;
    
    try {
        LegOdometryEstimatorLCM estimator;
        estimator.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
