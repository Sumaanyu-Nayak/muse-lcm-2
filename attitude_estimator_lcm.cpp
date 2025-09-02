#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <chrono>
#include <cmath>
#include <functional>

// Include LCM message types
#include "msgs/state_estimator_msgs/imu_t.hpp"
#include "msgs/state_estimator_msgs/attitude_t.hpp"
#include "msgs/state_estimator_msgs/header_t.hpp"

// Include the attitude estimation model
#include "include/state_estimator/Models/attitude_bias_XKF.hpp"
#include "include/state_estimator/lib.hpp"

// Eigen includes (assuming they're available)
#include <Eigen/Dense>
#include <Eigen/Geometry>

class AttitudeEstimatorLCM {
public:
    AttitudeEstimatorLCM() : 
        lcm_("udp://239.255.76.67:7667?ttl=1"),
        attitude_(nullptr),
        begin_(true),
        sequence_id_(0)
    {
        initialize();
        
        // Subscribe to IMU messages
        lcm_.subscribe("imu", &AttitudeEstimatorLCM::handleIMU, this);
        
        std::cout << "Attitude Estimator LCM initialized" << std::endl;
        std::cout << "Subscribing to 'imu' channel" << std::endl;
        std::cout << "Publishing to 'attitude' channel" << std::endl;
    }
    
    ~AttitudeEstimatorLCM() {
        if (attitude_ != nullptr) {
            delete attitude_;
        }
    }
    
    void run() {
        std::cout << "Starting attitude estimation loop..." << std::endl;
        while (true) {
            if (lcm_.handle() != 0) {
                std::cerr << "Error: LCM handle failed" << std::endl;
                break;
            }
        }
    }

private:
    void initialize() {
        // Initialize parameters (same as original plugin)
        t0_ = 0.0;
        xhat_estimated_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        xhat_estimated_.head(4) = xhat_estimated_.head(4) / xhat_estimated_.head(4).norm();

        // Set parameters (hardcoded for simplicity)
        ki_ = 0.02;
        kp_ = 10.0;
        
        // Base to IMU rotation matrix
        b_R_imu_ << -1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, -1.0;
        
        // North vector (magnetic field reference)
        m_n_ << 0.577, 0.577, 0.577;
        m_n_.normalize();
        
        // Gravity vector
        f_n_ << 0.0, 0.0, 9.81;
        f_n_.normalize();
        
        // Initialize covariance matrices
        Eigen::Matrix6d P0, Q, R;
        
        // P matrix (initial covariance)
        P0.setIdentity();
        P0 *= 1e-6;
        
        // Q matrix (process noise)
        Q.setIdentity();
        Q.block(0, 0, 3, 3) *= 1e-6;  // Attitude process noise
        Q.block(3, 3, 3, 3) *= 1e-8;  // Bias process noise
        
        // R matrix (measurement noise)
        R.setIdentity();
        R.block(0, 0, 3, 3) *= 4e-2;  // Accelerometer noise
        R.block(3, 3, 3, 3) *= 16.0;  // Magnetometer noise
        
        // Create the attitude estimator
        attitude_ = new state_estimator::AttitudeBiasXKF(t0_, xhat_estimated_, P0, Q, R, f_n_, m_n_, ki_, kp_);
        
        std::cout << "Attitude estimator initialized with:" << std::endl;
        std::cout << "  ki = " << ki_ << std::endl;
        std::cout << "  kp = " << kp_ << std::endl;
        std::cout << "  North vector: [" << m_n_.transpose() << "]" << std::endl;
        std::cout << "  Gravity vector: [" << f_n_.transpose() << "]" << std::endl;
    }
    
    void handleIMU(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const imu_t* msg) {
        // Extract angular velocity and linear acceleration
        Eigen::Vector3d omega(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        
        // Compute attitude
        computeAttitude(omega, acc, *msg);
    }
    
    void computeAttitude(Eigen::Vector3d &omega, Eigen::Vector3d &acc, const imu_t& imu_msg) {
        // Initialize timing on first message
        if (begin_) {
            time_begin_ = std::chrono::high_resolution_clock::now();
            begin_ = false;
        }

        auto current_time = std::chrono::high_resolution_clock::now();
        time_ = std::chrono::duration<double>(current_time - time_begin_).count();
        
        // Get current quaternion estimate
        quat_est_.w() = xhat_estimated_(0);
        quat_est_.vec() << xhat_estimated_(1), xhat_estimated_(2), xhat_estimated_(3);

        // Transform measurements to body frame
        f_b_ = b_R_imu_ * acc;
        m_b_ = quat_est_.toRotationMatrix() * m_n_;

        // Prepare measurement vector
        z_ << f_b_, m_b_;
        
        // Update the attitude estimator
        attitude_->update(time_, b_R_imu_ * omega, z_);

        // Get updated state estimate
        xhat_estimated_ = attitude_->getX();
        
        // Calculate quaternion derivative for angular velocity estimation
        xdot_ = attitude_->calc_f(time_, xhat_estimated_, b_R_imu_ * omega);
        quat_dot_.w() = xdot_(0);
        quat_dot_.vec() << xdot_(1), xdot_(2), xdot_(3);
        
        // Calculate filtered angular velocity using quaternion derivative
        Eigen::Quaterniond quat_conj = quat_est_.conjugate();
        Eigen::Quaterniond temp = quat_conj * quat_dot_;
        omega_filt_ << temp.x(), temp.y(), temp.z();
        omega_filt_ *= 2.0;
        
        // Publish attitude
        publishAttitude(imu_msg);
    }
    
    void publishAttitude(const imu_t& imu_msg) {
        attitude_t attitude_msg;
        
        // Copy header from IMU message and update sequence
        attitude_msg.head = imu_msg.head;
        attitude_msg.head.sequence_id = ++sequence_id_;
        attitude_msg.head.frame_name = "body";
        
        // Update quaternion estimate
        quat_est_.w() = xhat_estimated_(0);
        quat_est_.vec() << xhat_estimated_(1), xhat_estimated_(2), xhat_estimated_(3);
        
        // Quaternion (w, x, y, z)
        attitude_msg.quaternion[0] = quat_est_.w();
        attitude_msg.quaternion[1] = quat_est_.x();
        attitude_msg.quaternion[2] = quat_est_.y();
        attitude_msg.quaternion[3] = quat_est_.z();

        // Convert quaternion to Euler angles (Roll, Pitch, Yaw)
        double w = quat_est_.w();
        double x = quat_est_.x();
        double y = quat_est_.y();
        double z = quat_est_.z();
        
        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        euler_radians_(0) = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            euler_radians_(1) = std::copysign(M_PI / 2, sinp);
        else
            euler_radians_(1) = std::asin(sinp);
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        euler_radians_(2) = std::atan2(siny_cosp, cosy_cosp);
        
        euler_degrees_ = euler_radians_ * (180.0 / M_PI);
        
        attitude_msg.roll_deg = euler_degrees_(0);
        attitude_msg.pitch_deg = euler_degrees_(1);
        attitude_msg.yaw_deg = euler_degrees_(2);

        attitude_msg.angular_velocity[0] = omega_filt_(0);
        attitude_msg.angular_velocity[1] = omega_filt_(1);
        attitude_msg.angular_velocity[2] = omega_filt_(2);

        // Publish the message
        lcm_.publish("attitude", &attitude_msg);

        // Debug output
        if (sequence_id_ % 100 == 0) {  // Print every 100 messages
            std::cout << "Attitude [" << sequence_id_ << "]: "
                      << "Roll=" << attitude_msg.roll_deg << "° "
                      << "Pitch=" << attitude_msg.pitch_deg << "° "
                      << "Yaw=" << attitude_msg.yaw_deg << "°" << std::endl;
        }
    }

private:
    // LCM communication
    lcm::LCM lcm_;
    
    // Attitude estimator
    state_estimator::AttitudeBiasXKF* attitude_;
    
    // State variables
    Eigen::Matrix<double, 7, 1> xhat_estimated_;
    Eigen::Matrix<double, 3, 1> f_n_;  // Gravity vector in navigation frame
    Eigen::Matrix<double, 3, 1> m_n_;  // Magnetic field vector in navigation frame
    Eigen::Matrix3d b_R_imu_;          // Body to IMU rotation matrix
    double ki_;                        // Integral gain
    double kp_;                        // Proportional gain
    double t0_;                        // Initial time
    
    // Working variables
    Eigen::Quaterniond quat_est_;
    Eigen::Vector3d f_b_;              // Gravity in body frame
    Eigen::Vector3d m_b_;              // Magnetic field in body frame
    Eigen::Vector6d z_;                // Measurement vector
    Eigen::Vector7d xdot_;             // State derivative
    Eigen::Quaterniond quat_dot_;      // Quaternion derivative
    Eigen::Vector3d omega_filt_;       // Filtered angular velocity
    Eigen::Vector3d euler_radians_;
    Eigen::Vector3d euler_degrees_;
    
    // Timing
    double time_;
    bool begin_;
    std::chrono::high_resolution_clock::time_point time_begin_;
    int32_t sequence_id_;
};

int main(int argc, char** argv) {
    std::cout << "=== Attitude Estimator LCM ===" << std::endl;
    std::cout << "Converting ROS2 plugin to standalone LCM implementation" << std::endl;
    std::cout << "Using AttitudeBiasXKF filter" << std::endl << std::endl;
    
    try {
        AttitudeEstimatorLCM estimator;
        estimator.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
