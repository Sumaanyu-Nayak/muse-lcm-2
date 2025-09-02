#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

// Simple structures for demonstration (without external dependencies)
struct Vector3 {
    double x, y, z;
    
    Vector3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(double s) const { return Vector3(x * s, y * s, z * s); }
    Vector3 cross(const Vector3& v) const { return Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
    double dot(const Vector3& v) const { return x*v.x + y*v.y + z*v.z; }
    double norm() const { return sqrt(x*x + y*y + z*z); }
    Vector3 normalized() const { double n = norm(); return n > 0 ? (*this) * (1.0/n) : Vector3(); }
};

struct Quaternion {
    double w, x, y, z;
    
    Quaternion(double w = 1, double x = 0, double y = 0, double z = 0) : w(w), x(x), y(y), z(z) {}
    
    void normalize() {
        double n = sqrt(w*w + x*x + y*y + z*z);
        if (n > 0) {
            w /= n; x /= n; y /= n; z /= n;
        }
    }
    
    // Convert to rotation matrix (simplified)
    void toRotationMatrix(double R[9]) const {
        double w2 = w*w, x2 = x*x, y2 = y*y, z2 = z*z;
        double wx = w*x, wy = w*y, wz = w*z;
        double xy = x*y, xz = x*z, yz = y*z;
        
        R[0] = w2 + x2 - y2 - z2; R[1] = 2*(xy - wz);     R[2] = 2*(xz + wy);
        R[3] = 2*(xy + wz);       R[4] = w2 - x2 + y2 - z2; R[5] = 2*(yz - wx);
        R[6] = 2*(xz - wy);       R[7] = 2*(yz + wx);     R[8] = w2 - x2 - y2 + z2;
    }
};

// Simplified joint and foot state
struct JointState {
    double positions[12];   // Assuming 12 DOF quadruped (3 per leg)
    double velocities[12];
    int num_joints;
};

struct ContactState {
    bool stance_lf, stance_rf, stance_lh, stance_rh;
};

struct LegOdometryMessage {
    double timestamp;
    Vector3 lin_vel_lf, lin_vel_rf, lin_vel_lh, lin_vel_rh;
    Vector3 base_velocity;
    ContactState contact;
};

class SimpleLegOdometryEstimator {
public:
    SimpleLegOdometryEstimator() : sequence_id_(0) {
        // Initialize simplified leg parameters for demonstration
        // These would normally come from URDF/robot model
        
        // Simplified leg lengths (hip-to-knee, knee-to-foot)
        link1_length_ = 0.25;  // Upper leg
        link2_length_ = 0.25;  // Lower leg
        
        // Hip positions relative to body center (approximate for quadruped)
        hip_positions_[0] = Vector3( 0.2,  0.15, 0);  // LF
        hip_positions_[1] = Vector3( 0.2, -0.15, 0);  // RF  
        hip_positions_[2] = Vector3(-0.2,  0.15, 0);  // LH
        hip_positions_[3] = Vector3(-0.2, -0.15, 0);  // RH
        
        std::cout << "Simple Leg Odometry Estimator initialized" << std::endl;
        std::cout << "Using simplified kinematic model" << std::endl;
    }
    
    void processData(const JointState& joints, const Vector3& omega, 
                    const Quaternion& attitude, const ContactState& contact) {
        
        if (joints.num_joints != 12) {
            std::cerr << "Expected 12 joints, got " << joints.num_joints << std::endl;
            return;
        }
        
        std::vector<Vector3> foot_velocities;
        
        // Compute foot velocities using simplified kinematics
        for (int leg = 0; leg < 4; leg++) {
            Vector3 foot_vel = computeFootVelocity(leg, joints, omega);
            foot_velocities.push_back(foot_vel);
        }
        
        // Compute base velocity from stance legs
        double sum_stance = contact.stance_lf + contact.stance_rf + contact.stance_lh + contact.stance_rh;
        
        Vector3 base_velocity = (contact.stance_lf * foot_velocities[0] + 
                               contact.stance_rf * foot_velocities[1] + 
                               contact.stance_lh * foot_velocities[2] + 
                               contact.stance_rh * foot_velocities[3]) * (1.0 / (sum_stance + 1e-5));
        
        // Transform to world frame using attitude
        Vector3 base_velocity_world = transformToWorld(base_velocity, attitude);
        
        // Create and publish message
        LegOdometryMessage msg;
        msg.timestamp = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        msg.lin_vel_lf = foot_velocities[0];
        msg.lin_vel_rf = foot_velocities[1];
        msg.lin_vel_lh = foot_velocities[2];
        msg.lin_vel_rh = foot_velocities[3];
        msg.base_velocity = base_velocity_world;
        msg.contact = contact;
        
        publishLegOdometry(msg);
    }
    
private:
    Vector3 computeFootVelocity(int leg_index, const JointState& joints, const Vector3& omega) {
        // Simplified 2-DOF leg kinematics (hip pitch, knee pitch)
        // In reality, this would use full 3-DOF with proper coordinate frames
        
        int joint_offset = leg_index * 3;  // 3 joints per leg
        
        // Get joint angles and velocities (simplified to 2-DOF)
        double hip_angle = joints.positions[joint_offset + 1];    // Hip pitch
        double knee_angle = joints.positions[joint_offset + 2];   // Knee pitch
        double hip_vel = joints.velocities[joint_offset + 1];
        double knee_vel = joints.velocities[joint_offset + 2];
        
        // Forward kinematics (simplified)
        double c1 = cos(hip_angle);
        double s1 = sin(hip_angle);
        double c12 = cos(hip_angle + knee_angle);
        double s12 = sin(hip_angle + knee_angle);
        
        // Foot position relative to hip
        Vector3 foot_pos_rel;
        foot_pos_rel.x = link1_length_ * c1 + link2_length_ * c12;
        foot_pos_rel.y = 0;  // Simplified (no hip abduction)
        foot_pos_rel.z = -link1_length_ * s1 - link2_length_ * s12;
        
        // Jacobian (simplified 2-DOF)
        Vector3 J_col1, J_col2;
        J_col1.x = -link1_length_ * s1 - link2_length_ * s12;
        J_col1.y = 0;
        J_col1.z = -link1_length_ * c1 - link2_length_ * c12;
        
        J_col2.x = -link2_length_ * s12;
        J_col2.y = 0;
        J_col2.z = -link2_length_ * c12;
        
        // Foot velocity from joint motion
        Vector3 foot_vel_joint = J_col1 * hip_vel + J_col2 * knee_vel;
        
        // Add hip position to get foot position in body frame
        Vector3 foot_pos_body = hip_positions_[leg_index] + foot_pos_rel;
        
        // Velocity contribution from body angular motion: ω x r
        Vector3 omega_cross_r = omega.cross(foot_pos_body);
        
        // Total foot velocity relative to body
        Vector3 foot_vel_total = -(foot_vel_joint + omega_cross_r);
        
        return foot_vel_total;
    }
    
    Vector3 transformToWorld(const Vector3& vec_body, const Quaternion& attitude) {
        // Transform vector from body frame to world frame using quaternion
        // This is a simplified transformation
        double R[9];
        attitude.toRotationMatrix(R);
        
        Vector3 vec_world;
        vec_world.x = R[0] * vec_body.x + R[1] * vec_body.y + R[2] * vec_body.z;
        vec_world.y = R[3] * vec_body.x + R[4] * vec_body.y + R[5] * vec_body.z;
        vec_world.z = R[6] * vec_body.x + R[7] * vec_body.y + R[8] * vec_body.z;
        
        return vec_world;
    }
    
    void publishLegOdometry(const LegOdometryMessage& msg) {
        // Simulate message publishing (in real implementation, this would use LCM)
        static int count = 0;
        if (++count % 20 == 0) {  // Print every 20 messages
            std::cout << "Leg Odometry [" << ++sequence_id_ << "]: "
                      << "Base vel=[" << msg.base_velocity.x << ", " 
                      << msg.base_velocity.y << ", " << msg.base_velocity.z << "] "
                      << "Stance=[" << msg.contact.stance_lf << "," << msg.contact.stance_rf 
                      << "," << msg.contact.stance_lh << "," << msg.contact.stance_rh << "]" 
                      << std::endl;
        }
    }

private:
    // Robot model parameters
    double link1_length_, link2_length_;
    Vector3 hip_positions_[4];  // LF, RF, LH, RH
    
    int sequence_id_;
};

// Simulate robot data
void simulateRobotData(SimpleLegOdometryEstimator& estimator) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    double t = 0;
    double dt = 0.01;  // 100Hz
    
    std::cout << "Starting leg odometry simulation..." << std::endl;
    
    while (t < 10.0) {  // Run for 10 seconds
        // Simulate joint states (walking gait)
        JointState joints;
        joints.num_joints = 12;
        
        for (int i = 0; i < 12; i++) {
            // Simulate sinusoidal joint motion
            joints.positions[i] = 0.3 * sin(2.0 * t + i * 0.5);
            joints.velocities[i] = 0.6 * cos(2.0 * t + i * 0.5);
        }
        
        // Simulate angular velocity
        Vector3 omega(0.05 * sin(0.5 * t), 0.05 * cos(0.3 * t), 0.02 * sin(0.2 * t));
        
        // Simulate attitude (slow rotation)
        Quaternion attitude;
        attitude.w = cos(0.1 * t);
        attitude.x = 0.1 * sin(0.1 * t);
        attitude.y = 0.1 * cos(0.1 * t);
        attitude.z = 0.05 * sin(0.1 * t);
        attitude.normalize();
        
        // Simulate contact pattern (trotting gait)
        ContactState contact;
        bool phase = fmod(t, 1.0) < 0.5;  // 1 second gait cycle
        contact.stance_lf = phase;
        contact.stance_rh = phase;
        contact.stance_rf = !phase;
        contact.stance_lh = !phase;
        
        estimator.processData(joints, omega, attitude, contact);
        
        t += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz
    }
}

int main() {
    std::cout << "=== Simple Leg Odometry Estimator ===" << std::endl;
    std::cout << "This is a simplified implementation without Pinocchio dependencies." << std::endl;
    std::cout << "It uses basic kinematic calculations for demonstration." << std::endl << std::endl;
    
    SimpleLegOdometryEstimator estimator;
    
    // Simulate robot data
    simulateRobotData(estimator);
    
    std::cout << "Simulation complete." << std::endl;
    return 0;
}
