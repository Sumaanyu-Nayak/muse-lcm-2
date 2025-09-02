#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

// Simple vector and quaternion structures
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
    
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }
    
    Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }
    
    double norm() const { return sqrt(w*w + x*x + y*y + z*z); }
    
    void normalize() {
        double n = norm();
        if (n > 0) {
            w /= n; x /= n; y /= n; z /= n;
        }
    }
    
    Vector3 rotate(const Vector3& v) const {
        // q * [0, v] * q^-1
        Quaternion qv(0, v.x, v.y, v.z);
        Quaternion result = (*this) * qv * conjugate();
        return Vector3(result.x, result.y, result.z);
    }
    
    Vector3 toEuler() const {
        Vector3 euler;
        
        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        euler.x = atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (fabs(sinp) >= 1)
            euler.y = copysign(M_PI / 2, sinp);
        else
            euler.y = asin(sinp);
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        euler.z = atan2(siny_cosp, cosy_cosp);
        
        return euler;
    }
};

// LCM-like message structures (simplified)
struct IMUMessage {
    double timestamp;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;
};

struct AttitudeMessage {
    double timestamp;
    Quaternion quaternion;
    Vector3 euler_degrees;
    Vector3 angular_velocity;
};

class SimpleAttitudeEstimator {
public:
    SimpleAttitudeEstimator() : 
        attitude_(1, 0, 0, 0),
        last_time_(0),
        alpha_(0.98),  // Complementary filter coefficient
        initialized_(false) 
    {
        std::cout << "Simple Attitude Estimator initialized" << std::endl;
        std::cout << "Using complementary filter with alpha = " << alpha_ << std::endl;
    }
    
    void processIMU(const IMUMessage& imu) {
        double current_time = imu.timestamp;
        
        if (!initialized_) {
            last_time_ = current_time;
            
            // Initialize attitude from accelerometer
            Vector3 acc_norm = imu.linear_acceleration.normalized();
            Vector3 gravity(0, 0, -1);  // Assuming Z-up convention
            
            // Calculate initial roll and pitch from accelerometer
            double roll = atan2(acc_norm.y, acc_norm.z);
            double pitch = atan2(-acc_norm.x, sqrt(acc_norm.y*acc_norm.y + acc_norm.z*acc_norm.z));
            
            // Create initial quaternion (yaw = 0)
            attitude_ = eulerToQuaternion(Vector3(roll, pitch, 0));
            
            initialized_ = true;
            std::cout << "Attitude estimator initialized with roll=" << roll*180/M_PI 
                      << "°, pitch=" << pitch*180/M_PI << "°" << std::endl;
            return;
        }
        
        double dt = current_time - last_time_;
        if (dt <= 0 || dt > 1.0) {  // Invalid dt
            last_time_ = current_time;
            return;
        }
        
        // === PREDICTION STEP (using gyroscope) ===
        Vector3 omega = imu.angular_velocity;
        
        // Integrate quaternion using gyroscope
        Vector3 omega_half = omega * (0.5 * dt);
        double omega_norm = omega_half.norm();
        
        Quaternion q_gyro(1, 0, 0, 0);
        if (omega_norm > 1e-8) {
            q_gyro.w = cos(omega_norm);
            double sin_norm = sin(omega_norm) / omega_norm;
            q_gyro.x = omega_half.x * sin_norm;
            q_gyro.y = omega_half.y * sin_norm;
            q_gyro.z = omega_half.z * sin_norm;
        }
        
        // Predict attitude
        Quaternion attitude_pred = attitude_ * q_gyro;
        attitude_pred.normalize();
        
        // === CORRECTION STEP (using accelerometer) ===
        Vector3 acc_measured = imu.linear_acceleration.normalized();
        Vector3 gravity_expected(0, 0, -1);  // Expected gravity in world frame
        Vector3 gravity_body = attitude_pred.conjugate().rotate(gravity_expected);
        
        // Calculate attitude from accelerometer
        double roll_acc = atan2(acc_measured.y, acc_measured.z);
        double pitch_acc = atan2(-acc_measured.x, sqrt(acc_measured.y*acc_measured.y + acc_measured.z*acc_measured.z));
        
        Quaternion attitude_acc = eulerToQuaternion(Vector3(roll_acc, pitch_acc, attitude_.toEuler().z));
        
        // === COMPLEMENTARY FILTER ===
        // Interpolate between gyroscope prediction and accelerometer measurement
        attitude_ = slerp(attitude_acc, attitude_pred, alpha_);
        attitude_.normalize();
        
        last_time_ = current_time;
        
        // Publish attitude
        publishAttitude(imu.timestamp, omega);
    }
    
private:
    Quaternion eulerToQuaternion(const Vector3& euler) {
        double cr = cos(euler.x * 0.5);
        double sr = sin(euler.x * 0.5);
        double cp = cos(euler.y * 0.5);
        double sp = sin(euler.y * 0.5);
        double cy = cos(euler.z * 0.5);
        double sy = sin(euler.z * 0.5);
        
        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }
    
    // Spherical linear interpolation
    Quaternion slerp(const Quaternion& q1, const Quaternion& q2, double t) {
        double dot_product = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
        
        // If dot product is negative, use -q1 to ensure shortest path
        Quaternion qa = q1;
        if (dot_product < 0) {
            qa.w = -qa.w; qa.x = -qa.x; qa.y = -qa.y; qa.z = -qa.z;
            dot_product = -dot_product;
        }
        
        if (dot_product > 0.9995) {
            // Linear interpolation for very close quaternions
            Quaternion result(
                qa.w + t * (q2.w - qa.w),
                qa.x + t * (q2.x - qa.x),
                qa.y + t * (q2.y - qa.y),
                qa.z + t * (q2.z - qa.z)
            );
            result.normalize();
            return result;
        }
        
        double theta = acos(fabs(dot_product));
        double sin_theta = sin(theta);
        double w1 = sin((1-t) * theta) / sin_theta;
        double w2 = sin(t * theta) / sin_theta;
        
        return Quaternion(
            w1 * qa.w + w2 * q2.w,
            w1 * qa.x + w2 * q2.x,
            w1 * qa.y + w2 * q2.y,
            w1 * qa.z + w2 * q2.z
        );
    }
    
    void publishAttitude(double timestamp, const Vector3& omega) {
        AttitudeMessage attitude_msg;
        attitude_msg.timestamp = timestamp;
        attitude_msg.quaternion = attitude_;
        
        Vector3 euler_rad = attitude_.toEuler();
        attitude_msg.euler_degrees = Vector3(
            euler_rad.x * 180.0 / M_PI,
            euler_rad.y * 180.0 / M_PI,
            euler_rad.z * 180.0 / M_PI
        );
        
        attitude_msg.angular_velocity = omega;
        
        // Print attitude (simulate LCM publish)
        static int count = 0;
        if (++count % 50 == 0) {  // Print every 50 messages
            std::cout << "Attitude: Roll=" << attitude_msg.euler_degrees.x 
                      << "°, Pitch=" << attitude_msg.euler_degrees.y 
                      << "°, Yaw=" << attitude_msg.euler_degrees.z << "°" << std::endl;
        }
    }

private:
    Quaternion attitude_;
    double last_time_;
    double alpha_;  // Complementary filter coefficient
    bool initialized_;
};

// Simulate IMU data
void simulateIMU(SimpleAttitudeEstimator& estimator) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    double t = 0;
    double dt = 0.01;  // 100Hz
    
    std::cout << "Starting IMU simulation..." << std::endl;
    
    while (t < 10.0) {  // Run for 10 seconds
        auto current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);
        double timestamp = duration.count() / 1000000.0;
        
        // Simulate IMU data with some motion
        IMUMessage imu;
        imu.timestamp = timestamp;
        
        // Simulate some angular velocity (slow rotation)
        imu.angular_velocity = Vector3(
            0.1 * sin(0.5 * t),  // Roll rate
            0.1 * cos(0.3 * t),  // Pitch rate
            0.05 * sin(0.2 * t)  // Yaw rate
        );
        
        // Simulate accelerometer with gravity + some noise
        imu.linear_acceleration = Vector3(
            0.1 * sin(0.8 * t),      // Some lateral acceleration
            0.1 * cos(0.6 * t),      // Some lateral acceleration
            -9.81 + 0.2 * sin(1.2 * t)  // Gravity + vertical acceleration
        );
        
        estimator.processIMU(imu);
        
        t += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz
    }
}

int main() {
    std::cout << "=== Simple Attitude Estimator ===" << std::endl;
    std::cout << "This is a simplified implementation without plugins or complex dependencies." << std::endl;
    std::cout << "It uses a complementary filter to estimate attitude from IMU data." << std::endl << std::endl;
    
    SimpleAttitudeEstimator estimator;
    
    // In a real application, you would subscribe to LCM messages here
    // For demonstration, we'll simulate IMU data
    simulateIMU(estimator);
    
    std::cout << "Simulation complete." << std::endl;
    return 0;
}
