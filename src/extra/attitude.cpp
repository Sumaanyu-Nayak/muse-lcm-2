#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

// Generated LCM message headers (from your .lcm files)
#include "../msgs/state_estimator_msgs/attitude_t.hpp"  // must provide imu_t and attitude_t
#include "../msgs/state_estimator_msgs/imu_t.hpp"  // must provide imu_t and attitude_t

// Your estimator header (adjust include path if needed)
#include "../include/state_estimator/Models/attitude_bias_NLO.hpp"
#include "../include/state_estimator/Models/attitude_bias_XKF.hpp"

std::atomic<bool> keep_running(true);
void handle_sigint(int) { keep_running = false; }

class AttitudeEstimatorHandler {
public:
    AttitudeEstimatorHandler(lcm::LCM & lcm,
                             state_estimator::AttitudeBiasXKF * estimator,
                             const Eigen::Matrix3d & b_R_imu,
                             const std::string & out_channel = "ATTITUDE")
    : lcm_(lcm), estimator_(estimator), b_R_imu_(b_R_imu), out_channel_(out_channel)
    {
        // initial xhat state (7x1): quaternion (w,x,y,z) + 3 extras (biases)
        xhat_.setZero();
        xhat_(0) = 1.0;
        normalizeQuatInXhat();

        // inertial direction vectors as before
        m_n_ << 0.577, 0.577, 0.577;
        f_n_ << 0.0, 0.0, 9.81;

        begin_ = true;
    }

    ~AttitudeEstimatorHandler() {
    }

    // LCM callback (generated message pointer)
    void handleMessage(const lcm::ReceiveBuffer* /*rbuf*/, const std::string& chan, const imu_t* msg)
    {
        using namespace std::chrono;

        // Reconstruct message timestamp (ns) from header sec + nsec
        long long msg_ns = static_cast<long long>(msg->head.sec) * 1000000000LL + static_cast<long long>(msg->head.nsec);

        // get now in ns
        auto now_sys = system_clock::now();
        auto now_ns = duration_cast<nanoseconds>(now_sys.time_since_epoch()).count();

        long long latency_ns = now_ns - msg_ns;
        double latency_ms = static_cast<double>(latency_ns) / 1e6;

        std::cout << "[IMU SUB] chan=" << chan
                  << " seq=" << msg->head.sequence_id
                  << " latency=" << latency_ms << " ms\n";

        if (log_.is_open()) log_ << latency_ms << std::endl;

        // convert incoming vectors to Eigen
        Eigen::Vector3d omega(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        // timestamp in seconds (double)
        double msg_time_s = static_cast<double>(msg->head.sec) + static_cast<double>(msg->head.nsec) * 1e-9;

        // first message: initialize quaternion in xhat from IMU orientation (quaternion struct is x,y,z,w)
        if (begin_) {
            // map imu orientation (x,y,z,w) -> xhat ordering (w,x,y,z)
            xhat_(0) = msg->orientation.w;
            xhat_(1) = msg->orientation.x;
            xhat_(2) = msg->orientation.y;
            xhat_(3) = msg->orientation.z;
            normalizeQuatInXhat();

            // if AttitudeBiasXKF has a way to set state directly, call it here.
            // We'll assume estimator was constructed with same initial xhat, otherwise you'd call setter.
            // For safety, we attempt to set internal state via get/set if available (not assumed).
            begin_ = false;
            t0_ = msg_time_s;
            std::cout << "[IMU SUB] initialized state from IMU orientation\n";
        }

        // compute and publish attitude estimate
        computeAndPublish(msg_time_s, omega, acc);
    }

private:
    void normalizeQuatInXhat() {
        Eigen::Vector4d q = xhat_.head<4>();
        double nrm = q.norm();
        if (nrm <= 0.0) { q << 1.0, 0.0, 0.0, 0.0; nrm = 1.0; }
        xhat_.head<4>() = q / nrm;
    }

    void computeAndPublish(double msg_time_s, const Eigen::Vector3d &omega, const Eigen::Vector3d &acc)
    {
        double rel_t = msg_time_s - t0_;

        // measurement: body accel transformed by b_R_imu
        Eigen::Vector3d f_b = b_R_imu_ * acc;

        // predicted magnetometer measurement m_b from quaternion in xhat (remember xhat stores w,x,y,z)
        Eigen::Quaterniond quat_est(xhat_(0), xhat_(1), xhat_(2), xhat_(3));
        Eigen::Vector3d m_b = quat_est.toRotationMatrix() * m_n_;

        Eigen::Matrix<double,6,1> z;
        z << f_b, m_b;

        // Call estimator: estimator_->update(time, omega_in_body, z)
        estimator_->update(rel_t, b_R_imu_ * omega, z);

        // Retrieve state
        xhat_ = estimator_->getX();

        // Compute xdot and filtered angular velocity
        Eigen::Matrix<double,7,1> xdot = estimator_->calc_f(rel_t, xhat_, b_R_imu_ * omega);
        Eigen::Quaterniond quat_dot(xdot(0), xdot(1), xdot(2), xdot(3));

        Eigen::Quaterniond quat_conj(xhat_(0), -xhat_(1), -xhat_(2), -xhat_(3));
        Eigen::Quaterniond tmp = quat_conj * quat_dot;
        Eigen::Vector3d omega_filt(tmp.x(), tmp.y(), tmp.z());
        omega_filt *= 2.0;

        // Build attitude message and publish
        attitude_t out;
        // fill header (sec, nsec)
        double sec_part_d;
        double frac = std::modf(msg_time_s, &sec_part_d);
        out.head.sequence_id = 0; // you can maintain a sequence counter if desired
        out.head.sec = sec_part_d;
        out.head.nsec = frac * 1e9;
        out.head.frame_name = std::string("base_link");

        // store quaternion in array as [w,x,y,z] (consistent with earlier code)
        out.quaternion[0] = xhat_(0);
        out.quaternion[1] = xhat_(1);
        out.quaternion[2] = xhat_(2);
        out.quaternion[3] = xhat_(3);

        // compute Euler (roll,pitch,yaw) from quaternion
        double w = out.quaternion[0], x = out.quaternion[1], y = out.quaternion[2], zq = out.quaternion[3];
        double sinr_cosp = 2.0 * (w * x + y * zq);
        double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (w * y - zq * x);
        double pitch;
        if (std::abs(sinp) >= 1.0) pitch = std::copysign(M_PI/2.0, sinp);
        else pitch = std::asin(sinp);

        double siny_cosp = 2.0 * (w * zq + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y*y + zq*zq);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        out.roll_deg  = roll  * (180.0 / M_PI);
        out.pitch_deg = pitch * (180.0 / M_PI);
        out.yaw_deg   = yaw   * (180.0 / M_PI);

        out.angular_velocity[0] = omega_filt(0);
        out.angular_velocity[1] = omega_filt(1);
        out.angular_velocity[2] = omega_filt(2);

        lcm_.publish(out_channel_, &out);

        // optionally print a bit of info
        static int cnt = 0;
        if ((cnt++ % 20) == 0) {
            std::cout << "[ATT PUB] t=" << msg_time_s
                      << " quat=[ " << out.quaternion[0] << ", " << out.quaternion[1] << ", "
                      << out.quaternion[2] << ", " << out.quaternion[3] << " ]"
                      << " euler(deg)=(" << out.roll_deg << "," << out.pitch_deg << "," << out.yaw_deg << ")\n";
        }
    }

private:
    lcm::LCM & lcm_;
    state_estimator::AttitudeBiasXKF * estimator_;
    Eigen::Matrix3d b_R_imu_;
    Eigen::Vector3d m_n_;
    Eigen::Vector3d f_n_;
    Eigen::Matrix<double,7,1> xhat_;
    Eigen::Vector3d omega_filt;
    std::ofstream log_;
    std::string out_channel_;
    bool begin_;
    double t0_{0.0};
};

// ----------------------
// main()
// ----------------------
int main(int argc, char ** argv)
{
    // parse optional args: [imu_channel] [attitude_channel]
    std::string imu_channel = "/sensors/imu";
    std::string attitude_channel = "/state_estimator/attitude";
    if (argc > 1) imu_channel = argv[1];
    if (argc > 2) attitude_channel = argv[2];

    std::cout << "[MAIN] Subscribing to '" << imu_channel << "' and publishing to '" << attitude_channel << "'\n";

    // default transforms and covariances (same as your plugin)
    Eigen::Matrix3d b_R_imu;
    b_R_imu << -1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, -1.0;

    Eigen::Matrix<double,6,6> P0; P0.setZero(); P0.diagonal().setConstant(1.0e-6);
    Eigen::Matrix<double,6,6> Q; Q.setZero(); Q.diagonal() << 1.0e-6,1.0e-6,1.0e-6,1.0e-8,1.0e-8,1.0e-8;
    Eigen::Matrix<double,6,6> R; R.setZero(); R.diagonal() << 4.0e-2,4.0e-2,4.0e-2,16.0,16.0,16.0;

    Eigen::Matrix<double,7,1> xhat0; xhat0 << 1.0,0.0,0.0,0.0,0.0,0.0,0.0;

    double t0 = 0.0;
    double ki = 0.02, kp = 10.0;

    // instantiate estimator (stack)
    state_estimator::AttitudeBiasXKF estimator(t0, xhat0, P0, Q, R,
                                              Eigen::Vector3d(0.0,0.0,9.81),
                                              Eigen::Vector3d(0.577,0.577,0.577),
                                              ki, kp);

    // init LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "[ERROR] LCM not initialized. Make sure LCM daemon is available.\n";
        return 1;
    }

    // create handler and subscribe
    AttitudeEstimatorHandler handler(lcm, &estimator, b_R_imu, attitude_channel);
    lcm.subscribe(imu_channel, &AttitudeEstimatorHandler::handleMessage, &handler);

    // signals
    std::signal(SIGINT, handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    std::cout << "[MAIN] Ready — waiting for IMU messages...\n";

    // handle loop (non-blocking timeout so we can check keep_running)
    while (keep_running) {
        // use a small timeout to allow ctrl-c
        int r = lcm.handleTimeout(1000); // milliseconds
        if (r < 0) {
            std::cerr << "[ERROR] lcm.handleTimeout returned " << r << "\n";
            break;
        }
    }

    std::cout << "[MAIN] Shutting down.\n";
    return 0;
}
