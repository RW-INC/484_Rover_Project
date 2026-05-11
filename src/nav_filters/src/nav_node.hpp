#ifndef NAV_NODE_HPP
#define NAV_NODE_HPP

#include "ekf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <mutex>
#include <queue>
#include <fstream>

using SteadyClock = std::chrono::steady_clock;

class NavigationNode : public rclcpp::Node
{
private:
    static constexpr double MY_PI = 3.141592653589793238462643383279502884;

    // define initial x-y-z (height) inertial distance of rover UWB to anchor UWB (for ranging calibration)
    double START_X_FROM_ANCHOR = 0.0; // meters
    double START_Y_FROM_ANCHOR = 0.0; // meters
    double START_Z_FROM_ANCHOR = 0.0; // meters

    Eigen::Quaterniond rotation_matrix;

    inline double rad2deg(double a) { return a * 180.0 / MY_PI; }

    // attributes of the nav_node class
    Eigen::VectorXd prior_orientation;
    Eigen::VectorXd prior_location; // [x, y, z, vx, vy, vz, bax, bay, baz]
    Eigen::VectorXd latest_imu;
    Eigen::Matrix<double, 6, 6> prior_P_rot;
    Eigen::Matrix<double, 15, 15> prior_P_loc; // covariance matrix

    // --- Startup Calibration ---
    bool is_calibrated_imu = false;
    bool is_calibrated_uwb = false;
    uint32_t calibration_samples_imu = 0;
    uint32_t calibration_samples_uwb = 0;
    static constexpr uint32_t REQUIRED_CALIBRATION_SAMPLES = 1000;
    static constexpr uint32_t REQUIRED_UWB_CALIBRATION = 20;

    // zero'ing out things 
    Eigen::Vector3d baseline_gravity_vector = Eigen::Vector3d::Zero();
    Eigen::Vector3d baseline_gyro_vector = Eigen::Vector3d::Zero();
    double baseline_range_rate = 0.0;

    std::queue<double> range_buffer;
    std::queue<std::chrono::steady_clock::time_point> uwb_time_buffer;

    // --- Frame lock ---
    bool world_frame_locked = false;
    Eigen::Matrix3d R_anchor_world = Eigen::Matrix3d::Identity();
    double psi0 = 0.0;

    std::ofstream file_out;


    std::chrono::time_point<SteadyClock> last_imu;
    std::chrono::time_point<SteadyClock> last_uwb;
    std::chrono::time_point<SteadyClock> last_ctrl;
    double sigma_gyro = 0.001;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimation_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMU_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr UWB_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr control_subscription;

    
    float32_t flag = 0.0;

    // who has priority when accessing
    std::mutex state_mutex;

    Eigen::Vector3d latest_gyro = Eigen::Vector3d::Zero();
    Eigen::Quaterniond latest_q_triad = Eigen::Quaterniond::Identity();

    double latest_ctrl_x = 0.0;
    double latest_ctrl_y = 0.0;
    double latest_ctrl_dx = 0.0;
    double latest_ctrl_dy = 0.0;

    bool have_imu = false;
    bool have_control = false;
    bool have_uwb = false;
    bool prior_orientation_initialized = false;
    bool is_rotated = false;

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Eigen::Vector3d accel(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);
        Eigen::Vector3d gyro(msg->angular_velocity.x,
                             msg->angular_velocity.y,
                             msg->angular_velocity.z);
        Eigen::Quaterniond q_triad(msg->orientation.w, msg->orientation.x,
                                   msg->orientation.y, msg->orientation.z);

        if (!this->is_rotated) {
            Eigen::Quaterniond nominal = Eigen::Quaterniond::Identity();
            this->rotation_matrix = nominal * q_triad.conjugate();
            this->is_rotated = true;
        }

        if (!accel.allFinite() || !gyro.allFinite() || !q_triad.coeffs().allFinite())
            return;
        if (q_triad.coeffs().squaredNorm() < 1e-12)
            return;
        q_triad.normalize();

        Eigen::Vector3d global_accel = q_triad * accel;

        if (!is_calibrated_imu)
        {
            calibration_samples_imu++;
            baseline_gravity_vector += (global_accel - baseline_gravity_vector) / static_cast<double>(calibration_samples_imu);
            baseline_gyro_vector += (gyro - baseline_gyro_vector) / static_cast<double>(calibration_samples_imu);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "CALIBRATING IMU... [%u/%u] | baseline gravity vec=[%.4f, %.4f, %.4f] | baseline gyro vec=[%.4f, %.4f, %.4f]",
                                 calibration_samples_imu, REQUIRED_CALIBRATION_SAMPLES,
                                 baseline_gravity_vector.x(), baseline_gravity_vector.y(), baseline_gravity_vector.z(),
                                 baseline_gyro_vector.x(), baseline_gyro_vector.y(), baseline_gyro_vector.z());

            if (calibration_samples_imu >= REQUIRED_CALIBRATION_SAMPLES)
            {
                Eigen::Matrix3d R_wb_init = q_triad.toRotationMatrix();
                psi0 = std::atan2(R_wb_init(1, 0), R_wb_init(0, 0));

                R_anchor_world = Eigen::AngleAxisd(-psi0, Eigen::Vector3d::UnitZ())
                                     .toRotationMatrix();

                baseline_gravity_vector = R_anchor_world * baseline_gravity_vector;

                Eigen::Quaterniond q_anchor_world(R_anchor_world);
                Eigen::Quaterniond q_anchor_b = q_triad;//q_anchor_world * q_triad;
                q_anchor_b.normalize();
                prior_orientation[0] = q_anchor_b.w();
                prior_orientation[1] = q_anchor_b.x();
                prior_orientation[2] = q_anchor_b.y();
                prior_orientation[3] = q_anchor_b.z();

                prior_orientation_initialized = true;
                is_calibrated_imu = true;
                world_frame_locked = true;

                latest_q_triad = q_triad;

                RCLCPP_INFO(this->get_logger(),
                            "IMU CALIBRATION COMPLETE. World frame locked. psi0 = %.4f rad (%.2f deg)",
                            psi0, rad2deg(psi0));
            }

            last_imu = SteadyClock::now();
            have_imu = true;
            latest_imu = Eigen::VectorXd(3);
            latest_imu.setZero();

            prior_orientation[6] = baseline_gyro_vector[2];
            prior_orientation[5] = baseline_gyro_vector[1];
            prior_orientation[4] = baseline_gyro_vector[0];

            prior_location[10] = baseline_range_rate;
            prior_location[8] = baseline_gravity_vector[2];
            prior_location[7] = baseline_gravity_vector[1];
            prior_location[6] = baseline_gravity_vector[0];  
            baseline_range_rate = (range_buffer.back() - range_buffer.front()) /
                                    ((std::chrono::duration<double>(uwb_time_buffer.back() - uwb_time_buffer.front()).count()));
             
            return;
        }

        auto now = SteadyClock::now();
        std::lock_guard<std::mutex> lk(state_mutex);

        latest_gyro = gyro;
        
        latest_q_triad = this->rotation_matrix * q_triad;

        double dt = std::chrono::duration<double>(now - last_imu).count();
        last_imu = now;
        if (dt <= 0.0 || dt > 0.5)
            return;

        Eigen::Vector3d a_world = q_triad * accel;
        Eigen::Vector3d a_anchor = R_anchor_world * a_world;
        Eigen::Vector3d a_inertial = a_anchor;

        if (!a_inertial.allFinite())
            return;

        latest_imu[0] = a_inertial.x();
        latest_imu[1] = a_inertial.y();
        latest_imu[2] = a_inertial.z();
    }

    void control_cb(const geometry_msgs::msg::Quaternion::SharedPtr msg)
    {
        if (!std::isfinite(msg->x) || !std::isfinite(msg->y))
            return;

        auto now = SteadyClock::now();
        std::lock_guard<std::mutex> lk(state_mutex);

        if (have_control)
        {
            double dt = std::chrono::duration<double>(now - last_ctrl).count();
            // if (dt > 1e-3)
            // {
            //     latest_ctrl_dx = (msg->x - latest_ctrl_x) / dt;
            //     latest_ctrl_dy = (msg->y - latest_ctrl_y) / dt;
            // }
        }
        latest_ctrl_x = msg->w;
        latest_ctrl_y = msg->x;
        latest_ctrl_dx = msg->y;
        latest_ctrl_dy = msg->z;
        last_ctrl = now;
        have_control = true;
    }

    void uwb_cb(const std_msgs::msg::Float32::SharedPtr uwb_msg)
    {
            if (!std::isfinite(uwb_msg->data))
                return;
            auto now = SteadyClock::now();
            if (!is_calibrated_uwb)
            {
                calibration_samples_uwb++;
                range_buffer.push(uwb_msg->data);
                uwb_time_buffer.push(now);
                if (calibration_samples_uwb >= REQUIRED_UWB_CALIBRATION)
                {
                    is_calibrated_uwb = true;
                }
                return;
            }

            std::lock_guard<std::mutex> lk(state_mutex);
            if (!have_imu || !is_calibrated_imu)
                return;

            if (!have_uwb)
            {
                last_uwb = now;
                have_uwb = true;
                return;
            }

            const float_t range = uwb_msg->data;
            const float_t range_rate = (range_buffer.back() - range_buffer.front()) /
                                    ((std::chrono::duration<double>(uwb_time_buffer.back() - uwb_time_buffer.front()).count()));
            range_buffer.pop();
            range_buffer.push(range);
            double dt = std::chrono::duration<double>(now - last_uwb).count();

            uwb_time_buffer.pop();
            uwb_time_buffer.push(now);

            last_uwb = now;
            if (dt <= 0.0)
                return;

            Eigen::VectorXd imu_pseudo = latest_imu; // 3-vector now: [ax, ay, az]

            Eigen::Quaterniond q_anchor_world(R_anchor_world);
            Eigen::Quaterniond q_anchor_meas = q_anchor_world * latest_q_triad;
            q_anchor_meas.normalize();

            Eigen::VectorXd new_orientation;
            Eigen::Matrix<double, 6, 6> new_P_rot;
            rotational_mekf(q_anchor_meas, latest_gyro,
                            prior_orientation, prior_P_rot,
                            dt, sigma_gyro,
                            new_P_rot, new_orientation);

            if (new_orientation.allFinite())
            {
                prior_orientation = new_orientation;
                prior_P_rot = new_P_rot;
            }
            else
                exit(-696969);

            Eigen::Quaterniond quat(prior_orientation[0], prior_orientation[1],
                                    prior_orientation[2], prior_orientation[3]);
            quat.normalize();
            auto R = quat.toRotationMatrix();
            double roll = std::atan2(R(2, 1), R(2, 2));
            double s = -R(2, 0);
            double pitch = (std::abs(s) >= 1.0) ? std::copysign(MY_PI / 2.0, s) : std::asin(s);
            double yaw = std::atan2(R(1, 0), R(0, 0));
            Eigen::Vector3d orientation(roll, pitch, yaw);

            Eigen::VectorXd control(4);
            control << latest_ctrl_x, latest_ctrl_y, latest_ctrl_dx, latest_ctrl_dy;

            Eigen::VectorXd new_location;
            new_location = Eigen::VectorXd(15);
            Eigen::Matrix<double, 15, 15> new_P_loc;

            translational_ekf(imu_pseudo, range, range_rate, orientation, control,
                            prior_location, prior_P_loc, prior_P_rot, dt,
                            new_P_loc, new_location);

            if (new_location.allFinite())
            {
                prior_location << new_location.head<6>(), new_location.tail<5>();
                prior_P_loc = new_P_loc;
            }
            else
            {
                exit(-1000);
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "EKF OUT | pos=[%.3f, %.3f, %.3f], ranging=[%.3f, %.3f], a_inertial=[%.3f, %.3f, %.3f], time=%.3f",
                                prior_location[0], prior_location[1], prior_location[2],
                                range - prior_location[9], range_rate - prior_location[10],
                                latest_imu[0] - prior_location[6],
                                latest_imu[1] - prior_location[7],
                                latest_imu[2] - prior_location[8],
                                std::chrono::duration<double>(uwb_time_buffer.back().time_since_epoch()).count());

            publish_estimate_locked(prior_P_loc, prior_P_rot);
        
    }

    void publish_estimate_locked(Eigen::Matrix<double, 15, 15> p_loc,
                                 Eigen::Matrix<double, 6, 6> p_rot)
    {   
        // two structs within odom - pose & twist (position & orientation, angular velocity & linear velocity)
        nav_msgs::msg::Odometry est_msg;
        est_msg.header.stamp = this->now();
        est_msg.header.frame_id = "anchor";
        est_msg.child_frame_id = "base_link";

        // storing position & orientation
        auto &p = est_msg.pose.pose;
        p.position.x = prior_location[0]; 
        p.position.y = prior_location[1];
        p.position.z = prior_location[2];
        p.orientation.w = prior_orientation[0]; // quaternion
        p.orientation.x = prior_orientation[1];
        p.orientation.y = prior_orientation[2];
        p.orientation.z = prior_orientation[3];

        // storing linear/angular velocity 
        auto &t = est_msg.twist.twist;
        t.linear.x = prior_location[3];
        t.linear.y = prior_location[4];
        t.linear.z = prior_location[5];
        t.angular.x = latest_gyro[0] - prior_orientation[4] - baseline_gyro_vector[0];
        t.angular.y = latest_gyro[1] - prior_orientation[5] - baseline_gyro_vector[1];
        t.angular.z = latest_gyro[2] - prior_orientation[6] - baseline_gyro_vector[2];

        estimation_publisher->publish(est_msg);

        this->file_out << prior_location[0] << ","
                       << prior_location[1] << ","
                       << prior_location[2] << ","
                       << prior_location[3] << ","
                       << prior_location[4] << ","
                       << prior_location[5] << ","
                       << prior_orientation[0] << ","
                       << prior_orientation[1] << ","
                       << prior_orientation[2] << ","
                       << prior_orientation[3] << ",";
                    //    << t.angular.x << ","
                    //    << t.angular.y << ","
                    //    << t.angular.z << ",";

        auto p_loc_diag = p_loc.diagonal().array();
        auto p_rot_diag = p_rot.diagonal().array();

        for (uint32_t i = 0; i < p_loc_diag.size(); i++)
            file_out << p_loc_diag[i] << ",";
        for (uint32_t i = 0; i < p_rot_diag.size(); i++)
            file_out << p_rot_diag[i] << ",";

        file_out << "\n";
    }
public:
    NavigationNode() : Node("nav_node")
    {

        file_out.open("output.csv");
        IMU_subscription = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&NavigationNode::imu_cb, this, std::placeholders::_1));

        UWB_subscription = create_subscription<std_msgs::msg::Float32>(
            "/uwb", rclcpp::SensorDataQoS(),
            std::bind(&NavigationNode::uwb_cb, this, std::placeholders::_1));

        control_subscription = create_subscription<geometry_msgs::msg::Quaternion>(
            "/smc/control", 10,
            std::bind(&NavigationNode::control_cb, this, std::placeholders::_1));
        
        this->declare_parameter<double>("init_x", 0.0);
        this->declare_parameter<double>("init_y", 0.0);
        this->declare_parameter<double>("init_z", 0.0);

        this->START_X_FROM_ANCHOR = this->get_parameter("init_x").as_double();
        this->START_Y_FROM_ANCHOR = this->get_parameter("init_y").as_double();
        this->START_Z_FROM_ANCHOR = this->get_parameter("init_z").as_double();

        estimation_publisher = create_publisher<nav_msgs::msg::Odometry>(
            "/trans_est", 10);

        prior_orientation = Eigen::VectorXd(7);
        prior_orientation.setZero();
        prior_orientation[0] = 1.0;
        prior_orientation[4] = -5.303292015263019e-05;
        prior_orientation[5] = -0.0007559242840490004;
        prior_orientation[6] = 0.00016513284163815418;

        prior_location = Eigen::VectorXd::Zero(11);
        prior_location[0] = START_X_FROM_ANCHOR;
        prior_location[1] = START_Y_FROM_ANCHOR;
        prior_location[2] = START_Z_FROM_ANCHOR;

        prior_location[10] = 1e-4; // brangerate
        prior_location[9] = 1e-1;  // brange

        prior_location[8] = 1e-2; // baz
        prior_location[7] = 1e-2; // bay
        prior_location[6] = 1e-2; // bax

        prior_P_loc.setZero();
        prior_P_loc.diagonal() << pow(0.01, 2), pow(0.03, 2), pow(0.01, 2), pow(0.001, 2), pow(0.001, 2),
            pow(0.001, 2), pow(5e-2, 2), pow(5e-3, 2), pow(5e-2, 2), pow(5e-3, 2), pow(1e-2, 2), pow(1e-2, 2), pow(1e-2, 2), pow(1e-2,2), pow(1e-4,2);
        prior_P_rot.setZero();
        prior_P_rot.diagonal() << 0.01, 0.01, 0.01, 7.6e-4, 7.6e-4, 7.6e-4;
    }
};

#endif