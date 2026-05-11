#ifndef __SMC_NODE__
#define __SMC_NODE__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Eigen/Dense"
#include <chrono>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>


using namespace std::chrono_literals;

class SmcNode : public rclcpp::Node
{
private:
    // generate a publisher of data
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr control_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr autonomy_subscriber;

    rclcpp::TimerBase::SharedPtr control_publisher_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fuckass;

    // robot parameters
    double r_w_ = 0.05;
    double B_ = 0.3;
    double mu_R_ = 1.0;
    double mu_L_ = 1.0;

    // <return type> <function name> (<parameters>)
    // Jacobian function --> input: geometry (Rw/B), slip efficiencies, orientation
    static Eigen::Matrix<double, 3, 2> b_jacobian(double r_w, double B, double mu_R, double mu_L, Eigen::Vector3d &orientation)
    {

        // hardcode the jacobian matrix to be returned
        Eigen::Matrix<double, 3, 2> dBdu;

        // set all values to zero (they're random)
        dBdu.setZero();

        // unpack all of the orientation angles
        double roll = orientation[0];  // rad
        double pitch = orientation[1]; // rad
        double yaw = orientation[2];   // rad

        // actually input values for jacobian matrix (derivative of X_dot WRT control)
        dBdu << (r_w / 2) * mu_R * cos(pitch) * cos(yaw), (r_w / 2) * mu_L * cos(pitch) * cos(yaw),
            (r_w / 2) * mu_R * cos(pitch) * sin(yaw), (r_w / 2) * mu_L * cos(pitch) * sin(yaw),
            (r_w / B) * mu_R * cos(roll) * cos(pitch), (-r_w / B) * mu_L * cos(roll) * cos(pitch);

        // return
        return dBdu;
    }

    // function to compute B matrix evaluated at current state (and previous control input)
    static Eigen::Vector3d B_Xu(double r_w, double B, double mu_R, double mu_L, Eigen::Vector3d &orientation, Eigen::Vector4d &input)
    {

        // hardcode the B matrix to be returned
        Eigen::Vector3d B_vec;

        // set all values to zero (they're random)
        B_vec.setZero();

        // unpack all of the orientation angles
        double roll = orientation[0];  // rad
        double pitch = orientation[1]; // rad
        double yaw = orientation[2];   // rad

        // unpack control inputs
        double wR = input[0]; // rad/s
        double wL = input[1]; // rad/s
        // input[2], input[3] (dwR, dwL) currently unused in the dynamics

        // actually input values for B matrix
        B_vec << (r_w / 2) * (mu_R * wR + mu_L * wL) * cos(pitch) * cos(yaw), // m/s
            (r_w / 2) * (mu_R * wR + mu_L * wL) * cos(pitch) * sin(yaw),      // m/s
            (r_w / B) * (mu_R * wR - mu_L * wL) * cos(roll) * cos(pitch);     // rad/s

        // return
        return B_vec;
    }

    static Eigen::Vector3d comp_tanh(const Eigen::Vector3d &z, double epsilon)
    {
        return (z.array() / (2.0 * epsilon)).tanh();
    }

    std::mutex state_mutex;
    Eigen::Vector4d control;
    Eigen::VectorXd latest_state_estimate;
    Eigen::VectorXd latest_state_desired;

    // CALLER MUST HOLD state_mutex.
    // Returns the control increment du = [d_wR, d_wL] from the SMC law.
    Eigen::Vector2d compute_control_increment(double r_w, double B, double mu_r, double mu_l)
    {
        Eigen::Vector3d eulers(this->latest_state_estimate[2],
                               this->latest_state_estimate[3],
                               this->latest_state_estimate[4]);

        // matrix inversions -- materialized to a concrete type so we don't
        // hold a lazy expression that references a destroyed temporary
        Eigen::Matrix<double, 2, 3> dbdx_pseudo_inv =
            b_jacobian(r_w, B, mu_r, mu_l, eulers)
                .completeOrthogonalDecomposition()
                .pseudoInverse();

        Eigen::Vector3d B_prev = B_Xu(r_w, B, mu_r, mu_l, eulers, this->control);

        // tracking-relevant state slice: [x, y, yaw]
        Eigen::Vector3d x_track(this->latest_state_estimate[0],
                                this->latest_state_estimate[1],
                                this->latest_state_estimate[4]);
        Eigen::Vector3d s = this->latest_state_desired.head<3>() - x_track;
        Eigen::Vector3d xdot_d = this->latest_state_desired.tail<3>();

        const double epsilon = 10.0;
        Eigen::Vector3d K(0.1, 0.1, 2.0);

        Eigen::Vector3d Ks = (K.array() * comp_tanh(s, epsilon).array()).matrix();
        Eigen::Vector3d residual = -Ks - xdot_d - B_prev;

        return dbdx_pseudo_inv * residual;
    }

    // function to publish the control input calculations
    void publish_callback()
    {
        auto msg = geometry_msgs::msg::Quaternion();
        auto log = std_msgs::msg::String();

        // print estimate and desired state
        auto des = this->latest_state_desired.head<3>();
        Eigen::Vector3d est(this->latest_state_estimate[0],
                            this->latest_state_estimate[1],
                            this->latest_state_estimate[4]);
        Eigen::Vector3d xdot_d = this->latest_state_desired.tail<3>();

        
        // Inside your function:
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "Euler angles: " << this->latest_state_estimate[2] << ", " 
        << this->latest_state_estimate[3] << ", " << this->latest_state_estimate[4]
        << " | X,y,yaw desired: " << des[0] << ", " << des[1] << ", " << des[2]
        << " | X,y,yaw estimate: " << est[0] << ", " << est[1] << ", " << est[2]
        << " | x_dot: " << xdot_d[0] << ", " << xdot_d[1] << ", " << xdot_d[2];

        std::string result = ss.str();
        log.data = result;
        this->fuckass->publish(log);

        if (!this->drive_motors)
        {
            msg.w = 0;
            msg.x = 0;
            msg.y = 0;
            msg.z = 0;
            this->control_publisher->publish(msg);
            return;
            ;
        }

        Eigen::Vector4d ctrl_snapshot;
        {
            std::lock_guard<std::mutex> lk(this->state_mutex);
            Eigen::Vector2d du = compute_control_increment(this->r_w_, this->B_, this->mu_R_, this->mu_L_);
            this->control[0] += du[0];
            this->control[1] += du[1];
            this->control[2] = du[0];
            this->control[3] = du[1];

            ctrl_snapshot = this->control;
        }

        msg.w = std::clamp(ctrl_snapshot[0], -2.0, 2.0);
        msg.x = std::clamp(ctrl_snapshot[1], -2.0, 2.0);
        msg.y = ctrl_snapshot[2];
        msg.z = ctrl_snapshot[3];
        this->control_publisher->publish(msg);
    }

    // function that runs when nav node gives us state estimation
    void nav_callback(nav_msgs::msg::Odometry odom)
    {
        // extract state estimate from nav callback (do math outside the lock)
        auto position = odom.pose.pose.position;
        auto orientation = odom.pose.pose.orientation;

        const double MY_PI = 3.141592653589;

        Eigen::Quaterniond quat(orientation.w, orientation.x, orientation.y, orientation.z);
        quat.normalize();
        auto R = quat.toRotationMatrix();
        double roll = std::atan2(R(2, 1), R(2, 2));
        double s = -R(2, 0);
        double pitch = (std::abs(s) >= 1.0) ? std::copysign(MY_PI / 2.0, s) : std::asin(s);
        double yaw = std::atan2(R(1, 0), R(0, 0));

        std::lock_guard<std::mutex> lk(this->state_mutex);
        this->latest_state_estimate << position.x, position.y, roll, pitch, yaw;
    }

    bool drive_motors = true;

    void autonomy_callback(nav_msgs::msg::Odometry des)
    {
        if (des.pose.pose.orientation.x == 1.0)
        {
            this->drive_motors = false;
            return;
        }
        this->drive_motors = true;

        std::lock_guard<std::mutex> lk(this->state_mutex);
        this->latest_state_desired << des.pose.pose.position.x,
            des.pose.pose.position.y,
            des.pose.pose.position.z,
            des.twist.twist.linear.x,
            des.twist.twist.linear.y,
            des.twist.twist.linear.z;
    }

public:
    SmcNode() : Node("smc_node")
    {
        this->control_publisher = this->create_publisher<geometry_msgs::msg::Quaternion>(
            "/smc/control",
            10);

        // when nav publishes, nav callback runs
        this->nav_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/trans_est",
            10,
            std::bind(&SmcNode::nav_callback, this, std::placeholders::_1));

        this->autonomy_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/auto_next_state",
            10,
            std::bind(&SmcNode::autonomy_callback, this, std::placeholders::_1));

        this->control_publisher_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SmcNode::publish_callback, this));

        this->fuckass = this->create_publisher<std_msgs::msg::String>(
            "/smc/control/log",
            10
        );

        // x y orientation
        this->latest_state_estimate = Eigen::VectorXd(5);
        this->latest_state_estimate.setZero();

        this->latest_state_desired = Eigen::VectorXd(6);
        this->latest_state_desired.setZero();

        this->control.setZero();
    }
};

#endif