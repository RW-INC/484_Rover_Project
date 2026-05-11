#ifndef __MOTOR_COMMANDER_NODE__
#define __MOTOR_COMMANDER_NODE__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <chrono>
#include <cmath>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Creates the node which creates motor controller commands that will be sent to the arduino
class MotorCommander : public rclcpp::Node
{
    public:
        MotorCommander() : Node("motor_commander_node")
        {
            // Initialize parameters, subscribers, and publishers
            manual_operation = false;
            last_manual_operation_time = this->now();
            joystick_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(&MotorCommander::motor_command_callback, this, std::placeholders::_1)
            );
            control_subscription = this->create_subscription<geometry_msgs::msg::Quaternion>(
                "/smc/control",
                10,
                std::bind(&MotorCommander::auto_motor_command_callback, this, std::placeholders::_1)
            );
            arduino_publisher = this->create_publisher<std_msgs::msg::String>("/arduino_cmd", 10);
            manual_operation_timer = this->create_wall_timer(
                100ms,
                std::bind(&MotorCommander::check_manual_operation, this)
            );
            command_loop_timer = this->create_wall_timer(
                67ms,
                std::bind(&MotorCommander::command_loop, this)
            );
        }
    private:
        // Initializes values for linear and angular velocity, and speed and rotation values to be sent to the rover
        double linear_x = 0.0;
        double angular_z = 0.0;
        double rover_speed_manual = 0.0;
        double rover_rotation_speed_manual = 0.0;
        double w_r = 0.0, w_l = 0.0;

        // Pulls linear and angular values from the joystick topic and updates a last time since joystick published value 
        // Also updates the manual operation boolean to true if it is not already true
        void motor_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            last_manual_operation_time = this->now();
            linear_x = msg->linear.x;
            angular_z = msg->angular.z;
        }

        void auto_motor_command_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
        {
            last_manual_operation_time = this->now();
            std::cout << msg->w << " " << msg->x << " " << msg->y << " " << msg->z << " " << std::endl;
            this->w_r = msg->w;
            this->w_l = msg->x;
        }

        // Checks if the joystick has published a value in the last 0.5 seconds, if not it sets the manual operation boolean to false
        void check_manual_operation()
        {
            double dt = (this->now() - last_manual_operation_time).seconds();
            if (dt > 0.5 && manual_operation)
            {
                manual_operation = false;
            }
        }

        // Command loop pulls values from either manual or autonomy commands then sends values to arduino
        // Loop runs at 50Hz
        void command_loop()
        {
            if (manual_operation)
            {
                double x = linear_x;
                double z = angular_z;
                if (abs(x)<0.05 && abs(z)<0.15) {
                    rover_speed_manual = 0.0;
                    rover_rotation_speed_manual = 0.0;
                } else if (abs(x)<0.05 && abs(z)>=0.15) {
                    rover_speed_manual = 0.0;
                    rover_rotation_speed_manual = z;
                } else if (abs(x)>=0.05 && abs(z)<0.15) {
                    rover_speed_manual = x;
                    rover_rotation_speed_manual = 0.0;
                } else {
                    rover_speed_manual = x;
                    rover_rotation_speed_manual = z;
                }
                send_command_to_arduino("MOVE:" + std::to_string(rover_speed_manual) + "," + std::to_string(rover_rotation_speed_manual));              
            }
            else 
            {
                std::cout << "AUTO:" + std::to_string(this->w_r) + "," + std::to_string(this->w_l) << std::endl;
                send_command_to_arduino("AUTO:" + std::to_string(this->w_r) + "," + std::to_string(this->w_l));
            }
        }

        // Function that sends commands to a topic that the arduino bridge node is subscribed to
        void send_command_to_arduino(const std::string cmd)
        {
            std_msgs::msg::String cmd_msg;
            cmd_msg.data = cmd;
            arduino_publisher->publish(cmd_msg);
        }
        // Initializes subscriptions, publishers, and timers
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joystick_subscription;
        rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr control_subscription;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_publisher;
        rclcpp::TimerBase::SharedPtr manual_operation_timer;
        rclcpp::TimerBase::SharedPtr command_loop_timer;
        rclcpp::Time last_manual_operation_time;
        bool manual_operation = true;

};
#endif