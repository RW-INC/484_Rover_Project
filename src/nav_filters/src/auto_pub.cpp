
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <mutex>
#include <queue>
#include <fstream>


class AutoPublisher : public rclcpp::Node 
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_successful;
        void callback(const geometry_msgs::msg::Vector3::SharedPtr  msg) const{
            if (pow(msg->x,2) + pow(msg->y,2) + pow(msg->z,2) < 1e-3) 
            {
                std::string cmd = "pkill -f nav_node";

                int status = std::system(cmd.c_str());
                std_msgs::msg::Int32 msg;
                msg.data = WEXITSTATUS(status);

                this->publisher_successful->publish(msg);
            }
            else
            {
                std::string cmd = "ros2 run nav_filters nav_node --ros-args "
                "-p init_x:=" + std::to_string(msg->x) + " "
                "-p init_y:=" + std::to_string(msg->y) + " "
                "-p init_z:=" + std::to_string(msg->z) + " &";

                int status = std::system(cmd.c_str());
                std::cout << status << std::endl;
                std_msgs::msg::Int32 msg;
                msg.data = WEXITSTATUS(status);

                this->publisher_successful->publish(msg);
            }
        }
    public:
        AutoPublisher() : Node("auto_pub_node")
        {
            this->subscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/start_navigation", 10, std::bind(&AutoPublisher::callback, this, std::placeholders::_1)
            );

            this->publisher_successful = this->create_publisher<std_msgs::msg::Int32>(
                "/nav_node_successful",10
            );
        }
};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}