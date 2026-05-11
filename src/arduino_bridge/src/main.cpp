#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class ArduinoBridge : public rclcpp::Node
{
    public:
        ArduinoBridge() : Node("arduino_bridge_node")
        {
            // Allows different baud rate and serial port to be set up during launch if desired
            this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
            this->declare_parameter<int>("baud_rate", 115200);
            serial_port = this->get_parameter("serial_port").as_string();
            baud_rate = this->get_parameter("baud_rate").as_int();

            // Opens the serial port
            open_serial_port();

            // Subscription that pulls any message pushed into the arduino command topic
            arduino_subscription = this->create_subscription<std_msgs::msg::String>(
                "/arduino_cmd",
                10,
                std::bind(&ArduinoBridge::arduino_command_callback, this, std::placeholders::_1)
            );
        }

        ~ArduinoBridge()
        {
            if (serial_fd >= 0) {
                close(serial_fd);
            }
        }
        
    private:
        // Opens the serial port to desired baud rate and port
        void open_serial_port()
        {
            serial_fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (serial_fd < 0) 
            {
                RCLCPP_ERROR(this->get_logger(), "Error opening serial port %s: %s", serial_port.c_str(), strerror(errno));
                return; 
            }

            struct termios tty;
            memset(&tty, 0, sizeof tty);
            
            if (tcgetattr(serial_fd, &tty) != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", std::strerror(errno));
                close(serial_fd);
                serial_fd = -1;
                return;
            }
            
            speed_t speed = B115200;
            if (baud_rate == 9600) speed = B9600;
            else if (baud_rate == 57600) speed = B57600;
            else if (baud_rate == 15200) speed = B115200;

            cfsetospeed(&tty, speed);
            cfsetispeed(&tty, speed);

            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_iflag &= ~IGNBRK;
            tty.c_lflag = 0;
            tty.c_oflag = 0;
            tty.c_cc[VMIN]  = 0;
            tty.c_cc[VTIME] = 1;
          
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(PARENB | PARODD);
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;

            if (tcsetattr(serial_fd, TCSANOW, &tty) != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", std::strerror(errno));
                close(serial_fd);
                serial_fd = -1;
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened at baud rate %d", serial_port.c_str(), baud_rate);
            sleep(3);
        }

        // Callback function that is called whenever a message is published to the arduino command topic, sends the command to the arduino via serial
        void arduino_command_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            if (serial_fd < 0) 
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open. Cannot send command.");
                return;
            }
            // Sets up string to be sent an appends a new line for arduino parsing
            std::string command = msg->data;
            command.append("\n");
            ssize_t bytes_written = write(serial_fd, command.c_str(), command.size());
            tcdrain(serial_fd);
            if (bytes_written < 0) 
            {
                RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", strerror(errno));
            }
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arduino_subscription;
        int serial_fd = -1;
        std::string serial_port;
        int baud_rate;
};

// Responsible for spinning up the ArduinoBridge node
int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}