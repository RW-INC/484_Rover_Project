#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class UWBBridge : public rclcpp::Node
{
public:
    UWBBridge() : Node("uwb_bridge_node")
    {
        // Allows different baud rate and serial port to be set up during launch if desired
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        serial_port = this->get_parameter("serial_port").as_string();
        baud_rate = this->get_parameter("baud_rate").as_int();

        // Opens the serial port
        open_serial_port();

        // Subscription that pulls any message pushed into the uwb command topic
        this->uwb_publisher = this->create_publisher<std_msgs::msg::Float32>("/uwb", 10);
        using namespace std::chrono_literals;
        this->read_timer = this->create_wall_timer(10ms,
                                                   std::bind(&UWBBridge::rwd, this));
    }

    ~UWBBridge()
    {
        if (serial_fd >= 0)
        {
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
        if (baud_rate == 9600)
            speed = B9600;
        else if (baud_rate == 57600)
            speed = B57600;
        else if (baud_rate == 115200)
            speed = B115200;

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = ICANON;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

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

    // Callback function that is called whenever a message is published to the uwb command topic, sends the command to the uwb via serial
    void rwd()
    {
        if (serial_fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open. Cannot send command.");
            return;
        }

        // get the data 
        char buffer[256];
        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer));
        if (bytes_read <= 0) {
            return;   // nothing to do
        }
        buffer[bytes_read] = '\0';
        try {
            float range_m = std::stof(buffer);
            auto msg = std_msgs::msg::Float32();
            msg.data = range_m;
            uwb_publisher->publish(msg);
        } catch (const std::exception&) {}    
        
        // Sets up string to be sent an appends a new line for uwb parsing
        // std::string command = msg->data;
        // command.append("\n");
        // ssize_t bytes_written = write(serial_fd, command.c_str(), command.size());
        // tcdrain(serial_fd);
        // if (bytes_written < 0)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", strerror(errno));
        // }
    }
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr uwb_publisher;
    rclcpp::TimerBase::SharedPtr read_timer;
    int serial_fd = -1;
    std::string serial_port;
    int baud_rate;
};

// Responsible for spinning up the uwbBridge node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UWBBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}