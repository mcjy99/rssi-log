#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // Add this for String message type

class LoraPublisher : public rclcpp::Node
{
public:
    LoraPublisher() : Node("lora_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("lora_rssi");
        timer_ = this->create_wall_timer(1ms, std::bind(&LoraPublisher::scan_rssi, this));
    }

private:
    void setupSerialPort()
    {
        serial_port_ = open("/dev/ttyTHS1", O_RDWR);
        if (serial_port_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port");
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(serial_port_, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting serial port attributes");
            return;
        }

        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting serial port attributes");
            return;
        }

        // Configure LoRa module
        writeSerial("AT+MODE=TEST");
        writeSerial("AT+TEST=RFCFG,923,SF7,125,8,8,14,ON,OFF,OFF");
    }
    void writeSerial(const std::string &cmd)
    {
        write(serial_port_, cmd.c_str(), cmd.length());
        std::this_thread::sleep_for(std::chrono::seconds(1));

        char response[256];
        memset(response, 0, sizeof(response));
        int n = read(serial_port_, response, sizeof(response));
        if (n > 0)
        {
            RCLCPP_INFO(this->get_logger(), "LoRa module response: %s", response);
        }
    }
}