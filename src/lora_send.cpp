//sends timestamp from /mavros/localposition/odom to GCS via LoRa P2P
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <iomanip>  // For std::setw and std::setfill
#include <sstream>  // For std::stringstream

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/string.hpp"

class LoraSender : public rclcpp::Node
{
public:
    LoraSender() : Node("lora_send")
    {
        serial_port = open("/dev/ttyTHS1", O_RDWR); // Serial port of LoRa node
        if (serial_port < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
            return;
        }

        this->declare_parameter<std::string>("SF", "7"); // Change SF
        SF_ = this->get_parameter("SF").as_string();
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", 10,
            std::bind(&LoraSender::odom_callback, this, std::placeholders::_1));

        serial_setup();
        write_serial("AT+MODE=TEST");
        std::stringstream ss1;
        ss1 << "AT+TEST=RFCFG,923,SF" << SF_ << ",125,8,8,14,ON,OFF,OFF";
        write_serial(ss1.str().c_str());

        RCLCPP_INFO(this->get_logger(), "LoRa transmitter initialised");
    }
    
    ~LoraSender()
    {
        if (serial_port >= 0)
        {
            close(serial_port);
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto timestamp_sec = msg->header.stamp.sec;
        auto timestamp_nanosec = msg->header.stamp.nanosec;
        // Format timestamp as a hex string for LoRa transmission
        std::stringstream ss2;
        ss2 << std::hex << std::uppercase << std::setfill('0');
        ss2 << std::setw(8) << timestamp_sec;     // 8 hex digits for seconds
        ss2 << std::setw(8) << timestamp_nanosec; // 8 hex digits for nanoseconds
        std::string preamble = "AT+TEST=TXLRPKT,\"";
        std::string timestamp_hex = ss2.str();
        std::string lora_cmd = preamble + timestamp_hex + "\"\r\n";

        RCLCPP_INFO(this->get_logger(), "Sending timestamp: %s", timestamp_hex.c_str());
        write_serial(lora_cmd.c_str());
    }

    void serial_setup()
    {
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_port, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
            return;
        }

        cfsetospeed(&tty, B9600); // in/out baud rate
        cfsetispeed(&tty, B9600);
        tty.c_cflag &= ~PARENB;                                                      // disable parity
        tty.c_cflag &= ~CSTOPB;                                                      // 1 stop bit
        tty.c_cflag &= ~CSIZE;                                                       // clear all size bits then set below
        tty.c_cflag |= CS8;                                                          // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;                                                     // disable hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;                                               // turn on receiver
        tty.c_lflag &= ~ICANON;                                                      // Disable canonical mode
        tty.c_lflag &= ~ECHO;                                                        // Disable echo
        tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                      // Disable newline echo
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // disable software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes
        tty.c_oflag &= ~OPOST;                                                       // prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR;                                                       // Prevent conversion of newline to carriage return/line feed
        tty.c_cc[VMIN] = 0;                                                          // no minimum characters to read
        tty.c_cc[VTIME] = 10;                                                        // wait up to 1s (10 deciseconds) for data

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
        }
    }

    void write_serial(const char *serial_cmd)
    {
        write(serial_port, serial_cmd, strlen(serial_cmd)); // write to serial port

        // Add a delay to ensure the command is processed
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        char response[256]; // read and print response
        memset(response, 0, sizeof(response));
        int n = read(serial_port, response, sizeof(response));
        if (n > 0)
        {
            RCLCPP_INFO(this->get_logger(), "LoRa Response: %s", response);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No response from LoRa module");
        }
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    int serial_port;
    std::string SF_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoraSender>());
    rclcpp::shutdown();
    return 0;
}