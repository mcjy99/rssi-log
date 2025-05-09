//monitors the wifi signal strength and publishes to topic wifi_signal
#include <chrono>
#include <cstring>
#include <iostream>
#include <array>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class WifiPublisher : public rclcpp::Node
{
public:
  WifiPublisher() : Node("wifi_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("wifi_signal", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&WifiPublisher::scan_wifi, this));
  }

private:
  void scan_wifi()
  {
    FILE* pipe = popen("iw dev wlan0 link", "r");
    if (!pipe) {
      RCLCPP_ERROR(this->get_logger(), "popen() failed!");
      return;
    }
      
    char buffer[128];
    std::string result = "";
    while (!feof(pipe)) {
      if (fgets(buffer, 128, pipe) != NULL) {
        result += buffer;
      }
    }
    pclose(pipe);

    int signal_strength = -100;  // Default to very low signal

    // Check if not connected
    if (result.find("Not connected.") != std::string::npos) {
      RCLCPP_WARN(this->get_logger(), "WiFi connection not found, will switch to LoRa");
    }
    // Parse signal strength if connected
    else {
      size_t pos = result.find("signal:");
      if (pos != std::string::npos) {
        int parsed_signal;
        if (sscanf(result.c_str() + pos, "signal: %d", &parsed_signal) == 1) {
          signal_strength = parsed_signal;
        }
      }
    }

    // Create and publish the message
    auto message = std_msgs::msg::Int32();
    message.data = signal_strength;
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Published WiFi signal strength: %d dBm", signal_strength);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WifiPublisher>());
    rclcpp::shutdown();
    return 0;
}