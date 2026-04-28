#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fstream>

struct BridgeProteus : public rclcpp::Node {
  using Float64MultiArray = std_msgs::msg::Float64MultiArray;

  std::fstream arduino;
  std::string arduino_rx_buffer;

  rclcpp::Subscription<Float64MultiArray>::SharedPtr thrusters_sub;
  rclcpp::TimerBase::SharedPtr arduino_rx_timer;

  BridgeProteus();

  void handle_thrusters_msg(const Float64MultiArray::SharedPtr thruster_values);
  void poll_arduino_serial();
};
