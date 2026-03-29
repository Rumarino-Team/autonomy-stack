#include "std_msgs/msg/float64_multi_array.hpp"
#include <fstream>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
class Controller : public rclcpp::Node {
public:
  Controller();

  rclcpp::Subscription<Float64MultiArray>::SharedPtr thrusters_sub;

private:
  // Serial command format:
  // - T<NTH_THRUSTER>:<VALUE>\n - NTH_THRUSTER with VALUE (-1 (rev) .. 1 (fwd))
  std::fstream arduino;

  void handle_thrusters_msg(const Float64MultiArray::SharedPtr thrusters);
};
