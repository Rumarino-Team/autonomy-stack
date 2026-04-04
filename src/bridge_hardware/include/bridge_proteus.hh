#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fstream>

struct BridgeProteus : public rclcpp::Node {
  using Float64MultiArray = std_msgs::msg::Float64MultiArray;
  using ImuMsg = sensor_msgs::msg::Imu;
  using FluidPressureMsg = sensor_msgs::msg::FluidPressure;
  using MagneticFieldMsg = sensor_msgs::msg::MagneticField;

  std::fstream arduino;
  std::fstream vn100;

  rclcpp::Subscription<Float64MultiArray>::SharedPtr thrusters_sub;
  rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub;
  rclcpp::Publisher<FluidPressureMsg>::SharedPtr pressure_pub;
  rclcpp::Publisher<MagneticFieldMsg>::SharedPtr magnetic_pub;
  rclcpp::TimerBase::SharedPtr vn_timer;

  BridgeProteus();

  void handle_thrusters_msg(const Float64MultiArray::SharedPtr thruster_values);

  void read_vn100();
};
