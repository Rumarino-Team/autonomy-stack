#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#ifdef BRIDGE_HARDWARE_ENABLE_VN100
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#endif

#include <fstream>

struct BridgeProteus : public rclcpp::Node {
  using Float64MultiArray = std_msgs::msg::Float64MultiArray;
#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  using ImuMsg = sensor_msgs::msg::Imu;
  using FluidPressureMsg = sensor_msgs::msg::FluidPressure;
  using MagneticFieldMsg = sensor_msgs::msg::MagneticField;
#endif

  std::fstream arduino;
#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  std::fstream vn100;
  bool vn100_enabled = false;
#endif

  rclcpp::Subscription<Float64MultiArray>::SharedPtr thrusters_sub;
#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub;
  rclcpp::Publisher<FluidPressureMsg>::SharedPtr pressure_pub;
  rclcpp::Publisher<MagneticFieldMsg>::SharedPtr magnetic_pub;
  rclcpp::TimerBase::SharedPtr vn_timer;
#endif

  BridgeProteus();

  void handle_thrusters_msg(const Float64MultiArray::SharedPtr thruster_values);

  void read_vn100();
};
