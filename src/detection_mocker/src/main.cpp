#include <rclcpp/rclcpp.hpp>
#include "detection_mocker/detection_mocker.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<detection_mocker::DetectionMocker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
