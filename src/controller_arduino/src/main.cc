#include <main.hh>

Controller::Controller() : Node("controller") {
  this->declare_parameter<std::string>("control_port");
  this->declare_parameter<int>("baud_rate", 115200);

  std::string control_port;
  if (!this->get_parameter("control_port", control_port)) {
    RCLCPP_FATAL(this->get_logger(), "control_port not set!");
  }

  int baud_rate;
  if (!this->get_parameter("baud_rate", baud_rate)) {
    RCLCPP_WARN(this->get_logger(), "baud_rate not set!");
  }

  std::ostringstream oss;
  oss << "stty -F " << control_port << " " << baud_rate
      << " cs8 -cstopb -parenb -ixon -ixoff -crtscts";
  system(oss.str().c_str());

  arduino = std::fstream(control_port,
                         std::ios::in | std::ios::out | std::ios::binary);
  if (!arduino.is_open()) {
    RCLCPP_FATAL(this->get_logger(), "file not found: `%s`",
                 control_port.c_str());
  }

  thrusters_sub = this->create_subscription<Float64MultiArray>(
      "hydrus/thrusters", 10,
      [this](const Float64MultiArray::SharedPtr thrusters) {
        this->handle_thrusters_msg(thrusters);
      });
}

void Controller::handle_thrusters_msg(const Float64MultiArray::SharedPtr thruster_values) {
  for (int i = 0; i < thruster_values->data.size(); i += 1) {
    arduino << 'T' << i << ':' << thruster_values->data[i] << '\n';
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
