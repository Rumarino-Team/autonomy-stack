#include <sstream>
#include <string>

#include <bridge_proteus.hh>

BridgeProteus::BridgeProteus() : Node("bridge_proteus") {
  this->declare_parameter<std::string>("arduino_port");
  this->declare_parameter<int>("arduino_baud_rate", 115200);

  std::string arduino_port;
  if (!this->get_parameter("arduino_port", arduino_port)) {
    RCLCPP_FATAL(this->get_logger(), "arduino_port not set");
  }

  int arduino_baud_rate;
  this->get_parameter("arduino_baud_rate", arduino_baud_rate);

  // Setup Arduino serial
  {
    std::ostringstream oss;
    oss << "stty -F " << arduino_port << " " << arduino_baud_rate
        << " cs8 -cstopb -parenb -ixon -ixoff -crtscts";
    system(oss.str().c_str());

    arduino = std::fstream(arduino_port,
                           std::ios::in | std::ios::out | std::ios::binary);

    if (!arduino.is_open()) {
      RCLCPP_FATAL(this->get_logger(), "cannot open arduino: %s",
                   arduino_port.c_str());
    }
  }

  thrusters_sub = this->create_subscription<Float64MultiArray>(
      "/bridge/thrusters", 10,
      [this](const Float64MultiArray::SharedPtr thrusters) {
        this->handle_thrusters_msg(thrusters);
      });

  arduino_rx_timer = this->create_wall_timer(
      std::chrono::milliseconds(20), [this]() { this->poll_arduino_serial(); });
}

void BridgeProteus::handle_thrusters_msg(
    const Float64MultiArray::SharedPtr thruster_values) {
  for (size_t i = 0; i < thruster_values->data.size(); i++) {
    arduino << 'T' << i << ':' << thruster_values->data[i] << '\n';
  }
  arduino.flush();
}

void BridgeProteus::poll_arduino_serial() {
  if (!arduino.is_open())
    return;

  char chunk[256];
  const std::streamsize bytes_read = arduino.readsome(chunk, sizeof(chunk));
  if (bytes_read <= 0) {
    arduino.clear();
    return;
  }

  arduino_rx_buffer.append(chunk, static_cast<size_t>(bytes_read));

  size_t newline_pos = arduino_rx_buffer.find('\n');
  while (newline_pos != std::string::npos) {
    std::string line = arduino_rx_buffer.substr(0, newline_pos);
    arduino_rx_buffer.erase(0, newline_pos + 1);

    if (!line.empty() && line.back() == '\r')
      line.pop_back();

    if (!line.empty()) {
      RCLCPP_INFO(this->get_logger(), "[arduino] %s", line.c_str());
    }

    newline_pos = arduino_rx_buffer.find('\n');
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeProteus>());
  rclcpp::shutdown();
  return 0;
}
