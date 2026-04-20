#include <sstream>
#include <vector>

#include <bridge_proteus.hh>

std::string checksum(const std::string &cmd) {
  uint8_t cs = 0;

  for (size_t i = 1; i < cmd.size(); ++i)
    cs ^= cmd[i];

  std::stringstream ss;
  ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
     << (int)cs;

  return ss.str();
}

BridgeProteus::BridgeProteus() : Node("bridge_proteus") {
  this->declare_parameter<std::string>("arduino_port");
#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  this->declare_parameter<std::string>("vn100_port", "");
#endif
  this->declare_parameter<int>("arduino_baud_rate", 115200);
#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  this->declare_parameter<int>("vn100_baud_rate", 115200);
#endif

  std::string arduino_port;
  if (!this->get_parameter("arduino_port", arduino_port)) {
    RCLCPP_FATAL(this->get_logger(), "arduino_port not set");
  }

#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  std::string vn100_port;
  this->get_parameter("vn100_port", vn100_port);
#endif

  int arduino_baud_rate;
  this->get_parameter("arduino_baud_rate", arduino_baud_rate);

#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  int vn100_baud_rate;
  this->get_parameter("vn100_baud_rate", vn100_baud_rate);
#endif

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

  // Setup VN-100 serial
#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  if (!vn100_port.empty()) {
    std::ostringstream oss;
    oss << "stty -F " << vn100_port << " " << vn100_baud_rate
        << " cs8 -cstopb -parenb -ixon -ixoff -crtscts";
    system(oss.str().c_str());

    vn100 = std::fstream(vn100_port,
                         std::ios::in | std::ios::out | std::ios::binary);

    if (!vn100.is_open()) {
      RCLCPP_FATAL(this->get_logger(), "cannot open vn100: %s",
                   vn100_port.c_str());
    }

    std::string cmd;

    // stop async
    cmd = "$VNASY,0";
    vn100 << cmd << "*" << checksum(cmd) << "\r\n";

    // VNIMU + VNQTN at 100 Hz
    cmd = "$VNWRG,75,1,8,15,0001,000C,0014";
    vn100 << cmd << "*" << checksum(cmd) << "\r\n";

    // resume async
    cmd = "$VNASY,1";
    vn100 << cmd << "*" << checksum(cmd) << "\r\n";

    // save
    cmd = "$VNWNV";
    vn100 << cmd << "*" << checksum(cmd) << "\r\n";

    vn100.flush();

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    vn100_enabled = true;
  } else {
    RCLCPP_INFO(this->get_logger(),
                "vn100_port not provided; VN-100 integration disabled");
  }
#endif

  thrusters_sub = this->create_subscription<Float64MultiArray>(
      "/bridge/thrusters", 10,
      [this](const Float64MultiArray::SharedPtr thrusters) {
        this->handle_thrusters_msg(thrusters);
      });

#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  if (vn100_enabled) {
    imu_pub = this->create_publisher<ImuMsg>("/bridge/imu", 10);
    magnetic_pub = this->create_publisher<MagneticFieldMsg>(
        "/bridge/magnetic_field", 10);
    pressure_pub =
        this->create_publisher<FluidPressureMsg>("/bridge/fluid_pressure", 10);

    vn_timer = this->create_wall_timer(std::chrono::milliseconds(5),
                                       [this]() { this->read_vn100(); });
  }
#endif
}

void BridgeProteus::handle_thrusters_msg(
    const Float64MultiArray::SharedPtr thruster_values) {
  for (size_t i = 0; i < thruster_values->data.size(); i++) {
    arduino << 'T' << i << ':' << thruster_values->data[i] << '\n';
  }
}

void BridgeProteus::read_vn100() {
#ifdef BRIDGE_HARDWARE_ENABLE_VN100
  if (!vn100_enabled)
    return;

  std::string line;
  if (!std::getline(vn100, line))
    return;

  // Static storage for latest values
  static double qx = 0, qy = 0, qz = 0, qw = 0;
  static double ax = 0, ay = 0, az = 0;
  static double gx = 0, gy = 0, gz = 0;
  static double mx = 0, my = 0, mz = 0;
  static double pressure = 0;

  // Split line by commas once
  std::vector<std::string> parts;
  size_t start = 0;
  for (size_t i = 0; i <= line.size(); ++i) {
    if (i == line.size() || line[i] == ',') {
      parts.push_back(line.substr(start, i - start));
      start = i + 1;
    }
  }

  if (parts.empty())
    return;

  const std::string &prefix = parts[0];

  try {
    if (prefix == "$VNQTN" && parts.size() >= 5) {
      std::string clean_qw = parts[4];
      size_t star = clean_qw.find('*');
      if (star != std::string::npos)
        clean_qw = clean_qw.substr(0, star);

      qx = std::stod(parts[1]);
      qy = std::stod(parts[2]);
      qz = std::stod(parts[3]);
      qw = std::stod(clean_qw);

    } else if (prefix == "$VNIMU" && parts.size() >= 12) {
      auto clean_field = [](const std::string &s) -> std::string {
        size_t star = s.find('*');
        return star == std::string::npos ? s : s.substr(0, star);
      };

      mx = std::stod(parts[1]);
      my = std::stod(parts[2]);
      mz = std::stod(clean_field(parts[3]));

      ax = std::stod(parts[4]);
      ay = std::stod(parts[5]);
      az = std::stod(clean_field(parts[6]));

      gx = std::stod(parts[7]);
      gy = std::stod(parts[8]);
      gz = std::stod(clean_field(parts[9]));

      pressure = std::stod(clean_field(parts[11]));

      // Timestamp
      auto stamp = this->now();

      // IMU message
      ImuMsg imu_msg;
      imu_msg.header.stamp = stamp;
      imu_msg.header.frame_id = "vn100";
      imu_msg.orientation.x = qx;
      imu_msg.orientation.y = qy;
      imu_msg.orientation.z = qz;
      imu_msg.orientation.w = qw;
      imu_msg.linear_acceleration.x = ax;
      imu_msg.linear_acceleration.y = ay;
      imu_msg.linear_acceleration.z = az;
      imu_msg.angular_velocity.x = gx;
      imu_msg.angular_velocity.y = gy;
      imu_msg.angular_velocity.z = gz;

      imu_pub->publish(imu_msg);

      // Pressure message
      FluidPressureMsg pressure_msg;
      pressure_msg.header.stamp = stamp;
      pressure_msg.header.frame_id = "vn100";
      pressure_msg.fluid_pressure = pressure;
      pressure_msg.variance = 0.0;
      pressure_pub->publish(pressure_msg);

      // Magnetic field message
      MagneticFieldMsg magnetic_msg;
      magnetic_msg.header.stamp = stamp;
      magnetic_msg.header.frame_id = "vn100";
      magnetic_msg.magnetic_field.x = mx;
      magnetic_msg.magnetic_field.y = my;
      magnetic_msg.magnetic_field.z = mz;
      magnetic_msg.magnetic_field_covariance[0] = 0.0;
      magnetic_msg.magnetic_field_covariance[4] = 0.0;
      magnetic_msg.magnetic_field_covariance[8] = 0.0;
      magnetic_pub->publish(magnetic_msg);
    }
  } catch (...) {
    // silently ignore malformed data
    return;
  }
#endif
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeProteus>());
  rclcpp::shutdown();
  return 0;
}
