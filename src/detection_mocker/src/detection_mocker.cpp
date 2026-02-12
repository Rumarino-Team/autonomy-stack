#include "detection_mocker/detection_mocker.hpp"
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <chrono>
#include <functional>
#include <cmath>

using namespace std::chrono_literals;

namespace detection_mocker
{
  DetectionMocker::DetectionMocker() 
    : Node("detection_mocker"),
      odom_received_(false),
      fov_ready_(false),
      horizontal_fov_(0.0),
      vertical_fov_(0.0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Detection Mocker node...");

    // Load parameters
    loadParameters();

    // Parse scene files
    try
    {
      parseSceneFiles();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse scene files: %s", e.what());
      throw;
    }

    // Initialize frustum culler
    frustum_culler_ = std::make_unique<FrustumCuller>(
        min_detection_distance_,
        max_detection_distance_);

    // Create subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 10,
        std::bind(&DetectionMocker::odometryCallback, this, std::placeholders::_1));

    // Create publisher
    map_pub_ = this->create_publisher<interfaces::msg::Map>(map_output_topic_, 10);

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&DetectionMocker::publishMap, this));

    RCLCPP_INFO(this->get_logger(), "Detection Mocker initialized successfully!");
    RCLCPP_INFO(this->get_logger(), "  - Publishing to: %s at %.1f Hz",
                map_output_topic_.c_str(), publish_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "  - Static objects: %zu", static_objects_.size());
    RCLCPP_INFO(this->get_logger(), "  - Detection range: %.1f - %.1f meters",
                min_detection_distance_, max_detection_distance_);
  }

  void DetectionMocker::loadParameters()
  {
    // Declare and get parameters with defaults
    this->declare_parameter("scn_file_path", "");
    this->declare_parameter("robot_scn_file_path", "");
    this->declare_parameter("odometry_topic", "/hydrus/odometry");
    this->declare_parameter("map_output_topic", "/map");
    this->declare_parameter("publish_rate_hz", 10.0);
    this->declare_parameter("min_detection_distance", 0.1);
    this->declare_parameter("max_detection_distance", 50.0);

    scn_file_path_ = this->get_parameter("scn_file_path").as_string();
    robot_scn_file_path_ = this->get_parameter("robot_scn_file_path").as_string();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    map_output_topic_ = this->get_parameter("map_output_topic").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    min_detection_distance_ = this->get_parameter("min_detection_distance").as_double();
    max_detection_distance_ = this->get_parameter("max_detection_distance").as_double();

    // Validate required parameters
    if (scn_file_path_.empty())
    {
      throw std::runtime_error("Required parameter 'scn_file_path' not set!");
    }
    if (robot_scn_file_path_.empty())
    {
      throw std::runtime_error("Required parameter 'robot_scn_file_path' not set!");
    }
  }

  void DetectionMocker::parseSceneFiles()
  {
    RCLCPP_INFO(this->get_logger(), "Parsing scene files...");

    // Parse static objects from environment scene
    static_objects_ = XMLParser::parseStaticObjects(scn_file_path_);

    // Parse camera configuration from robot scene
    camera_config_ = std::make_unique<CameraConfig>(XMLParser::parseCameraConfig(robot_scn_file_path_));

    if (camera_config_->resolution_x > 0 && camera_config_->resolution_y > 0)
    {
      horizontal_fov_ = camera_config_->horizontal_fov_rad;
      double aspect_ratio = static_cast<double>(camera_config_->resolution_y) /
                            static_cast<double>(camera_config_->resolution_x);
      vertical_fov_ = 2.0 * std::atan(std::tan(horizontal_fov_ / 2.0) * aspect_ratio);
      fov_ready_ = true;
      RCLCPP_INFO(this->get_logger(), "Camera specs loaded from scene file");
      RCLCPP_INFO(this->get_logger(), "  - Resolution: %dx%d",
                  camera_config_->resolution_x, camera_config_->resolution_y);
      RCLCPP_INFO(this->get_logger(), "  - Horizontal FOV: %.1f degrees",
                  horizontal_fov_ * 180.0 / M_PI);
      RCLCPP_INFO(this->get_logger(), "  - Vertical FOV: %.1f degrees",
                  vertical_fov_ * 180.0 / M_PI);
    }

    RCLCPP_INFO(this->get_logger(), "Scene parsing complete!");
  }

  void DetectionMocker::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
    odom_received_ = true;

  }

  void DetectionMocker::publishMap()
  {
    // Check if we have required data
    if (!odom_received_ || !fov_ready_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for odometry and camera specs... (odom: %s, cam: %s)",
                           odom_received_ ? "OK" : "waiting",
                           fov_ready_ ? "OK" : "waiting");
      return;
    }

    interfaces::msg::Map map_msg;

    // Set map bounds (covering entire simulation area based on hydrus_env.scn)
    // Objects range roughly: x[-3, 20], y[0, 2], z[0, 3]
    map_msg.map_bounds.center.position.x = 8.5;
    map_msg.map_bounds.center.position.y = 1.0;
    map_msg.map_bounds.center.position.z = 1.5;
    map_msg.map_bounds.center.orientation.w = 1.0;
    map_msg.map_bounds.size.x = 25.0;
    map_msg.map_bounds.size.y = 3.0;
    map_msg.map_bounds.size.z = 4.0;

    // Get robot pose
    Eigen::Vector3d robot_pos = getRobotPosition();
    Eigen::Quaterniond robot_orient = getRobotOrientation();

    // Process each static object
    for (const auto &static_obj : static_objects_)
    {
      // Skip ground plane
      if (static_obj.type == ObjectType::PLANE)
      {
        continue;
      }

      // Transform object position to camera frame
      Eigen::Vector3d point_cam = Transforms::worldToCameraFrame(
          static_obj.position,
          robot_pos,
          robot_orient,
          camera_config_->offset,
          getCameraRotationMatrix());

      // Check if object is visible in camera frustum
      if (frustum_culler_->isVisible(point_cam, horizontal_fov_, vertical_fov_))
      {
        // Create MapObject
        interfaces::msg::MapObject map_obj;
        map_obj.cls = classifyObject(static_obj.name);
        map_obj.bbox = objectToBoundingBox(static_obj);

        map_msg.objects.push_back(map_obj);

      }
    }

    // Publish map
    map_pub_->publish(map_msg);

  }

  Eigen::Vector3d DetectionMocker::getRobotPosition() const
  {
    if (!latest_odom_)
    {
      return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(
        latest_odom_->pose.pose.position.x,
        latest_odom_->pose.pose.position.y,
        latest_odom_->pose.pose.position.z);
  }

  Eigen::Quaterniond DetectionMocker::getRobotOrientation() const
  {
    if (!latest_odom_)
    {
      return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(
        latest_odom_->pose.pose.orientation.w,
        latest_odom_->pose.pose.orientation.x,
        latest_odom_->pose.pose.orientation.y,
        latest_odom_->pose.pose.orientation.z);
  }

  Eigen::Matrix3d DetectionMocker::getCameraRotationMatrix() const
  {
    return Transforms::rpyToRotationMatrix(camera_config_->rotation_rpy);
  }

  int32_t DetectionMocker::classifyObject(const std::string &name) const
  {
    // Classification based on mission_executor ObjectCls enum
    // 0: Cube, 1: Rectangle/Gate, 2: Marker, 3: Unknown

    if (name == "Box")
    {
      return 0; // Cube
    }
    else if (name.find("Gate") != std::string::npos)
    {
      return 1; // Rectangle/Gate
    }
    else if (name == "Marker")
    {
      return 2; // Marker/Cylinder
    }
    else if (name == "Ground")
    {
      return -1; // Should be filtered out, but just in case
    }

    return 3; // Unknown
  }

  vision_msgs::msg::BoundingBox3D DetectionMocker::objectToBoundingBox(const StaticObject &obj) const
  {
    vision_msgs::msg::BoundingBox3D bbox;

    // Set center position
    bbox.center.position.x = obj.position.x();
    bbox.center.position.y = obj.position.y();
    bbox.center.position.z = obj.position.z();

    // Set orientation from RPY
    Eigen::Quaterniond quat = Transforms::rpyToQuaternion(obj.rotation_rpy);
    bbox.center.orientation.x = quat.x();
    bbox.center.orientation.y = quat.y();
    bbox.center.orientation.z = quat.z();
    bbox.center.orientation.w = quat.w();

    // Set size based on object type
    switch (obj.type)
    {
    case ObjectType::BOX:
      bbox.size.x = obj.dimensions.x();
      bbox.size.y = obj.dimensions.y();
      bbox.size.z = obj.dimensions.z();
      break;

    case ObjectType::CYLINDER:
      // Approximate cylinder as bounding box
      bbox.size.x = 2.0 * obj.dimensions.x(); // diameter (2 * radius)
      bbox.size.y = 2.0 * obj.dimensions.x(); // diameter (2 * radius)
      bbox.size.z = obj.dimensions.y();       // height
      break;

    case ObjectType::MODEL:
      bbox.size.x = obj.dimensions.x();
      bbox.size.y = obj.dimensions.y();
      bbox.size.z = obj.dimensions.z();
      break;

    case ObjectType::PLANE:
      // Should never reach here
      bbox.size.x = 100.0;
      bbox.size.y = 100.0;
      bbox.size.z = 0.1;
      break;
    }

    return bbox;
  }

} // namespace detection_mocker
