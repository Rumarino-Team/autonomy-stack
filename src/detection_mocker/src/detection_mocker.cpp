#include "detection_mocker/detection_mocker.hpp"
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
      static_map_published_(false),
      horizontal_fov_(0.0),
      vertical_fov_(0.0),
      markers_published_(false)
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

    // Create publishers
    rclcpp::QoS map_qos(10);
    map_qos.transient_local();
    map_pub_ = this->create_publisher<interfaces::msg::Map>(map_output_topic_, map_qos);
    
    // Create marker publisher with transient local for static visualization
    rclcpp::QoS marker_qos(10);
    marker_qos.transient_local();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        map_output_topic_ + "_markers", marker_qos);

    // Publish static markers once (after marker_pub_ is created)
    publishStaticMarkers();

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
    this->declare_parameter("publish_all_objects", false);

    scn_file_path_ = this->get_parameter("scn_file_path").as_string();
    robot_scn_file_path_ = this->get_parameter("robot_scn_file_path").as_string();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    map_output_topic_ = this->get_parameter("map_output_topic").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    min_detection_distance_ = this->get_parameter("min_detection_distance").as_double();
    max_detection_distance_ = this->get_parameter("max_detection_distance").as_double();
    publish_all_objects_ = this->get_parameter("publish_all_objects").as_bool();

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

    // Compute map bounds from static objects
    map_bounds_ = XMLParser::computeMapBounds(static_objects_);
    RCLCPP_INFO(this->get_logger(), "Map bounds - Center: (%.2f, %.2f, %.2f), Size: (%.2f, %.2f, %.2f)",
                map_bounds_.center.x(), map_bounds_.center.y(), map_bounds_.center.z(),
                map_bounds_.size.x(), map_bounds_.size.y(), map_bounds_.size.z());

    RCLCPP_INFO(this->get_logger(), "Scene parsing complete!");
  }

  void DetectionMocker::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
    odom_received_ = true;

  }

  void DetectionMocker::publishMap()
  {
    if (publish_all_objects_)
    {
      publishStaticMap();
      return;
    }

    // Check if we have required data
    if (!odom_received_ || !fov_ready_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for required data... (odom: %s, cam: %s)",
                           odom_received_ ? "OK" : "waiting",
                           fov_ready_ ? "OK" : "waiting");
      return;
    }

    interfaces::msg::Map map_msg;

    // Set map bounds from computed AABB
    map_msg.map_bounds.center.position.x = map_bounds_.center.x();
    map_msg.map_bounds.center.position.y = map_bounds_.center.y();
    map_msg.map_bounds.center.position.z = map_bounds_.center.z();
    map_msg.map_bounds.center.orientation.w = 1.0;
    map_msg.map_bounds.size.x = map_bounds_.size.x();
    map_msg.map_bounds.size.y = map_bounds_.size.y();
    map_msg.map_bounds.size.z = map_bounds_.size.z();

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
          Transforms::rpyToRotationMatrix(camera_config_->rotation_rpy));

      // Check if object is visible in camera frustum
      if (frustum_culler_->isVisible(point_cam, horizontal_fov_, vertical_fov_))
      {
        // Create MapObject
        interfaces::msg::MapObject map_obj;
        map_obj.cls = static_cast<int32_t>(static_obj.cls);

        // Create bounding box
        vision_msgs::msg::BoundingBox3D bbox;
        bbox.center.position.x = static_obj.position.x();
        bbox.center.position.y = static_obj.position.y();
        bbox.center.position.z = static_obj.position.z();

        Eigen::Quaterniond quat = Transforms::rpyToQuaternion(static_obj.rotation_rpy);
        bbox.center.orientation.x = quat.x();
        bbox.center.orientation.y = quat.y();
        bbox.center.orientation.z = quat.z();
        bbox.center.orientation.w = quat.w();

        // Set size based on object type
        switch (static_obj.type)
        {
        case ObjectType::BOX:
        case ObjectType::MODEL:
          bbox.size.x = static_obj.dimensions.x();
          bbox.size.y = static_obj.dimensions.y();
          bbox.size.z = static_obj.dimensions.z();
          break;

        case ObjectType::CYLINDER:
          bbox.size.x = 2.0 * static_obj.dimensions.x();
          bbox.size.y = 2.0 * static_obj.dimensions.x();
          bbox.size.z = static_obj.dimensions.y();
          break;

        case ObjectType::PLANE:
          bbox.size.x = 100.0;
          bbox.size.y = 100.0;
          bbox.size.z = 0.1;
          break;
        }

        map_obj.bbox = bbox;
        map_msg.objects.push_back(map_obj);
      }
    }

    // Publish map
    map_pub_->publish(map_msg);

  }

  void DetectionMocker::publishStaticMap()
  {
    if (static_map_published_)
    {
      return; // Already published
    }

    interfaces::msg::Map map_msg;

    // Set map bounds from computed AABB
    map_msg.map_bounds.center.position.x = map_bounds_.center.x();
    map_msg.map_bounds.center.position.y = map_bounds_.center.y();
    map_msg.map_bounds.center.position.z = map_bounds_.center.z();
    map_msg.map_bounds.center.orientation.w = 1.0;
    map_msg.map_bounds.size.x = map_bounds_.size.x();
    map_msg.map_bounds.size.y = map_bounds_.size.y();
    map_msg.map_bounds.size.z = map_bounds_.size.z();

    for (const auto &static_obj : static_objects_)
    {
      // Skip ground plane
      if (static_obj.type == ObjectType::PLANE)
      {
        continue;
      }

      interfaces::msg::MapObject map_obj;
      map_obj.cls = static_cast<int32_t>(static_obj.cls);

      vision_msgs::msg::BoundingBox3D bbox;
      bbox.center.position.x = static_obj.position.x();
      bbox.center.position.y = static_obj.position.y();
      bbox.center.position.z = static_obj.position.z();

      Eigen::Quaterniond quat = Transforms::rpyToQuaternion(static_obj.rotation_rpy);
      bbox.center.orientation.x = quat.x();
      bbox.center.orientation.y = quat.y();
      bbox.center.orientation.z = quat.z();
      bbox.center.orientation.w = quat.w();

      switch (static_obj.type)
      {
      case ObjectType::BOX:
      case ObjectType::MODEL:
        bbox.size.x = static_obj.dimensions.x();
        bbox.size.y = static_obj.dimensions.y();
        bbox.size.z = static_obj.dimensions.z();
        break;

      case ObjectType::CYLINDER:
        bbox.size.x = 2.0 * static_obj.dimensions.x();
        bbox.size.y = 2.0 * static_obj.dimensions.x();
        bbox.size.z = static_obj.dimensions.y();
        break;

      case ObjectType::PLANE:
        bbox.size.x = 100.0;
        bbox.size.y = 100.0;
        bbox.size.z = 0.1;
        break;
      }

      map_obj.bbox = bbox;
      map_msg.objects.push_back(map_obj);
    }

    map_pub_->publish(map_msg);
    static_map_published_ = true;

    RCLCPP_INFO(this->get_logger(), "Published static map with %zu objects", map_msg.objects.size());
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

  void DetectionMocker::publishStaticMarkers()
  {
    if (markers_published_)
    {
      return; // Already published
    }

    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    for (const auto &static_obj : static_objects_)
    {
      // Skip ground plane
      if (static_obj.type == ObjectType::PLANE)
      {
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world_ned";
      marker.header.stamp = this->now();
      marker.ns = "static_objects";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Set position
      marker.pose.position.x = static_obj.position.x();
      marker.pose.position.y = static_obj.position.y();
      marker.pose.position.z = static_obj.position.z();

      // Set orientation
      Eigen::Quaterniond quat = Transforms::rpyToQuaternion(static_obj.rotation_rpy);
      marker.pose.orientation.x = quat.x();
      marker.pose.orientation.y = quat.y();
      marker.pose.orientation.z = quat.z();
      marker.pose.orientation.w = quat.w();

      // Set scale based on object type
      switch (static_obj.type)
      {
      case ObjectType::BOX:
      case ObjectType::MODEL:
        marker.scale.x = static_obj.dimensions.x();
        marker.scale.y = static_obj.dimensions.y();
        marker.scale.z = static_obj.dimensions.z();
        break;

      case ObjectType::CYLINDER:
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.scale.x = 2.0 * static_obj.dimensions.x();
        marker.scale.y = 2.0 * static_obj.dimensions.x();
        marker.scale.z = static_obj.dimensions.y();
        break;

      case ObjectType::PLANE:
        // Should not reach here
        continue;
      }

      // Set color based on class type
      marker.color.a = 0.5; // Semi-transparent
      switch (static_obj.cls)
      {
      case ClassType::GATE:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        break;
      case ClassType::BOUY:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case ClassType::PATH:
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        break;
      case ClassType::BIND:
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case ClassType::SHARK:
        marker.color.r = 0.5;
        marker.color.g = 0.0;
        marker.color.b = 0.5;
        break;
      case ClassType::SWORDFISH:
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        break;
      }

      marker.lifetime = rclcpp::Duration::from_seconds(0); // Persistent

      marker_array.markers.push_back(marker);
    }

    // Publish markers
    marker_pub_->publish(marker_array);
    markers_published_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Published %zu static object markers", marker_array.markers.size());
  }

} // namespace detection_mocker
