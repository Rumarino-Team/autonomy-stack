#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/map.hpp>
#include <interfaces/msg/map_object.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <chrono>

#include "detection_mocker/xml_parser.hpp"
#include "detection_mocker/types.hpp"
#include "detection_mocker/transforms.hpp"

using namespace detection_mocker;
using namespace std::chrono_literals;

class StaticMapPublisher : public rclcpp::Node
{
public:
  StaticMapPublisher() : Node("static_map_publisher")
  {
    // Declare parameters
    this->declare_parameter<std::string>("scn_file_path", "");
    this->declare_parameter<std::string>("map_output_topic", "/map");
    this->declare_parameter<bool>("publish_once", false);
    this->declare_parameter<double>("publish_rate_hz", 1.0);

    // Get parameters
    scn_file_path_ = this->get_parameter("scn_file_path").as_string();
    std::string map_topic = this->get_parameter("map_output_topic").as_string();
    bool publish_once = this->get_parameter("publish_once").as_bool();
    double publish_rate = this->get_parameter("publish_rate_hz").as_double();

    if (scn_file_path_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Required parameter 'scn_file_path' not set!");
      throw std::runtime_error("Required parameter 'scn_file_path' not set!");
    }

    // Parse scene file
    RCLCPP_INFO(this->get_logger(), "Parsing scene file: %s", scn_file_path_.c_str());
    try
    {
      static_objects_ = XMLParser::parseStaticObjects(scn_file_path_);
      RCLCPP_INFO(this->get_logger(), "Successfully parsed %zu static objects", static_objects_.size());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse scene file: %s", e.what());
      throw;
    }

    // Create publisher with transient local QoS for late joiners
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    map_pub_ = this->create_publisher<interfaces::msg::Map>(map_topic, qos);

    // Build and publish map
    buildMap();
    map_pub_->publish(map_msg_);
    RCLCPP_INFO(this->get_logger(), "Published static map with %zu objects to %s", 
                map_msg_.objects.size(), map_topic.c_str());

    if (!publish_once)
    {
      // Keep publishing periodically
      int timer_ms = static_cast<int>(1000.0 / publish_rate);
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(timer_ms),
          std::bind(&StaticMapPublisher::publishMap, this));
      
      RCLCPP_INFO(this->get_logger(), "Publishing at %.1f Hz (use publish_once:=true for one-shot)", publish_rate);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "One-shot mode: map published once");
    }
  }

private:
  void buildMap()
  {
    map_msg_.objects.clear();

    // Find map bounds from all objects
    if (static_objects_.empty())
    {
      // Default bounds
      map_msg_.map_bounds.center.position.x = 0.0;
      map_msg_.map_bounds.center.position.y = 0.0;
      map_msg_.map_bounds.center.position.z = 0.0;
      map_msg_.map_bounds.center.orientation.w = 1.0;
      map_msg_.map_bounds.size.x = 100.0;
      map_msg_.map_bounds.size.y = 100.0;
      map_msg_.map_bounds.size.z = 10.0;
      return;
    }

    // Calculate bounds from objects
    Eigen::Vector3d min_pos = static_objects_[0].position;
    Eigen::Vector3d max_pos = static_objects_[0].position;

    for (const auto &obj : static_objects_)
    {
      // Skip planes as they don't contribute to bounds meaningfully
      if (obj.type == ObjectType::PLANE)
        continue;

      Eigen::Vector3d half_size = obj.dimensions * 0.5;
      min_pos = min_pos.cwiseMin(obj.position - half_size);
      max_pos = max_pos.cwiseMax(obj.position + half_size);
    }

    Eigen::Vector3d center = (min_pos + max_pos) * 0.5;
    Eigen::Vector3d size = max_pos - min_pos;

    // Add some padding
    size += Eigen::Vector3d(10.0, 10.0, 2.0);

    map_msg_.map_bounds.center.position.x = center.x();
    map_msg_.map_bounds.center.position.y = center.y();
    map_msg_.map_bounds.center.position.z = center.z();
    map_msg_.map_bounds.center.orientation.w = 1.0;
    map_msg_.map_bounds.center.orientation.x = 0.0;
    map_msg_.map_bounds.center.orientation.y = 0.0;
    map_msg_.map_bounds.center.orientation.z = 0.0;
    map_msg_.map_bounds.size.x = size.x();
    map_msg_.map_bounds.size.y = size.y();
    map_msg_.map_bounds.size.z = size.z();

    // Convert all objects to MapObjects
    for (const auto &obj : static_objects_)
    {
      // Skip planes - they're not detectable objects
      if (obj.type == ObjectType::PLANE)
        continue;

      interfaces::msg::MapObject map_obj;
      map_obj.cls = classifyObject(obj.name);
      map_obj.bbox = objectToBoundingBox(obj);

      map_msg_.objects.push_back(map_obj);
    }
  }

  void publishMap()
  {
    map_pub_->publish(map_msg_);
  }

  int32_t classifyObject(const std::string &name) const
  {
    // Simple classification based on name
    if (name.find("gate") != std::string::npos || name.find("Gate") != std::string::npos)
      return 2; // Gate
    if (name.find("buoy") != std::string::npos || name.find("Buoy") != std::string::npos)
      return 0; // Buoy
    if (name.find("pipe") != std::string::npos || name.find("Pipe") != std::string::npos)
      return 1; // Pipe
    
    return 3; // Unknown
  }

  vision_msgs::msg::BoundingBox3D objectToBoundingBox(const StaticObject &obj) const
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

  std::string scn_file_path_;
  std::vector<StaticObject> static_objects_;
  interfaces::msg::Map map_msg_;
  
  rclcpp::Publisher<interfaces::msg::Map>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  try
  {
    auto node = std::make_shared<StaticMapPublisher>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("static_map_publisher"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
