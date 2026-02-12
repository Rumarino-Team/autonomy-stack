#ifndef DETECTION_MOCKER__DETECTION_MOCKER_HPP_
#define DETECTION_MOCKER__DETECTION_MOCKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <interfaces/msg/map.hpp>
#include <interfaces/msg/map_object.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

#include "detection_mocker/types.hpp"
#include "detection_mocker/xml_parser.hpp"
#include "detection_mocker/transforms.hpp"
#include "detection_mocker/frustum_culler.hpp"

namespace detection_mocker
{

class DetectionMocker : public rclcpp::Node
{
public:
    DetectionMocker();

private:
    // Initialization
    void parseSceneFiles();
    void loadParameters();

    // ROS callbacks
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishMap();

    // Helper methods
    Eigen::Vector3d getRobotPosition() const;
    Eigen::Quaterniond getRobotOrientation() const;
    Eigen::Matrix3d getCameraRotationMatrix() const;
    int32_t classifyObject(const std::string& name) const;
    vision_msgs::msg::BoundingBox3D objectToBoundingBox(const StaticObject& obj) const;

    // Parsed static data
    std::vector<StaticObject> static_objects_;
    std::unique_ptr<CameraConfig> camera_config_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<interfaces::msg::Map>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Latest sensor data
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;

    // State flags
    bool odom_received_;
    bool fov_ready_;

    // Camera parameters
    double horizontal_fov_;
    double vertical_fov_;

    // Frustum culler
    std::unique_ptr<FrustumCuller> frustum_culler_;

    // ROS parameters
    std::string scn_file_path_;
    std::string robot_scn_file_path_;
    std::string odometry_topic_;
    std::string map_output_topic_;
    double publish_rate_hz_;
    double min_detection_distance_;
    double max_detection_distance_;
};

}  // namespace detection_mocker

#endif  // DETECTION_MOCKER__DETECTION_MOCKER_HPP_
