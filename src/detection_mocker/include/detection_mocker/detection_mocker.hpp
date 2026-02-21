#ifndef DETECTION_MOCKER__DETECTION_MOCKER_HPP_
#define DETECTION_MOCKER__DETECTION_MOCKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <interfaces/msg/map.hpp>
#include <interfaces/msg/map_object.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

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
    void publishStaticMap();
    void publishStaticMarkers();
    interfaces::msg::MapObject createMapObject(const StaticObject &static_obj) const;
    bool buildMarkerFromStaticObject(
        const StaticObject &static_obj,
        const std::string &marker_ns,
        int marker_id,
        float alpha,
        float red,
        float green,
        float blue,
        bool use_class_color,
        visualization_msgs::msg::Marker &marker) const;
    size_t publishMarkers(
        const std::vector<const StaticObject *> &objects,
        const std::string &marker_ns,
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &publisher,
        float alpha,
        float red,
        float green,
        float blue,
        bool use_class_color) const;
    void publishVisibleMarkers(const std::vector<const StaticObject *> &visible_objects);
    void publishDetectedMarkers();

    // Parsed static data
    std::vector<StaticObject> static_objects_;
    CameraConfig camera_config_;
    MapBounds map_bounds_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<interfaces::msg::Map>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visible_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detected_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Detected-object memory
    std::vector<const StaticObject *> detected_map_objects_;
    std::vector<bool> detected_object_flags_;

    // Latest sensor data
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;

    // State flags
    bool odom_received_;
    bool fov_ready_;
    bool static_map_published_;
    bool markers_published_;

    // Camera parameters
    double horizontal_fov_;
    double vertical_fov_;

    FrustumCuller frustum_culler_;

    // ROS parameters
    std::string scn_file_path_;
    std::string robot_scn_file_path_;
    std::string mesh_base_path_;
    std::string odometry_topic_;
    std::string map_output_topic_;
    double publish_rate_hz_;
    double min_detection_distance_;
    double max_detection_distance_;
    bool publish_all_objects_;
};

}  // namespace detection_mocker

#endif  // DETECTION_MOCKER__DETECTION_MOCKER_HPP_
