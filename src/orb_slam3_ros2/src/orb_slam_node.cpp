/**
 * ROS2 SLAM Node for ORB-SLAM3
 *
 * Configurable sensor modes:
 * - MONOCULAR mode (default)
 * - RGB-D mode (if use_depth parameter is true)
 *
 * Publishes:
 * - Camera pose (geometry_msgs/PoseStamped)
 * - Camera odometry (nav_msgs/Odometry)
 * - Camera trajectory path (nav_msgs/Path)
 * - TF transforms (camera pose in world frame)
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

class SlamNode : public rclcpp::Node
{
public:
    SlamNode()
    : Node("orb_slam3_node")
    {
        // Declare and get ROS parameters
        this->declare_parameter<std::string>("vocabulary_path", "");
        this->declare_parameter<std::string>("settings_path", "");
        this->declare_parameter<bool>("use_viewer", true);
        this->declare_parameter<bool>("use_depth", false);  // Enable RGB-D mode
        this->declare_parameter<bool>("use_imu", false);  // Enable IMU-Inertial mode
        
        this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("pose_topic", "orb_slam3/camera_pose");
        this->declare_parameter<std::string>("odom_topic", "orb_slam3/camera_odom");
        this->declare_parameter<std::string>("path_topic", "orb_slam3/camera_path");
        this->declare_parameter<std::string>("world_frame_id", "world");
        this->declare_parameter<std::string>("camera_frame_id", "camera");
        this->declare_parameter<int>("queue_size", 10);
        this->declare_parameter<bool>("publish_tf", true);

        // Get parameters
        std::string vocabulary_path = this->get_parameter("vocabulary_path").as_string();
        std::string settings_path = this->get_parameter("settings_path").as_string();
        bool use_viewer = this->get_parameter("use_viewer").as_bool();
        use_depth_ = this->get_parameter("use_depth").as_bool();
        use_imu_ = this->get_parameter("use_imu").as_bool();
        
        std::string image_topic = this->get_parameter("image_topic").as_string();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string path_topic = this->get_parameter("path_topic").as_string();
        world_frame_id_ = this->get_parameter("world_frame_id").as_string();
        camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
        int queue_size = this->get_parameter("queue_size").as_int();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        // Validate required parameters
        if (vocabulary_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'vocabulary_path' is required!");
            throw std::runtime_error("Missing vocabulary_path parameter");
        }
        if (settings_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'settings_path' is required!");
            throw std::runtime_error("Missing settings_path parameter");
        }

        // Determine sensor mode
        ORB_SLAM3::System::eSensor sensor_mode;
        if (use_depth_) {
            sensor_mode = ORB_SLAM3::System::RGBD;
            RCLCPP_INFO(this->get_logger(), "=== RGB-D SLAM MODE ===");
        } else if (use_imu_) {
            sensor_mode = ORB_SLAM3::System::IMU_MONOCULAR;
            RCLCPP_INFO(this->get_logger(), "=== MONOCULAR-INERTIAL SLAM MODE ===");
        } else {
            sensor_mode = ORB_SLAM3::System::MONOCULAR;
            RCLCPP_INFO(this->get_logger(), "=== MONOCULAR SLAM MODE ===");
        }

        RCLCPP_INFO(this->get_logger(), "Vocabulary: %s", vocabulary_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Settings: %s", settings_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Use viewer: %s", use_viewer ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic.c_str());
        if (use_depth_) {
            RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_topic.c_str());
        }
        if (use_imu_) {
            RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Pose topic: %s", pose_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Odom topic: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Path topic: %s", path_topic.c_str());

        // Initialize ORB-SLAM3
        RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3...");
        slam_system_ = std::make_shared<ORB_SLAM3::System>(
            vocabulary_path,
            settings_path,
            sensor_mode,
            use_viewer
        );
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 initialized!");

        // Create subscribers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic,
            queue_size,
            std::bind(&SlamNode::imageCallback, this, std::placeholders::_1)
        );

        if (use_depth_) {
            depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                depth_topic,
                queue_size,
                std::bind(&SlamNode::depthCallback, this, std::placeholders::_1)
            );
        }

        if (use_imu_) {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                queue_size,
                std::bind(&SlamNode::imuCallback, this, std::placeholders::_1)
            );
        }

        // Create publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, queue_size);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, queue_size);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, queue_size);

        // TF broadcaster
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

        // Initialize path message
        path_msg_.header.frame_id = world_frame_id_;

        RCLCPP_INFO(this->get_logger(), "SLAM node ready!");
    }

    ~SlamNode()
    {
        if (slam_system_) {
            RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3...");
            slam_system_->Shutdown();

            // Save trajectory
            slam_system_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
            slam_system_->SaveTrajectoryTUM("CameraTrajectory.txt");
            RCLCPP_INFO(this->get_logger(), "Trajectories saved!");
        }
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        // Buffer IMU messages for preintegration between frames
        imu_buffer_.push_back(imu_msg);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr depth_msg)
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        
        // Store latest depth image
        try {
            latest_depth_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Depth cv_bridge exception: %s", e.what());
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Get timestamp
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        Sophus::SE3f Tcw_se3;
        
        // Use appropriate tracking mode
        if (use_depth_) {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            
            if (latest_depth_) {
                // RGB-D tracking
                Tcw_se3 = slam_system_->TrackRGBD(cv_ptr->image, latest_depth_->image, timestamp);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "RGB-D mode enabled but no depth data received yet");
                return;
            }
        } else if (use_imu_) {
            // IMU-Monocular tracking with preintegration
            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                
                // Convert buffered ROS IMU messages to ORB-SLAM3 format
                for (const auto& imu_msg : imu_buffer_) {
                    double imu_timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
                    
                    // Create IMU measurement (acceleration, angular velocity, timestamp)
                    ORB_SLAM3::IMU::Point imu_point(
                        imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z,
                        imu_msg->angular_velocity.x,
                        imu_msg->angular_velocity.y,
                        imu_msg->angular_velocity.z,
                        imu_timestamp
                    );
                    vImuMeas.push_back(imu_point);
                }
                
                // Clear buffer after processing
                imu_buffer_.clear();
            }
            
            // Monocular + IMU tracking with preintegration
            Tcw_se3 = slam_system_->TrackMonocular(cv_ptr->image, timestamp, vImuMeas);
            
            if (!vImuMeas.empty()) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Using %zu IMU measurements for tracking", vImuMeas.size());
            }
        } else {
            // Monocular tracking (no IMU)
            Tcw_se3 = slam_system_->TrackMonocular(cv_ptr->image, timestamp);
        }

        // Check if tracking succeeded
        const float eps = 1e-6f;
        if (Tcw_se3.translation().norm() < eps &&
            Tcw_se3.rotationMatrix().isApprox(Eigen::Matrix3f::Identity(), 1e-6f)) {
            return;  // Tracking failed
        }

        // Convert Sophus::SE3f to cv::Mat
        cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
        Eigen::Matrix3f R = Tcw_se3.rotationMatrix();
        Eigen::Vector3f t = Tcw_se3.translation();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Tcw.at<float>(r, c) = R(r, c);
        Tcw.at<float>(0, 3) = t(0);
        Tcw.at<float>(1, 3) = t(1);
        Tcw.at<float>(2, 3) = t(2);

        // Publish pose
        publishPose(Tcw, msg->header.stamp);
    }

    void publishPose(const cv::Mat& Tcw, const builtin_interfaces::msg::Time& timestamp)
    {
        // Convert camera pose to world frame
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0,3).col(3);

        // Create pose message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = world_frame_id_;

        pose_msg.pose.position.x = twc.at<float>(0);
        pose_msg.pose.position.y = twc.at<float>(1);
        pose_msg.pose.position.z = twc.at<float>(2);

        // Convert rotation to quaternion
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
                          Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
                          Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);

        Eigen::Quaternionf q(rotation_matrix);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // Publish
        pose_pub_->publish(pose_msg);

        // Add to path
        path_msg_.header.stamp = timestamp;
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);

        // Publish odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = world_frame_id_;
        odom_msg.child_frame_id = camera_frame_id_;
        
        // Set pose
        odom_msg.pose.pose = pose_msg.pose;
        
        // Compute velocity if we have a previous pose
        if (last_pose_time_ > 0.0) {
            double current_time = timestamp.sec + timestamp.nanosec * 1e-9;
            double dt = current_time - last_pose_time_;
            
            if (dt > 0.0 && dt < 1.0) {  // Only compute velocity if dt is reasonable
                // Linear velocity
                odom_msg.twist.twist.linear.x = (twc.at<float>(0) - last_position_[0]) / dt;
                odom_msg.twist.twist.linear.y = (twc.at<float>(1) - last_position_[1]) / dt;
                odom_msg.twist.twist.linear.z = (twc.at<float>(2) - last_position_[2]) / dt;
                
                // Angular velocity (simplified - compute from quaternion difference)
                Eigen::Quaternionf q_current(q);
                Eigen::Quaternionf q_last(last_orientation_[3], last_orientation_[0], 
                                           last_orientation_[1], last_orientation_[2]);
                Eigen::Quaternionf q_diff = q_current * q_last.inverse();
                
                // Convert quaternion to angular velocity
                Eigen::AngleAxisf aa(q_diff);
                Eigen::Vector3f angular_vel = aa.axis() * aa.angle() / dt;
                
                odom_msg.twist.twist.angular.x = angular_vel.x();
                odom_msg.twist.twist.angular.y = angular_vel.y();
                odom_msg.twist.twist.angular.z = angular_vel.z();
            }
        }
        
        // Store current pose for next iteration
        last_pose_time_ = timestamp.sec + timestamp.nanosec * 1e-9;
        last_position_[0] = twc.at<float>(0);
        last_position_[1] = twc.at<float>(1);
        last_position_[2] = twc.at<float>(2);
        last_orientation_[0] = q.x();
        last_orientation_[1] = q.y();
        last_orientation_[2] = q.z();
        last_orientation_[3] = q.w();
        
        odom_pub_->publish(odom_msg);

        // Publish TF
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = timestamp;
            transform.header.frame_id = world_frame_id_;
            transform.child_frame_id = camera_frame_id_;
            transform.transform.translation.x = twc.at<float>(0);
            transform.transform.translation.y = twc.at<float>(1);
            transform.transform.translation.z = twc.at<float>(2);
            transform.transform.rotation = pose_msg.pose.orientation;

            tf_broadcaster_->sendTransform(transform);
        }
    }

    // ORB-SLAM3
    std::shared_ptr<ORB_SLAM3::System> slam_system_;
    
    // ROS2 subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // ROS2 publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Path
    nav_msgs::msg::Path path_msg_;

    // Parameters
    std::string world_frame_id_;
    std::string camera_frame_id_;
    bool publish_tf_;
    bool use_depth_;
    bool use_imu_;
    
    // Odometry tracking for velocity computation
    double last_pose_time_ = 0.0;
    float last_position_[3] = {0.0f, 0.0f, 0.0f};
    float last_orientation_[4] = {0.0f, 0.0f, 0.0f, 1.0f};  // x, y, z, w
    
    // Depth data
    cv_bridge::CvImagePtr latest_depth_;
    std::mutex depth_mutex_;
    
    // IMU data buffer for preintegration
    std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
    std::mutex imu_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<SlamNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}