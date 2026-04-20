from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    orb_slam_share = get_package_share_directory("orb_slam3_ros2")
    
    # Declare launch arguments
    use_viewer = LaunchConfiguration("use_viewer")
    use_imu = LaunchConfiguration("use_imu")
    use_depth = LaunchConfiguration("use_depth")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    
    # Load ORB-SLAM3 settings
    settings_file = os.path.join(orb_slam_share, "config", "stonefish_hydrus.yaml")
    vocabulary_file = "/home/cesar/autonomy-stack/vendor/ORBvoc.txt"
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            "use_viewer",
            default_value="false",
            description="Enable ORB-SLAM3 viewer"
        ),
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            description="Enable IMU-inertial SLAM mode"
        ),
        DeclareLaunchArgument(
            "use_depth",
            default_value="true",
            description="Enable RGB-D SLAM mode"
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/hydrus/rgb_camera/image_raw",
            description="Image topic consumed by ORB-SLAM3"
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/hydrus/depth_camera/image_raw",
            description="Depth topic consumed by ORB-SLAM3"
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/hydrus/imu",
            description="IMU topic consumed by ORB-SLAM3"
        ),
        
        # ORB-SLAM3 Node
        Node(
            package="orb_slam3_ros2",
            executable="orb_slam_node",
            name="orb_slam3_node",
            output="screen",
            parameters=[
                {
                    "vocabulary_path": vocabulary_file,
                    "settings_path": settings_file,
                    "use_viewer": use_viewer,
                    "use_imu": use_imu,
                    "use_depth": use_depth,
                    "image_topic": image_topic,
                    "depth_topic": depth_topic,
                    "imu_topic": imu_topic,
                    # Output topics
                    "pose_topic": "orb_slam3/camera_pose",
                    "odom_topic": "orb_slam3/camera_odom",
                    "path_topic": "orb_slam3/camera_path",
                    # Frame IDs
                    "world_frame_id": "world",
                    "camera_frame_id": "hydrus_camera",
                    "queue_size": 10,
                    "publish_tf": True,
                }
            ]
        ),
    ])