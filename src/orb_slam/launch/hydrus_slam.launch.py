from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    orb_slam_share = get_package_share_directory("orb_slam3_ros2")
    
    # Declare launch arguments
    use_viewer = LaunchConfiguration("use_viewer")
    use_imu = LaunchConfiguration("use_imu")
    use_depth = LaunchConfiguration("use_depth")
    
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
        
        # Include Stonefish simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('controller_stonefish'),
                    'launch',
                    'hydrussim.launch.py'
                ])
            ])
        ),
        
        # Thruster Teleop Node
        Node(
            package="controller_stonefish",
            executable="thruster_teleop",
            name="thruster_teleop",
            output="screen"
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
                    # Stonefish Hydrus simulation topics
                    "image_topic": "/hydrus/rgb_camera/image_color",
                    "depth_topic": "/hydrus/depth_camera/image_depth",
                    "imu_topic": "/hydrus/imu",
                    # Output topics
                    "pose_topic": "orb_slam3/camera_pose",
                    "odom_topic": "orb_slam3/camera_odom",
                    "path_topic": "orb_slam3/camera_path",
                    # Frame IDs
                    "world_frame_id": "world_ned",
                    "camera_frame_id": "hydrus_camera",
                    "queue_size": 10,
                    "publish_tf": True,
                }
            ]
        ),
    ])
