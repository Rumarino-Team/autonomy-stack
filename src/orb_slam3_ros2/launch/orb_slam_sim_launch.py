from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
    world_frame_id = LaunchConfiguration("world_frame_id")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    settings_file = LaunchConfiguration("settings_file")
    use_usb_cam = LaunchConfiguration("use_usb_cam")
    use_vectornav = LaunchConfiguration("use_vectornav")
    usb_cam_video_device = LaunchConfiguration("usb_cam_video_device")
    usb_cam_framerate = LaunchConfiguration("usb_cam_framerate")
    usb_cam_io_method = LaunchConfiguration("usb_cam_io_method")
    usb_cam_pixel_format = LaunchConfiguration("usb_cam_pixel_format")
    usb_cam_image_width = LaunchConfiguration("usb_cam_image_width")
    usb_cam_image_height = LaunchConfiguration("usb_cam_image_height")
    usb_cam_camera_info_url = LaunchConfiguration("usb_cam_camera_info_url")
    usb_cam_camera_name = LaunchConfiguration("usb_cam_camera_name")
    usb_cam_brightness = LaunchConfiguration("usb_cam_brightness")
    usb_cam_contrast = LaunchConfiguration("usb_cam_contrast")
    usb_cam_saturation = LaunchConfiguration("usb_cam_saturation")
    usb_cam_sharpness = LaunchConfiguration("usb_cam_sharpness")
    usb_cam_gain = LaunchConfiguration("usb_cam_gain")
    usb_cam_auto_white_balance = LaunchConfiguration("usb_cam_auto_white_balance")
    usb_cam_white_balance = LaunchConfiguration("usb_cam_white_balance")
    usb_cam_autoexposure = LaunchConfiguration("usb_cam_autoexposure")
    usb_cam_exposure = LaunchConfiguration("usb_cam_exposure")
    usb_cam_autofocus = LaunchConfiguration("usb_cam_autofocus")
    usb_cam_focus = LaunchConfiguration("usb_cam_focus")
    
    # Load ORB-SLAM3 settings
    default_settings_file = os.path.join(orb_slam_share, "config", "stonefish_hydrus.yaml")
    vocabulary_file = os.path.join(orb_slam_share, "vendor", "ORBvoc.txt")
    
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
        DeclareLaunchArgument(
            "world_frame_id",
            default_value="world",
            description="World frame used by ORB-SLAM3"
        ),
        DeclareLaunchArgument(
            "camera_frame_id",
            default_value="hydrus_camera",
            description="Camera frame used by ORB-SLAM3"
        ),
        DeclareLaunchArgument(
            "settings_file",
            default_value=default_settings_file,
            description="ORB-SLAM3 settings YAML path"
        ),
        DeclareLaunchArgument(
            "use_usb_cam",
            default_value="false",
            description="Start usb_cam node in this launch"
        ),
        DeclareLaunchArgument(
            "use_vectornav",
            default_value="false",
            description="Start vectornav nodes in this launch"
        ),
        DeclareLaunchArgument(
            "usb_cam_video_device",
            default_value="/dev/video0",
            description="V4L2 device path for usb_cam"
        ),
        DeclareLaunchArgument(
            "usb_cam_framerate",
            default_value="30.0",
            description="Camera output frame rate (Hz)"
        ),
        DeclareLaunchArgument(
            "usb_cam_io_method",
            default_value="mmap",
            description="usb_cam io method (mmap/read/userptr)"
        ),
        DeclareLaunchArgument(
            "usb_cam_pixel_format",
            default_value="mjpeg2rgb",
            description="usb_cam pixel format"
        ),
        DeclareLaunchArgument(
            "usb_cam_image_width",
            default_value="640",
            description="usb_cam image width"
        ),
        DeclareLaunchArgument(
            "usb_cam_image_height",
            default_value="480",
            description="usb_cam image height"
        ),
        DeclareLaunchArgument(
            "usb_cam_camera_info_url",
            default_value="",
            description="CameraInfo URL (e.g. file:///tmp/camera_info.yaml)"
        ),
        DeclareLaunchArgument(
            "usb_cam_camera_name",
            default_value="laptop_webcam",
            description="Camera name published in CameraInfo"
        ),
        DeclareLaunchArgument(
            "usb_cam_brightness",
            default_value="-1",
            description="usb_cam brightness (-1 leaves driver default)"
        ),
        DeclareLaunchArgument(
            "usb_cam_contrast",
            default_value="-1",
            description="usb_cam contrast (-1 leaves driver default)"
        ),
        DeclareLaunchArgument(
            "usb_cam_saturation",
            default_value="-1",
            description="usb_cam saturation (-1 leaves driver default)"
        ),
        DeclareLaunchArgument(
            "usb_cam_sharpness",
            default_value="-1",
            description="usb_cam sharpness (-1 leaves driver default)"
        ),
        DeclareLaunchArgument(
            "usb_cam_gain",
            default_value="-1",
            description="usb_cam gain (-1 leaves driver default)"
        ),
        DeclareLaunchArgument(
            "usb_cam_auto_white_balance",
            default_value="true",
            description="Enable automatic white balance"
        ),
        DeclareLaunchArgument(
            "usb_cam_white_balance",
            default_value="4000",
            description="Manual white balance value"
        ),
        DeclareLaunchArgument(
            "usb_cam_autoexposure",
            default_value="true",
            description="Enable automatic exposure"
        ),
        DeclareLaunchArgument(
            "usb_cam_exposure",
            default_value="100",
            description="Manual exposure value"
        ),
        DeclareLaunchArgument(
            "usb_cam_autofocus",
            default_value="false",
            description="Enable autofocus"
        ),
        DeclareLaunchArgument(
            "usb_cam_focus",
            default_value="-1",
            description="Manual focus value"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("vectornav"),
                    "launch",
                    "vectornav.launch.py",
                ])
            ),
            condition=IfCondition(use_vectornav),
        ),

        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            namespace="usb_cam",
            output="screen",
            parameters=[{
                "video_device": usb_cam_video_device,
                "framerate": usb_cam_framerate,
                "io_method": usb_cam_io_method,
                "frame_id": camera_frame_id,
                "pixel_format": usb_cam_pixel_format,
                "camera_name": usb_cam_camera_name,
                "image_width": usb_cam_image_width,
                "image_height": usb_cam_image_height,
                "camera_info_url": usb_cam_camera_info_url,
                "brightness": usb_cam_brightness,
                "contrast": usb_cam_contrast,
                "saturation": usb_cam_saturation,
                "sharpness": usb_cam_sharpness,
                "gain": usb_cam_gain,
                "auto_white_balance": usb_cam_auto_white_balance,
                "white_balance": usb_cam_white_balance,
                "autoexposure": usb_cam_autoexposure,
                "exposure": usb_cam_exposure,
                "autofocus": usb_cam_autofocus,
                "focus": usb_cam_focus,
            }],
            condition=IfCondition(use_usb_cam),
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
                    "world_frame_id": world_frame_id,
                    "camera_frame_id": camera_frame_id,
                    "queue_size": 10,
                    "publish_tf": True,
                }
            ]
        ),
    ])
