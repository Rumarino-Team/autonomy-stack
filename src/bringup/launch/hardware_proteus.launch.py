import os
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mission_name_arg = DeclareLaunchArgument('mission_name')

    arduino_port_arg = DeclareLaunchArgument('arduino_port')
    arduino_baud_rate_arg = DeclareLaunchArgument('arduino_baud_rate')
    use_vectornav_arg = DeclareLaunchArgument('use_vectornav', default_value='true')
    use_usb_cam_arg = DeclareLaunchArgument('use_usb_cam', default_value='false')
    use_orb_slam_arg = DeclareLaunchArgument('use_orb_slam', default_value='false')

    mission_name = LaunchConfiguration(mission_name_arg.name)

    arduino_port = LaunchConfiguration(arduino_port_arg.name)
    arduino_baud_rate = LaunchConfiguration(arduino_baud_rate_arg.name)
    use_vectornav = LaunchConfiguration(use_vectornav_arg.name)
    use_usb_cam = LaunchConfiguration(use_usb_cam_arg.name)
    use_orb_slam = LaunchConfiguration(use_orb_slam_arg.name)

    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vectornav'),
                'launch',
                'vectornav.launch.py',
            ])
        ),
        condition=IfCondition(use_vectornav),
    )

    orb_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orb_slam3_ros2'),
                'launch',
                'orb_slam_sim_launch.py',
            ])
        ),
        launch_arguments={
            'use_viewer': 'false',
            'use_imu': 'true',
            'use_depth': 'false',
            'image_topic': '/usb_cam/image_raw',
            'imu_topic': '/vectornav/imu',
        }.items(),
        condition=IfCondition(use_orb_slam),
    )

    return LaunchDescription([
        mission_name_arg,

        arduino_port_arg,
        arduino_baud_rate_arg,
        use_vectornav_arg,
        use_usb_cam_arg,
        use_orb_slam_arg,

        Node(
            package='mission_executor',
            executable='mission_executor',
            parameters=[{
                'mission_name': mission_name,
                'bridge_name': 'hardware',
                'auv_name': 'proteus',
                # 'live_config_path': PathJoinSubstitution([FindPackageShare('bringup'), 'config', 'mission_executor.toml']),
                'live_config_path': os.path.join(
                    os.getcwd(), 'src', 'bringup', 'config', 'mission_executor.toml'
                ),
            }],
        ),
        # TODO: some sort vision stuff
        Node(
            package='bridge_hardware',
            executable='bridge_proteus_node',
            parameters=[{
                'arduino_port': arduino_port,
                'arduino_baud_rate': arduino_baud_rate,
            }],
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='usb_cam',
            output='screen',
            condition=IfCondition(use_usb_cam),
        ),

        vectornav_launch,
        orb_slam_launch,
    ])
