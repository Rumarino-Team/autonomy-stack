import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mission_name_arg = DeclareLaunchArgument('mission_name')

    arduino_port_arg = DeclareLaunchArgument('arduino_port')
    arduino_baud_rate_arg = DeclareLaunchArgument('arduino_baud_rate')
    vn100_port_arg = DeclareLaunchArgument('vn100_port')
    vn100_baud_rate_arg = DeclareLaunchArgument('vn100_baud_rate')

    mission_name = LaunchConfiguration(mission_name_arg.name)

    arduino_port = LaunchConfiguration(arduino_port_arg.name)
    arduino_baud_rate = LaunchConfiguration(arduino_baud_rate_arg.name)
    vn100_port = LaunchConfiguration(vn100_port_arg.name)
    vn100_baud_rate = LaunchConfiguration(vn100_baud_rate_arg.name)

    return LaunchDescription([
        mission_name_arg,

        arduino_port_arg,
        arduino_baud_rate_arg,
        vn100_port_arg,
        vn100_baud_rate_arg,

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
                'vn100_port': vn100_port,
                'vn100_baud_rate': vn100_baud_rate,
            }],
        ),
    ])
