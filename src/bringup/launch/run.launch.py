from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    mission_name = LaunchConfiguration('mission_name')
    controller_name = LaunchConfiguration('controller_name')
    control_port = LaunchConfiguration('control_port')
    baud_rate = LaunchConfiguration('baud_rate')

    return LaunchDescription([
        # TODO: some sort vision stuff
        Node(
            package='mission_executor',
            executable='mission_executor',
            parameters=[{
                'mission_name': mission_name,
                'controller_name': controller_name,
            }],
        ),
        Node(
            package='controller_arduino',
            executable='controller_node',
            parameters=[{
                'control_port': control_port,
                'baud_rate': baud_rate,
                'controller_name': controller_name,
            }],
        ),
    ])


# import os
#
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
#
#
# def generate_launch_description():
#     bringup_share = get_package_share_directory("bringup")
#     default_params = os.path.join(bringup_share, "config", "vn100.yaml")
#
#     params_file_arg = DeclareLaunchArgument(
#         "params_file",
#         default_value=default_params,
#         description="Path to the VN100 parameter file",
#     )
#     port_arg = DeclareLaunchArgument(
#         "port",
#         default_value="/dev/ttyUSB0",
#         description="Serial device for VN100",
#     )
#     baud_arg = DeclareLaunchArgument(
#         "baud",
#         default_value="115200",
#         description="Serial baud rate",
#     )
#
#     params_file = LaunchConfiguration("params_file")
#     port = LaunchConfiguration("port")
#     baud = LaunchConfiguration("baud")
#     vectornav_node = Node(
#         package="vectornav",
#         executable="vectornav",
#         output="screen",
#         parameters=[params_file, {"port": port, "baud": baud}],
#     )
#
#     vn_sensor_msgs_node = Node(
#         package="vectornav",
#         executable="vn_sensor_msgs",
#         output="screen",
#         parameters=[params_file],
#     )
#     return LaunchDescription([
#         params_file_arg,
#         port_arg,
#         baud_arg,
#         parent_frame_arg,
#         vectornav_node,
#         vn_sensor_msgs_node,
#     ])
