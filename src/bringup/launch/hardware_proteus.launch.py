import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    mission_name_arg = DeclareLaunchArgument('mission_name')

    arduino_port_arg = DeclareLaunchArgument('arduino_port')
    arduino_baud_rate_arg = DeclareLaunchArgument('arduino_baud_rate')
    use_usb_cam_arg = DeclareLaunchArgument('use_usb_cam', default_value='false')
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link')
    camera_frame_arg = DeclareLaunchArgument('camera_frame', default_value='camera_link')

    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.0')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0.0')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.0')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='0.0')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0')

    mission_name = LaunchConfiguration(mission_name_arg.name)

    arduino_port = LaunchConfiguration(arduino_port_arg.name)
    arduino_baud_rate = LaunchConfiguration(arduino_baud_rate_arg.name)
    use_usb_cam = LaunchConfiguration(use_usb_cam_arg.name)
    base_frame = LaunchConfiguration(base_frame_arg.name)
    camera_frame = LaunchConfiguration(camera_frame_arg.name)
    camera_x = LaunchConfiguration(camera_x_arg.name)
    camera_y = LaunchConfiguration(camera_y_arg.name)
    camera_z = LaunchConfiguration(camera_z_arg.name)
    camera_roll = LaunchConfiguration(camera_roll_arg.name)
    camera_pitch = LaunchConfiguration(camera_pitch_arg.name)
    camera_yaw = LaunchConfiguration(camera_yaw_arg.name)

    return LaunchDescription([
        mission_name_arg,

        arduino_port_arg,
        arduino_baud_rate_arg,
        use_usb_cam_arg,
        base_frame_arg,
        camera_frame_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,

        Node(
            package='mission_executor',
            executable='mission_executor',
            parameters=[{
                'mission_name': mission_name,
                'bridge_name': 'hardware',
                'auv_name': 'proteus',
                'live_config_path': os.path.join(
                    os.getcwd(), 'src', 'bringup', 'config', 'mission_executor.toml'
                ),
            }],
        ),
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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            output='screen',
            arguments=['--frame-id', base_frame,
                       '--child-frame-id', camera_frame,
                       '--x', camera_x,
                       '--y', camera_y,
                       '--z', camera_z,
                       '--roll', camera_roll,
                       '--pitch', camera_pitch,
                       '--yaw', camera_yaw],
            condition=IfCondition(use_usb_cam),
        ),
    ])
