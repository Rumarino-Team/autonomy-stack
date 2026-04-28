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
    orb_use_viewer_arg = DeclareLaunchArgument('orb_use_viewer', default_value='false')
    world_frame_arg = DeclareLaunchArgument('world_frame', default_value='world')
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link')
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value='vectornav')
    camera_frame_arg = DeclareLaunchArgument('camera_frame', default_value='camera_link')

    imu_x_arg = DeclareLaunchArgument('imu_x', default_value='0.0')
    imu_y_arg = DeclareLaunchArgument('imu_y', default_value='0.0')
    imu_z_arg = DeclareLaunchArgument('imu_z', default_value='0.0')
    imu_roll_arg = DeclareLaunchArgument('imu_roll', default_value='0.0')
    imu_pitch_arg = DeclareLaunchArgument('imu_pitch', default_value='0.0')
    imu_yaw_arg = DeclareLaunchArgument('imu_yaw', default_value='0.0')

    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.0')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0.0')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.0')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='0.0')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0')

    mission_name = LaunchConfiguration(mission_name_arg.name)

    arduino_port = LaunchConfiguration(arduino_port_arg.name)
    arduino_baud_rate = LaunchConfiguration(arduino_baud_rate_arg.name)
    use_vectornav = LaunchConfiguration(use_vectornav_arg.name)
    use_usb_cam = LaunchConfiguration(use_usb_cam_arg.name)
    use_orb_slam = LaunchConfiguration(use_orb_slam_arg.name)
    orb_use_viewer = LaunchConfiguration(orb_use_viewer_arg.name)
    world_frame = LaunchConfiguration(world_frame_arg.name)
    base_frame = LaunchConfiguration(base_frame_arg.name)
    imu_frame = LaunchConfiguration(imu_frame_arg.name)
    camera_frame = LaunchConfiguration(camera_frame_arg.name)
    imu_x = LaunchConfiguration(imu_x_arg.name)
    imu_y = LaunchConfiguration(imu_y_arg.name)
    imu_z = LaunchConfiguration(imu_z_arg.name)
    imu_roll = LaunchConfiguration(imu_roll_arg.name)
    imu_pitch = LaunchConfiguration(imu_pitch_arg.name)
    imu_yaw = LaunchConfiguration(imu_yaw_arg.name)
    camera_x = LaunchConfiguration(camera_x_arg.name)
    camera_y = LaunchConfiguration(camera_y_arg.name)
    camera_z = LaunchConfiguration(camera_z_arg.name)
    camera_roll = LaunchConfiguration(camera_roll_arg.name)
    camera_pitch = LaunchConfiguration(camera_pitch_arg.name)
    camera_yaw = LaunchConfiguration(camera_yaw_arg.name)

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
            'use_viewer': orb_use_viewer,
            'use_imu': 'true',
            'use_depth': 'false',
            'settings_file': PathJoinSubstitution([
                FindPackageShare('orb_slam3_ros2'),
                'config',
                'webcamera.yaml',
            ]),
            'image_topic': '/usb_cam/image_raw',
            'imu_topic': '/vectornav/imu',
            'world_frame_id': world_frame,
            'camera_frame_id': camera_frame,
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
        orb_use_viewer_arg,
        world_frame_arg,
        base_frame_arg,
        imu_frame_arg,
        camera_frame_arg,
        imu_x_arg,
        imu_y_arg,
        imu_z_arg,
        imu_roll_arg,
        imu_pitch_arg,
        imu_yaw_arg,
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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            output='screen',
            arguments=['--frame-id', base_frame,
                       '--child-frame-id', imu_frame,
                       '--x', imu_x,
                       '--y', imu_y,
                       '--z', imu_z,
                       '--roll', imu_roll,
                       '--pitch', imu_pitch,
                       '--yaw', imu_yaw],
            condition=IfCondition(use_vectornav),
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

        vectornav_launch,
        orb_slam_launch,
    ])
