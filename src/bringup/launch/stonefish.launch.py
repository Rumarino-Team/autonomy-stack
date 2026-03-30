import os
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    mission_name_arg = DeclareLaunchArgument('mission_name')
    auv_name_arg = DeclareLaunchArgument('auv_name')

    env_file_name_arg = DeclareLaunchArgument('env_file_name')
    auv_file_name_arg = DeclareLaunchArgument('auv_file_name')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    mission_name = LaunchConfiguration(mission_name_arg.name)
    auv_name = LaunchConfiguration(auv_name_arg.name)

    env_file_name = LaunchConfiguration(env_file_name_arg.name)
    auv_file_name = LaunchConfiguration(auv_file_name_arg.name)
    headless = LaunchConfiguration(headless_arg.name)

    return LaunchDescription([
        mission_name_arg,
        auv_name_arg,

        env_file_name_arg,
        headless_arg,

        Node(
            package='mission_executor',
            executable='mission_executor',
            emulate_tty=True,
            output='screen',
            prefix='xterm -e',
            parameters=[{
                'mission_name': mission_name,
                'bridge_name': 'stonefish',
                'auv_name': auv_name,
                # 'live_config_path': PathJoinSubstitution([FindPackageShare('bringup'), 'config', 'mission_executor.toml']),
                'live_config_path': os.path.join(
                    os.getcwd(), 'src', 'bringup', 'config', 'mission_executor.toml'
                ),
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare('stonefish_ros2'), 'launch', 'stonefish_simulator.launch.py'])
            ]),
            condition=UnlessCondition(headless),  # runs when headless is false
            launch_arguments={
                'simulation_data': PathJoinSubstitution(
                    [FindPackageShare('bridge_stonefish'), 'data']),
                'scenario_desc': PathJoinSubstitution(
                    [FindPackageShare('bridge_stonefish'), 'data', 'scenarios', env_file_name]),
                'simulation_rate': '300.0',
                'window_res_x': '1920',
                'window_res_y': '1080',
                'rendering_quality': 'high'
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare('stonefish_ros2'), 'launch', 'stonefish_simulator_nogpu.launch.py'])
            ]),
            condition=IfCondition(headless),  # true if headless evaluates to "true"/"1"
            launch_arguments={
                'simulation_data': PathJoinSubstitution(
                    [FindPackageShare('bridge_stonefish'), 'data']),
                'scenario_desc': PathJoinSubstitution(
                    [FindPackageShare('bridge_stonefish'), 'data', 'scenarios', env_file_name]),
                'simulation_rate': '300.0',
            }.items(),
        ),

        Node(
            package='detection_mocker',
            executable='detection_mocker',
            parameters=[{
                'scn_file_path': PathJoinSubstitution(
                    [FindPackageShare('bridge_stonefish'), 'data', 'scenarios', env_file_name]),
                'robot_scn_file_path': PathJoinSubstitution(
                    [FindPackageShare('bridge_stonefish'), 'data', 'scenarios', auv_file_name]),
                'odometry_topic': '/vision/odometry',
                'map_output_topic': '/vision/map',
                'publish_all_objects': True,
            }],
        ),
    ])
