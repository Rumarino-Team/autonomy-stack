from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    mission_name = LaunchConfiguration('mission_name', default='prequalify')
    env_file_name = LaunchConfiguration('env_file_name', default='hydrus_env_headless.scn')

    env_scn_path = PathJoinSubstitution([
        FindPackageShare('controller_stonefish'), 'data', 'scenarios', env_file_name
    ])

    return LaunchDescription([
        # Stonefish headless (nogpu)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'),
                    'launch',
                    'stonefish_simulator_nogpu.launch.py'
                ])
            ]),
            launch_arguments={
                'simulation_data': PathJoinSubstitution([FindPackageShare('controller_stonefish'), 'data']),
                'scenario_desc': env_scn_path,
                'simulation_rate': '300.0',
            }.items()
        ),
        # Detection mocker — publishes /hydrus/map from scene objects
        Node(
            package='detection_mocker',
            executable='detection_mocker',
            parameters=[{
                'scn_file_path': env_scn_path,
                'robot_scn_file_path': PathJoinSubstitution([
                    FindPackageShare('controller_stonefish'), 'data', 'scenarios', 'hydrus_auv.scn'
                ]),
                'odometry_topic': '/hydrus/odometry',
                'map_output_topic': '/hydrus/map',
                'publish_all_objects': True,
            }],
        ),
        # Mission executor — navigates the AUV based on map data
        Node(
            package='mission_executor',
            executable='mission_executor',
            parameters=[{
                'mission_name': mission_name,
            }],
        ),
        # Volume trigger checker — monitors AUV pose vs. trigger zones
        Node(
            package='sim_test_harness',
            executable='volume_trigger_node',
            parameters=[{
                'scn_file_path': env_scn_path,
            }],
        ),
    ])
