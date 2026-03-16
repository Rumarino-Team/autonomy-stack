from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')

    scenario_file = PythonExpression([
        "'pool_env.scn' if '", robot_type, "' == 'hydrus' else 'pool_env_girona500.scn'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type',
            default_value='hydrus',
            description="Robot to simulate: 'hydrus' or 'girona500'",
            choices=['hydrus', 'girona500'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'),
                    'launch',
                    'stonefish_simulator.launch.py'
                ])
            ]),
            launch_arguments={
                'simulation_data': PathJoinSubstitution([FindPackageShare('controller_stonefish'), 'data']),
                'scenario_desc': PathJoinSubstitution([FindPackageShare('controller_stonefish'), 'data', 'scenarios', scenario_file]),
                'simulation_rate': '300.0',
                'window_res_x': '1920',
                'window_res_y': '1080',
                'rendering_quality': 'high'
            }.items()
        ),
        # Node(
        #     package='controller_stonefish',
        #     executable='thruster_teleop',
        #     name='thruster_teleop'
        # ),
    ])
