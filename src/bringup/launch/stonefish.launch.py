import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


DEFAULT_AUV_FILE_BY_NAME = {
    'bluerov2': 'bluerov2.scn',
    'proteus': 'proteus_auv.scn',
    'hydrus': 'hydrus_auv.scn',
}

POOL_ENV_SCENARIO_BY_AUV = {
    'bluerov2': 'pool_env_bluerov2.scn',
    'proteus': 'pool_env_proteus.scn',
    'hydrus': 'pool_env_hydrus.scn',
}


def _resolve_auv_file_name(auv_name, explicit_auv_file_name):
    if explicit_auv_file_name:
        return explicit_auv_file_name
    return DEFAULT_AUV_FILE_BY_NAME.get(auv_name, f'{auv_name}.scn')


def _launch_setup(context, *args, **kwargs):
    mission_name = LaunchConfiguration('mission_name').perform(context)
    auv_name = LaunchConfiguration('auv_name').perform(context)
    env_file_name = LaunchConfiguration('env_file_name').perform(context)
    explicit_auv_file_name = LaunchConfiguration('auv_file_name').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    stonefish_only = LaunchConfiguration('stonefish_only', default="no").perform(context).lower() in ('true', '1', 'yes')

    bridge_share = get_package_share_directory('bridge_stonefish')
    stonefish_share = get_package_share_directory('stonefish_ros2')

    resolved_auv_file_name = _resolve_auv_file_name(auv_name, explicit_auv_file_name)
    auv_file_path = os.path.join(bridge_share, 'data', 'scenarios', resolved_auv_file_name)
    resolved_env_file_name = env_file_name
    if env_file_name == 'pool_env.scn':
        resolved_env_file_name = POOL_ENV_SCENARIO_BY_AUV.get(auv_name, f'pool_env_{auv_name}.scn')
    scenario_desc_path = os.path.join(bridge_share, 'data', 'scenarios', resolved_env_file_name)

    simulator_launch = 'stonefish_simulator_nogpu.launch.py' if headless else 'stonefish_simulator.launch.py'
    simulator_arguments = {
        'simulation_data': os.path.join(bridge_share, 'data'),
        'scenario_desc': scenario_desc_path,
        'simulation_rate': '300.0',
    }
    if not headless:
        simulator_arguments.update({
            'window_res_x': '1920',
            'window_res_y': '1080',
            'rendering_quality': 'high',
        })

    if 'TERM' in os.environ.keys():
        terminal = os.environ['TERM']
    elif 'TERMINAL' in os.environ.keys():
        terminal = os.environ['TERMINAL']
    else:
        print("Default terminal not found, using xterm...")
        terminal = "xterm"
    terminal += ' -e'
    ret = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(stonefish_share, 'launch', simulator_launch)
            ]),
            launch_arguments=simulator_arguments.items(),
        ),
        Node(
            package='detection_mocker',
            executable='detection_mocker',
            parameters=[{
                'scn_file_path': scenario_desc_path,
                'robot_scn_file_path': auv_file_path,
                'odometry_topic': '/vision/odometry',
                'map_output_topic': '/vision/map',
                'publish_all_objects': True,
            }],
        ),
    ]
    if not stonefish_only:
        ret += [
            Node(
                package='mission_executor',
                executable='mission_executor',
                emulate_tty=True,
                output='screen',
                prefix=terminal,
                parameters=[{
                    'mission_name': mission_name,
                    'bridge_name': 'stonefish',
                    'auv_name': auv_name,
                    'live_config_path': os.path.join(
                        os.getcwd(), 'src', 'bringup', 'config', 'mission_executor.toml'
                    ),
                }],
            ),
        ]
    return ret


def generate_launch_description():
    mission_name_arg = DeclareLaunchArgument('mission_name')
    auv_name_arg = DeclareLaunchArgument('auv_name')

    env_file_name_arg = DeclareLaunchArgument('env_file_name')
    auv_file_name_arg = DeclareLaunchArgument('auv_file_name', default_value='')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    return LaunchDescription([
        mission_name_arg,
        auv_name_arg,
        env_file_name_arg,
        auv_file_name_arg,
        headless_arg,
        OpaqueFunction(function=_launch_setup),
    ])
