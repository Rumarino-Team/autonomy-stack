import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# --- discoverable configuration (also validated via --show-args choices) ---

MISSIONS = (
    'prequalify',
    'teleop',
    'drop_into_box',
    'cardinal_directions',
)

AUVS = (
    'hydrus',
    'proteus',
    'bluerov2',
    'girona500',
)

ENV_FILES = (
    'auto',
    'hydrus_env.scn',
    'hydrus_env_headless.scn',
    'proteus_env.scn',
    'pool_env.scn',
    'pool_env_hydrus.scn',
    'pool_env_proteus.scn',
    'pool_env_bluerov2.scn',
    'pool_env_girona500.scn',
    'bluerov2_tank.scn',
)

DEFAULT_AUV_FILE_BY_NAME = {
    'bluerov2': 'bluerov2.scn',
    'proteus': 'proteus_auv.scn',
    'hydrus': 'hydrus_auv.scn',
    'girona500': 'girona500_auv.scn',
}

DEFAULT_ENV_BY_AUV = {
    'hydrus': 'hydrus_env.scn',
    'proteus': 'proteus_env.scn',
    'bluerov2': 'pool_env.scn',
    'girona500': 'pool_env.scn',
}

POOL_ENV_SCENARIO_BY_AUV = {
    'bluerov2': 'pool_env_bluerov2.scn',
    'proteus': 'pool_env_proteus.scn',
    'hydrus': 'pool_env_hydrus.scn',
    'girona500': 'pool_env_girona500.scn',
}


def _resolve_auv_file_name(auv_name, explicit_auv_file_name):
    if explicit_auv_file_name:
        return explicit_auv_file_name
    return DEFAULT_AUV_FILE_BY_NAME.get(auv_name, f'{auv_name}.scn')


def _resolve_env_file_name(auv_name, env_file_name):
    if env_file_name in ('', 'auto'):
        return DEFAULT_ENV_BY_AUV.get(auv_name, f'{auv_name}_env.scn')
    return env_file_name


def _launch_setup(context, *args, **kwargs):
    mission_name = LaunchConfiguration('mission_name').perform(context)
    auv_name = LaunchConfiguration('auv_name').perform(context)
    env_file_name = _resolve_env_file_name(
        auv_name,
        LaunchConfiguration('env_file_name').perform(context),
    )
    explicit_auv_file_name = LaunchConfiguration('auv_file_name').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    stonefish_only = LaunchConfiguration('stonefish_only', default="no").perform(context).lower() in ('true', '1', 'yes')
    fast_fixed_step_s = LaunchConfiguration('fast_fixed_step').perform(context)
    fast_fixed_step = fast_fixed_step_s.lower() in ('true', '1', 'yes')
    use_joy = LaunchConfiguration('use_joy').perform(context).lower() in ('true', '1', 'yes')

    # Sim-time stamps and RTF cap are for fast_fixed_step runs. Real-time graphical
    # sim uses wall-clock odometry stamps (stonefish_ros2 default when fast=false).
    use_sim_time_stamps = LaunchConfiguration('use_sim_time_stamps').perform(context)
    if not fast_fixed_step:
        use_sim_time_stamps = 'false'
    realtime_factor_cap = LaunchConfiguration('realtime_factor_cap').perform(context)
    if not fast_fixed_step:
        realtime_factor_cap = '0.0'

    cwd = os.getcwd()
    bridge_share = get_package_share_directory('bridge_stonefish')
    stonefish_share = get_package_share_directory('stonefish_ros2')

    resolved_auv_file_name = _resolve_auv_file_name(auv_name, explicit_auv_file_name)
    auv_file_path = os.path.join(bridge_share, 'data', 'scenarios', resolved_auv_file_name)
    resolved_env_file_name = env_file_name
    if env_file_name == 'pool_env.scn':
        resolved_env_file_name = POOL_ENV_SCENARIO_BY_AUV.get(auv_name, f'pool_env_{auv_name}.scn')
    scenario_desc_path = os.path.join(bridge_share, 'data', 'scenarios', resolved_env_file_name)
    detection_env_file_name = env_file_name
    if resolved_env_file_name in POOL_ENV_SCENARIO_BY_AUV.values():
        detection_env_file_name = 'pool_env.scn'
    detection_scn_path = os.path.join(bridge_share, 'data', 'scenarios', detection_env_file_name)

    # Evaluate in this launch context. Passing unevaluated LaunchConfiguration
    # into IncludeLaunchDescription can resolve against stonefish_ros2's own
    # defaults (use_sim_time_stamps=false).
    simulation_rate = LaunchConfiguration('simulation_rate').perform(context)

    simulator_launch = 'stonefish_simulator_nogpu.launch.py' if headless else 'stonefish_simulator.launch.py'
    simulator_arguments = {
        'simulation_data': os.path.join(bridge_share, 'data'),
        'scenario_desc': scenario_desc_path,
        'simulation_rate': simulation_rate,
        'fast_fixed_step': fast_fixed_step_s,
        'use_sim_time_stamps': use_sim_time_stamps,
        'realtime_factor_cap': realtime_factor_cap,
    }
    if not headless:
        # High-quality 1080p plus camera/IMU publish at 100x realtime adds
        # wall-clock delay that becomes seconds of plant delay in sim time.
        simulator_arguments.update({
            'window_res_x': '1280' if fast_fixed_step else '1920',
            'window_res_y': '720' if fast_fixed_step else '1080',
            'rendering_quality': 'low' if fast_fixed_step else 'high',
        })

    # Setup ROS2 workspace based on current working directory
    ros_setup = os.path.join(cwd, 'install', 'setup.bash')

    # Common command
    common_cmd = (
        f'source {ros_setup} && ros2 run mission_executor mission_executor '
        f'--ros-args -p mission_name:={mission_name} '
        f'-p bridge_name:=stonefish '
        f'-p auv_name:={auv_name} '
        f'-p live_config_path:={os.path.join(cwd, "src/bringup/config/mission_executor.toml")}; '
        f'if [ $? -ne 0 ]; then exec bash; fi'
    )

    # Terminal prefix
    if 'TMUX' in os.environ:
        terminal_prefix = f'tmux split-window -v -- bash -c "{common_cmd}"'
    else:
        term = os.environ.get('TERMINAL', 'xterm')
        terminal_prefix = f'{term} -e bash -c "{common_cmd}"'

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
                'scn_file_path': detection_scn_path,
                'robot_scn_file_path': auv_file_path,
                'mesh_base_path': os.path.join(bridge_share, 'data'),
                'odometry_topic': '/vision/odometry',
                'map_output_topic': '/vision/map',
                'publish_all_objects': True,
            }],
        ),
    ]
    if use_joy:
        ret.append(Node(
            package='joy',
            executable='joy_node',
        ))
    if not stonefish_only:
        ret += [
            Node(
                package='mission_executor',
                executable='mission_executor',
                emulate_tty=True,
                output='screen',
                prefix=terminal_prefix,
                parameters=[{
                    'mission_name': mission_name,
                    'bridge_name': 'stonefish',
                    'auv_name': auv_name,
                    'live_config_path': os.path.join(cwd, 'src', 'bringup', 'config', 'mission_executor.toml'),
                }],
            ),
        ]
    return ret


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_name',
            default_value='prequalify',
            description='Mission loaded by mission_executor.',
            choices=list(MISSIONS),
        ),
        DeclareLaunchArgument(
            'auv_name',
            default_value='hydrus',
            description='AUV model; selects default scenario and mesh when env/auv files are omitted.',
            choices=list(AUVS),
        ),
        DeclareLaunchArgument(
            'env_file_name',
            default_value='auto',
            description=(
                'Stonefish environment scenario under bridge_stonefish/data/scenarios/. '
                'auto picks the default for auv_name. pool_env.scn auto-picks the per-AUV pool file.'
            ),
            choices=list(ENV_FILES),
        ),
        DeclareLaunchArgument(
            'auv_file_name',
            default_value='',
            description=(
                'Override AUV mesh scenario. Empty uses the default for auv_name '
                f'({", ".join(f"{k}→{v}" for k, v in DEFAULT_AUV_FILE_BY_NAME.items())}).'
            ),
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Stonefish without a GPU window (nogpu launch file).',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'stonefish_only',
            default_value='false',
            description='Skip mission_executor; publish thrusters on /bridge/thrusters for direct testing.',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'use_joy',
            default_value='false',
            description='Start joy_node (install with scripts/install_deps.sh --with-joy). Required for teleop.',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'simulation_rate',
            default_value='300.0',
            description='Stonefish physics update rate (Hz).',
        ),
        DeclareLaunchArgument(
            'fast_fixed_step',
            default_value='false',
            description='Run physics as fast as possible with fixed substeps.',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'use_sim_time_stamps',
            default_value='false',
            description=(
                'Stamp odometry with simulation time (for fast_fixed_step). '
                'Automatically forced false for real-time sim.'
            ),
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'realtime_factor_cap',
            default_value='0.0',
            description=(
                'Max sim-time/wall-time ratio when fast_fixed_step is on. '
                'Automatically 0 (disabled) for real-time sim. CI uses 5.0.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
