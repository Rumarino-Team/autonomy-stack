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
    fast_fixed_step_s = LaunchConfiguration('fast_fixed_step').perform(context)
    use_sim_time_stamps = LaunchConfiguration('use_sim_time_stamps').perform(context)
    realtime_factor_cap = LaunchConfiguration('realtime_factor_cap').perform(context)
    fast_fixed_step = fast_fixed_step_s.lower() in ('true', '1', 'yes')

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
    if not headless:
        ret.append(Node(
            package='joy',
            executable='joy_node',
        ))
    if not stonefish_only:
        # xterm/tmux startup is ~1s of wall time. With fast_fixed_step that is
        # minutes of uncommanded sim before the PID starts.
        me_kwargs = {
            'package': 'mission_executor',
            'executable': 'mission_executor',
            'emulate_tty': True,
            'output': 'screen',
            'parameters': [{
                'mission_name': mission_name,
                'bridge_name': 'stonefish',
                'auv_name': auv_name,
                'live_config_path': os.path.join(cwd, 'src', 'bringup', 'config', 'mission_executor.toml'),
            }],
        }
        if mission_name == 'teleop':
            me_kwargs['prefix'] = terminal_prefix
        ret += [Node(**me_kwargs)]
    return ret


def generate_launch_description():
    mission_name_arg = DeclareLaunchArgument('mission_name')
    auv_name_arg = DeclareLaunchArgument('auv_name')

    env_file_name_arg = DeclareLaunchArgument('env_file_name')
    auv_file_name_arg = DeclareLaunchArgument('auv_file_name', default_value='')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    stonefish_only_arg = DeclareLaunchArgument('stonefish_only', default_value='false')
    simulation_rate_arg = DeclareLaunchArgument('simulation_rate', default_value='300.0')
    fast_fixed_step_arg = DeclareLaunchArgument('fast_fixed_step', default_value='false')
    # Stamp odometry with simulation time so PID dt stays correct when the
    # sim runs faster than wall clock (fast_fixed_step:=true).
    use_sim_time_stamps_arg = DeclareLaunchArgument('use_sim_time_stamps', default_value='true')
    # Only applies when fast_fixed_step is on; real-time stepping is already 1x.
    # Keeps thruster command delay a fraction of a control period instead of
    # seconds of plant time. 0.0 disables the cap.
    realtime_factor_cap_arg = DeclareLaunchArgument('realtime_factor_cap', default_value='5.0')

    return LaunchDescription([
        mission_name_arg,
        auv_name_arg,
        env_file_name_arg,
        auv_file_name_arg,
        headless_arg,
        stonefish_only_arg,
        simulation_rate_arg,
        fast_fixed_step_arg,
        use_sim_time_stamps_arg,
        realtime_factor_cap_arg,
        OpaqueFunction(function=_launch_setup),
    ])
