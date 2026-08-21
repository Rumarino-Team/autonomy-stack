"""Headless Hydrus prequalify launch used by CI (fast_fixed_step)."""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Wrap stonefish.launch.py with headless CI defaults."""
    bringup_share = get_package_share_directory('bringup')

    return LaunchDescription([
        DeclareLaunchArgument('mission_name', default_value='prequalify'),
        DeclareLaunchArgument('auv_name', default_value='hydrus'),
        DeclareLaunchArgument(
            'env_file_name', default_value='hydrus_env_headless.scn'),
        DeclareLaunchArgument('use_sim_time_stamps', default_value='true'),
        DeclareLaunchArgument('fast_fixed_step', default_value='true'),
        DeclareLaunchArgument('realtime_factor_cap', default_value='5.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(bringup_share, 'launch', 'stonefish.launch.py')
            ]),
            launch_arguments={
                'headless': 'true',
                'auv_name': LaunchConfiguration('auv_name'),
                'env_file_name': LaunchConfiguration('env_file_name'),
                'mission_name': LaunchConfiguration('mission_name'),
                'use_sim_time_stamps': LaunchConfiguration(
                    'use_sim_time_stamps'),
                'fast_fixed_step': LaunchConfiguration('fast_fixed_step'),
                'realtime_factor_cap': LaunchConfiguration(
                    'realtime_factor_cap'),
            }.items(),
        ),
    ])
