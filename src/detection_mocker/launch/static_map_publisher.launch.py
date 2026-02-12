from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to scenario files
    controller_pkg_dir = get_package_share_directory('controller_stonefish')
    env_scn = os.path.join(controller_pkg_dir, 'data', 'scenarios', 'hydrus_env.scn')

    return LaunchDescription([
        Node(
            package='detection_mocker',
            executable='static_map_publisher',
            name='static_map_publisher',
            output='screen',
            parameters=[{
                'scn_file_path': env_scn,
                'map_output_topic': '/map',
                'publish_once': False,  # Set to True for one-shot behavior
                'publish_rate_hz': 1.0   # Rate to republish (for transient local)
            }]
        )
    ])
