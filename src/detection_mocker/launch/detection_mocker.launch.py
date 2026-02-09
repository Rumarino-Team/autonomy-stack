from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to scenario files
    controller_pkg_dir = get_package_share_directory('controller_stonefish')
    env_scn = os.path.join(controller_pkg_dir, 'data', 'scenarios', 'hydrus_env.scn')
    robot_scn = os.path.join(controller_pkg_dir, 'data', 'scenarios', 'hydrus_auv.scn')

    return LaunchDescription([
        Node(
            package='detection_mocker',
            executable='detection_mocker',
            name='detection_mocker',
            output='screen',
            prefix='gdbserver localhost:3000',  # Debug server on port 3000
            parameters=[{
                'scn_file_path': env_scn,
                'robot_scn_file_path': robot_scn,
                'odometry_topic': '/hydrus/odometry',
                'camera_info_topic': '/hydrus/camera/camera_info',
                'map_output_topic': '/map',
                'publish_rate_hz': 10.0,
                'min_detection_distance': 0.1,
                'max_detection_distance': 50.0,
                'debug_mode': True
            }],
            remappings=[
                # Add remappings if needed
            ]
        )
    ])
