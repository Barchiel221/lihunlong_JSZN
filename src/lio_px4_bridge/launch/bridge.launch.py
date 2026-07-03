import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lio_px4_bridge')
    config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    return LaunchDescription([
        Node(
            package='lio_px4_bridge',
            executable='bridge_node',
            name='lio_px4_bridge',
            output='screen',
            parameters=[config],
        ),
    ])
