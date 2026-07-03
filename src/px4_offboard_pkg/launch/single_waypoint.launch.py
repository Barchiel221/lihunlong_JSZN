import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_offboard_pkg')
    config = os.path.join(pkg_share, 'config', 'single_waypoint.yaml')

    enable_arm_arg = DeclareLaunchArgument(
        'enable_arm', default_value='false',
        description='True 才会真切 Offboard / arm。第一次跑务必 false 做 dry-run')

    return LaunchDescription([
        enable_arm_arg,
        Node(
            package='px4_offboard_pkg',
            executable='offboard_node',
            name='px4_offboard_waypoint',
            output='screen',
            parameters=[
                config,
                {'enable_arm': LaunchConfiguration('enable_arm')},
            ],
            emulate_tty=True,
        ),
    ])
