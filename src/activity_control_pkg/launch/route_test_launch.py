from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='activity_control_pkg',
            executable='route_test_node',
            name='route_test_node',
            output='screen',
        )
    ])
