from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='activity_control_pkg',
            executable='route_target_publisher_node',
            name='route_target_publisher',
            output='screen',
            parameters=[{
                'position_tolerance_cm': 9.0,
                'yaw_tolerance_deg': 5.0,
                'height_tolerance_cm': 12.0,
                'map_frame': 'map',
                'laser_link_frame': 'laser_link',
                'output_topic': '/target_position',
                'visual_align_pixel_threshold': 100.0,
                'visual_align_required_frames': 3,
                'visual_takeover_timeout_sec': 5.0,
                'fine_data_stale_timeout_sec': 0.5,
            }]
        )
    ])
