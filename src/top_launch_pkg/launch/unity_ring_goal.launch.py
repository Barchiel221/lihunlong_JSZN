from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ring_topic_arg = DeclareLaunchArgument(
        'ring_topic',
        default_value='/unity/target_rings',
        description='Set this after confirming the Unity UI PoseArray topic with rostopic list/info')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/Odometry')
    goal_topic_arg = DeclareLaunchArgument('goal_topic', default_value='/move_base_simple/goal')

    sequencer = Node(
        package='top_launch_pkg',
        executable='pose_array_goal_sequencer.py',
        name='pose_array_goal_sequencer',
        output='screen',
        parameters=[{
            'ring_topic': LaunchConfiguration('ring_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'frame_id': 'map',
            'reach_xy_tolerance': 0.35,
            'reach_z_tolerance': 0.30,
            'republish_period_sec': 1.0,
            'auto_start': True,
        }],
    )

    return LaunchDescription([ring_topic_arg, odom_topic_arg, goal_topic_arg, sequencer])
