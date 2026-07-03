from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    wait_subscribers = LaunchConfiguration('wait_subscribers')

    goal_msg = [
        '{header: {frame_id: map}, pose: {position: {x: ',
        x,
        ', y: ',
        y,
        ', z: ',
        z,
        '}, orientation: {w: 1.0}}}',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='2.5'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.8'),
        DeclareLaunchArgument(
            'wait_subscribers',
            default_value='2',
            description='Wait for this many /move_base_simple/goal subscribers before publishing.',
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'topic',
                'pub',
                '-1',
                '-w',
                wait_subscribers,
                '--keep-alive',
                '1.0',
                '/move_base_simple/goal',
                'geometry_msgs/msg/PoseStamped',
                goal_msg,
            ],
            output='screen',
        ),
    ])
