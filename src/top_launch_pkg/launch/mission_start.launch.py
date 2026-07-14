import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_mission = os.path.join(
        get_package_share_directory('top_launch_pkg'), 'config', 'mission.yaml')
    mission_script = os.path.join(
        get_package_share_directory('top_launch_pkg'), 'launch', 'mission_executor.py')

    mission_file = LaunchConfiguration('mission_file')
    odom_topic = LaunchConfiguration('odom_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    ego_node = LaunchConfiguration('ego_node')
    wait_subscribers = LaunchConfiguration('wait_subscribers')
    vel_tol = LaunchConfiguration('vel_tol')
    settle_sec = LaunchConfiguration('settle_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_file',
            default_value=default_mission,
            description='mission.yaml to publish after the flight stack is already running'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('goal_topic', default_value='/move_base_simple/goal'),
        DeclareLaunchArgument('ego_node', default_value='drone_0_ego_planner_node'),
        DeclareLaunchArgument('wait_subscribers', default_value='2'),
        DeclareLaunchArgument('vel_tol', default_value='0.20'),
        DeclareLaunchArgument('settle_sec', default_value='1.0'),
        ExecuteProcess(
            cmd=[
                'python3', mission_script,
                '--mission', mission_file,
                '--odom_topic', odom_topic,
                '--goal_topic', goal_topic,
                '--ego_node', ego_node,
                '--wait_subscribers', wait_subscribers,
                '--vel_tol', vel_tol,
                '--settle_sec', settle_sec,
            ],
            output='screen',
        ),
    ])
