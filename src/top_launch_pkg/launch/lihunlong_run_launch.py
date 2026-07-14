from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz for visualization'
    )

    # 1. LiDAR driver (Livox MID360)
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        ])
    )

    # 2. FAST-LIO SLAM
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    # 曾有 uart_to_stm32 / pid_control_pkg / activity_control_pkg 三个节点在此，
    # 2026-07-14 连同源码一并移除（PX4 接管后不再需要；uart 与 XRCE-DDS Agent 抢 /dev/ttyS6）。

    # FAST-LIO -> PX4 EKF2 bridge (uXRCE-DDS, vehicle_visual_odometry)
    lio_px4_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lio_px4_bridge'),
                'launch',
                'bridge.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        rviz_arg,
        livox_launch,
        fast_lio_launch,
        lio_px4_bridge_launch,
    ])
