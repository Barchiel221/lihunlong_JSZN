# RC altitude-hold flight support launch.
#
# Starts only the MID360 sensing stack and the FAST-LIO -> PX4 EKF2 bridge:
#   livox_ros_driver2  -> /livox/lidar, /livox/imu
#   fast_lio           -> /Odometry, /cloud_effected
#   lio_px4_bridge     -> /fmu/in/vehicle_visual_odometry
#
# It deliberately does not start px4_offboard_pkg, ego_planner, traj_server,
# ego_px4_adapter, or any trajectory/offboard setpoint publisher.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch FAST-LIO RViz visualization',
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py',
            ])
        ])
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py',
            ])
        ]),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    lio_px4_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lio_px4_bridge'),
                'launch',
                'bridge.launch.py',
            ])
        ])
    )

    return LaunchDescription([
        rviz_arg,
        livox_launch,
        fast_lio_launch,
        lio_px4_bridge_launch,
    ])
