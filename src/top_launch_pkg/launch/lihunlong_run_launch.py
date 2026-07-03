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

    # 3. UART <-> STM32 communication node
   # uart_node = Node(
   #    package='uart_to_stm32',
   #     executable='uart_to_stm32_node',
   #    name='uart_to_stm32_node',
   #     output='screen',
   #)

    # 4. PID controller
    pid_node = Node(
        package='pid_control_pkg',
        executable='pid_control_node',
        name='pid_control_node',
        output='screen',
        parameters=[{
            'source_frame': 'map',
            'target_frame': 'laser_link',
        }]
    )

    # 5. Route / activity control
    activity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('activity_control_pkg'),
                'launch',
                'route_target_publisher_launch.py'
            ])
        ])
    )

    # 6. FAST-LIO -> PX4 EKF2 bridge (uXRCE-DDS, vehicle_visual_odometry)
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
        # uart_node,           # 暂时关掉:与 Micro-XRCE-DDS-Agent 在 /dev/ttyS6 上冲突
        # pid_node,            # 暂时关掉:对接 PX4 后位置外环交给 PX4 MPC,不再叠这层 PID
        #activity_launch,
        lio_px4_bridge_launch,
    ])
