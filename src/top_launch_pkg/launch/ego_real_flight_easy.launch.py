import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_arm = LaunchConfiguration('enable_arm')
    debug_rviz = LaunchConfiguration('debug_rviz')
    rviz = LaunchConfiguration('rviz')
    record = LaunchConfiguration('record')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_prefix = LaunchConfiguration('bag_prefix')
    auto_goal = LaunchConfiguration('auto_goal')
    goal_delay = LaunchConfiguration('goal_delay')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_z = LaunchConfiguration('goal_z')
    wait_subscribers = LaunchConfiguration('wait_subscribers')
    final_correction = LaunchConfiguration('final_correction')
    precision_xy_tolerance = LaunchConfiguration('precision_xy_tolerance')
    precision_z_tolerance = LaunchConfiguration('precision_z_tolerance')
    multi_goal = LaunchConfiguration('multi_goal')
    multi_waypoints = LaunchConfiguration('multi_waypoints')
    multi_xy_tol = LaunchConfiguration('multi_xy_tol')
    multi_z_tol = LaunchConfiguration('multi_z_tol')
    multi_vel_tol = LaunchConfiguration('multi_vel_tol')
    multi_settle_sec = LaunchConfiguration('multi_settle_sec')

    flight_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('top_launch_pkg'),
                'launch',
                'ego_real_flight.launch.py',
            ])
        ]),
        launch_arguments={
            'enable_arm': enable_arm,
            'debug_rviz': debug_rviz,
            'rviz': rviz,
            'final_correction': final_correction,
            'precision_xy_tolerance': precision_xy_tolerance,
            'precision_z_tolerance': precision_z_tolerance,
        }.items(),
    )

    # 用 bash 变量把时间戳和路径锁住, printf 提示和 ros2 bag record 共用同一个路径,
    # 这样终端打出的"RECORDING -> ..."一定就是真实落盘的目录,避免误以为没录。
    record_cmd = [
        'TS=$(date +%Y%m%d_%H%M%S); ',
        'BAG_DIR="', bag_dir, '"; ',
        'BAG_PATH="${BAG_DIR}/', bag_prefix, '_${TS}"; ',
        'mkdir -p "${BAG_DIR}"; ',
        r'printf "\n\033[1;42;30m================================================================\033[0m\n"; ',
        r'printf "\033[1;42;30m  RECORDING ROSBAG ->\033[0m \033[1;33m%s\033[0m\n" "$BAG_PATH"; ',
        r'printf "\033[1;42;30m  (pass record:=false to disable)                              \033[0m\n"; ',
        r'printf "\033[1;42;30m================================================================\033[0m\n\n"; ',
        'ros2 bag record -a --include-hidden-topics -o "${BAG_PATH}"',
    ]
    recorder = ExecuteProcess(
        cmd=['bash', '-lc', record_cmd],
        output='screen',
        condition=IfCondition(record),
    )

    goal_msg = [
        '{header: {frame_id: map}, pose: {position: {x: ',
        goal_x,
        ', y: ',
        goal_y,
        ', z: ',
        goal_z,
        '}, orientation: {w: 1.0}}}',
    ]
    goal_cmd = [
        'sleep ',
        goal_delay,
        '; ros2 topic pub -1 -w ',
        wait_subscribers,
        ' --keep-alive 1.0 /move_base_simple/goal '
        'geometry_msgs/msg/PoseStamped "',
        *goal_msg,
        '"',
    ]
    goal_once = ExecuteProcess(
        cmd=['bash', '-lc', goal_cmd],
        output='screen',
        condition=IfCondition(auto_goal),
    )

    # 多航点 sequencer: 顺序发 waypoints 到 /move_base_simple/goal,
    # 中间点过路不停, 最后一点交给 px4_offboard 做 FINAL_CORRECTION + LAND。
    multi_script = PathJoinSubstitution([
        FindPackageShare('top_launch_pkg'), 'launch', 'multi_waypoint_publisher.py'
    ])
    multi_cmd = [
        'sleep ', goal_delay,
        '; python3 ', multi_script,
        ' --waypoints "', multi_waypoints, '"',
        ' --wait_subscribers ', wait_subscribers,
        ' --xy_tol ', multi_xy_tol,
        ' --z_tol ', multi_z_tol,
        ' --vel_tol ', multi_vel_tol,
        ' --settle_sec ', multi_settle_sec,
    ]
    multi_goal_proc = ExecuteProcess(
        cmd=['bash', '-lc', multi_cmd],
        output='screen',
        condition=IfCondition(multi_goal),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_arm',
            default_value='false',
            description='Forwarded to ego_real_flight.launch.py. Use true only for real flight.',
        ),
        DeclareLaunchArgument(
            'debug_rviz',
            default_value='false',
            description='Open the preconfigured EGO RViz view.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Forwarded to the FAST-LIO base stack.',
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='Record all ROS topics into a full rosbag.',
        ),
        DeclareLaunchArgument(
            'bag_dir',
            default_value=os.path.expanduser('~/bags'),
            description='Directory where rosbag files are stored.',
        ),
        DeclareLaunchArgument(
            'bag_prefix',
            default_value='ego_real_flight',
            description='Rosbag name prefix before the timestamp.',
        ),
        DeclareLaunchArgument(
            'auto_goal',
            default_value='false',
            description='Publish one RViz goal automatically after goal_delay seconds.',
        ),
        DeclareLaunchArgument(
            'goal_delay',
            default_value='0.0',
            description='Seconds to wait before publishing auto_goal.',
        ),
        DeclareLaunchArgument('goal_x', default_value='2.5'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_z', default_value='0.8'),
        DeclareLaunchArgument(
            'wait_subscribers',
            default_value='2',
            description='Wait for this many /move_base_simple/goal subscribers before auto_goal publishes.',
        ),
        DeclareLaunchArgument(
            'final_correction',
            default_value='true',
            description='EGO 到 RViz goal 后,用 MID360 /Odometry 做最终校准,随后自动降落停桨',
        ),
        DeclareLaunchArgument('precision_xy_tolerance', default_value='0.12'),
        DeclareLaunchArgument('precision_z_tolerance', default_value='0.15'),
        DeclareLaunchArgument(
            'multi_goal',
            default_value='false',
            description='True 时按 multi_waypoints 顺序串行发航点 (与 auto_goal 互斥)。',
        ),
        DeclareLaunchArgument(
            'multi_waypoints',
            default_value='2,0.5,1;2,2,1;4,4,1',
            description='";" 分隔的 "x,y,z" 列表 (ENU map). 仅 multi_goal=true 时生效。',
        ),
        DeclareLaunchArgument(
            'multi_xy_tol',
            default_value='0.35',
            description='中间航点 XY 到点容差 (m)',
        ),
        DeclareLaunchArgument(
            'multi_z_tol',
            default_value='0.25',
            description='中间航点 Z 到点容差 (m)',
        ),
        DeclareLaunchArgument(
            'multi_vel_tol',
            default_value='0.20',
            description='中间航点速度阈值 (m/s)',
        ),
        DeclareLaunchArgument(
            'multi_settle_sec',
            default_value='1.0',
            description='进入容差后稳定多久才切下一航点 (s)',
        ),
        flight_stack,
        recorder,
        goal_once,
        multi_goal_proc,
    ])
