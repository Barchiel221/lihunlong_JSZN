# 多航点发布器: 顺序发送多个 /move_base_simple/goal 给 EGO + px4_offboard。
#
# 用法 (默认 3 个点):
#   ros2 launch top_launch_pkg ego_multi_goal.launch.py
#
# 自定义航点 (";" 分隔, 每点 "x,y,z" 在 ENU map 系):
#   ros2 launch top_launch_pkg ego_multi_goal.launch.py \
#       waypoints:='2,0.5,1;2,2,1;4,4,1'
#
# 中间航点: XY <= 0.35 / Z <= 0.25 / |v| <= 0.20 持续 1s 即切下一个,
#           过路不悬停, 全程仅 EGO 局部规划在跑。
# 最后航点: 发完即交给 px4_offboard 处理 (FINAL_CORRECTION -> MID360 精对 -> LAND -> DISARM)。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    script_path = os.path.join(
        get_package_share_directory('top_launch_pkg'),
        'launch', 'multi_waypoint_publisher.py')

    waypoints = LaunchConfiguration('waypoints')
    wait_subscribers = LaunchConfiguration('wait_subscribers')
    xy_tol = LaunchConfiguration('xy_tol')
    z_tol = LaunchConfiguration('z_tol')
    vel_tol = LaunchConfiguration('vel_tol')
    settle_sec = LaunchConfiguration('settle_sec')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints',
            default_value='2,0.5,1;2,2,1;4,4,1',
            description='";" 分隔的 "x,y,z" 列表 (ENU, frame=map)'),
        DeclareLaunchArgument(
            'wait_subscribers', default_value='2',
            description='等待 /move_base_simple/goal 上至少这么多订阅者再发首点'),
        DeclareLaunchArgument(
            'xy_tol', default_value='0.35',
            description='中间点 XY 到点容差 (m), 与 px4_offboard 的 ego_goal_reach_xy_tolerance 同'),
        DeclareLaunchArgument(
            'z_tol', default_value='0.25',
            description='中间点 Z 到点容差 (m)'),
        DeclareLaunchArgument(
            'vel_tol', default_value='0.20',
            description='中间点速度阈值 (m/s), 略松于 px4_offboard 内部 0.15 (odom 速度噪声更大)'),
        DeclareLaunchArgument(
            'settle_sec', default_value='1.0',
            description='进入容差后需稳定的秒数, 再切下一航点'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/Odometry',
            description='用于到点判定的里程计 topic (FAST-LIO 输出, frame=camera_init ≈ map)'),

        ExecuteProcess(
            cmd=[
                'python3', script_path,
                '--waypoints', waypoints,
                '--wait_subscribers', wait_subscribers,
                '--xy_tol', xy_tol,
                '--z_tol', z_tol,
                '--vel_tol', vel_tol,
                '--settle_sec', settle_sec,
                '--odom_topic', odom_topic,
            ],
            output='screen',
        ),
    ])
