# 真机 EGO-Planner 集成 launch
#
# 数据流:
#   livox_ros_driver2  ──► /livox/lidar (CustomMsg, 仅给 FAST-LIO)
#                          /livox/imu
#   FAST_LIO            ──► /Odometry            (nav_msgs/Odometry, ENU)
#                           /cloud_registered    (sensor_msgs/PointCloud2, ENU)
#   topic_tools relay   ──► /drone_0_Odometry
#                           /drone_0_cloud_registered
#   ego_planner         ──► /drone_0_planning/bspline   (traj_utils/Bspline)
#   traj_server         ──► /position_cmd               (quadrotor_msgs/PositionCommand, ENU)
#   ego_px4_adapter     ──► /ego/trajectory_setpoint    (px4_msgs/TrajectorySetpoint, NED)
#   px4_offboard_pkg(FLY) 转发 ──► /fmu/in/trajectory_setpoint
#                         EGO 到点后用 /Odometry 做 FINAL_CORRECTION,再 LAND/DISARM
#   lio_px4_bridge       并行 ──► /fmu/in/vehicle_visual_odometry  (PX4 EKF2 视觉里程计)
#
# Goal:RViz 里"2D Goal Pose"工具发到 /move_base_simple/goal,frame=map (ENU)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')
    enable_arm_arg = DeclareLaunchArgument(
        'enable_arm', default_value='false',
        description='True 才会真切 Offboard / arm。第一次跑务必 false 做 dry-run')
    debug_rviz_arg = DeclareLaunchArgument(
        'debug_rviz', default_value='false',
        description='True 启动配好的 ego_debug.rviz')
    final_correction_arg = DeclareLaunchArgument(
        'final_correction', default_value='true',
        description='EGO 到 RViz goal 后,用 MID360 /Odometry 做最终校准,随后自动降落停桨')
    precision_xy_arg = DeclareLaunchArgument(
        'precision_xy_tolerance', default_value='0.12',
        description='MID360 二次校准的 XY 到点容差,米')
    precision_z_arg = DeclareLaunchArgument(
        'precision_z_tolerance', default_value='0.15',
        description='MID360 二次校准的 Z 到点容差,米')

    # 1. 传感+SLAM+EKF2 桥(直接复用现有的 lihunlong_run_launch)
    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('top_launch_pkg'),
                'launch', 'lihunlong_run_launch.py'
            ])
        ]),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()
    )

    # 2a. ego-planner 内部 marker 用 'world' / 'map' 两套 frame_id,FAST-LIO 用 'camera_init'。
    #     加两个 identity static TF 把它们桥到 camera_init,RViz Fixed Frame 选 camera_init 就能看到全部。
    tf_world_to_init = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_world_to_camera_init',
        arguments=['0','0','0','0','0','0','camera_init','world'], output='log',
    )
    tf_map_to_init = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_map_to_camera_init',
        arguments=['0','0','0','0','0','0','camera_init','map'], output='log',
    )

    # 2b. /Odometry → /drone_0_Odometry 转发(适配 ego-planner 的 drone_<id>_<topic> 命名)
    relay_odom = Node(
        package='topic_tools', executable='relay', name='relay_odom',
        arguments=['/Odometry', '/drone_0_Odometry'], output='screen',
    )
    # 3. /cloud_registered → /drone_0_cloud_registered
    relay_cloud = Node(
        package='topic_tools', executable='relay', name='relay_cloud',
        arguments=['/cloud_registered', '/drone_0_cloud_registered'], output='screen',
    )

    # 4. ego_planner advanced_param launch
    ego_advanced = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ego_planner'),
                'launch', 'advanced_param.launch.py'
            ])
        ]),
        launch_arguments={
            'drone_id':         '0',
            'odometry_topic':   'Odometry',           # 实际 sub: /drone_0_Odometry
            'cloud_topic':      'cloud_registered',   # 实际 sub: /drone_0_cloud_registered
            'camera_pose_topic': 'unused_pose',       # 不用深度相机
            'depth_topic':       'unused_depth',
            'map_size_x_': '40.0', 'map_size_y_': '40.0', 'map_size_z_': '4.0',
            # ===== 机体物理参数(330mm X型, 含桨外径 50cm × 40cm × 高 35cm, R≈0.25 m) =====
            # grid_map 用精确圆柱膨胀: XY 按欧氏距离防墙, Z 已由 virtual_ground 硬挡, 单独 inflate 只兜 voxel 量化误差。
            # 0.225 在 0.1m voxel 下不会再被方块膨胀放大成 0.30m/侧, 0.70m 通道仍应连通。
            # 0.10 (=1 voxel @resolution=0.1) 替代 0.175 半机体高: 015940 replay 中 0.175 把虚地板顶推到 0.625m,
            # 离 goal z=0.8 只剩 17.5cm; 0.10 后顶 ≈0.55m 余量回到 25cm, 且 bspline 抖动 median 0.153→0.114。
            # 注: EMERGENCY 没降到 0 是因为原 bag 实际飞到 z=0.46m, 这一段无论怎么调 inflate 都会被 virtual_ground 判危险 —— 闭环实飞不会到这。
            'obstacles_inflation': '0.28',
            'obstacles_inflation_z': '0.20',
            # dist0 是硬膨胀之外的优化软余量, 不再和机体半径重复叠加。
            'dist0':               '0.08',
            # ===== 第一次飞保守动力学(默认值的 ~50%) =====
            'max_vel': '0.6', 'max_acc': '0.5',
            'planning_horizon': '4.0',      # ≈ max_vel × 4s
            'emergency_time':   '1.0',      # > max_vel/max_acc = 0.67s
            # ===== 局部地图: ≥ planning_horizon + 1, ≤ MID-360 有效距离 40m =====
            'local_update_range_x': '8.0',
            'local_update_range_y': '8.0',
            'local_update_range_z': '4.0',
            # ===== 虚拟天花板 =====
            # virtual_ceil 把 z >= ceil 的所有 voxel 全部标 occupied (填实, 不是单层薄膜)。
            # 飞机经优化器与天花板保持 dist0 软距离,
            # 以当前 dist0=0.08 估算, 实际飞行峰值 ≈ virtual_ceil_height - 0.10(ceil voxel 偏移) - 0.08(dist0) - 0.05(余量) ≈ ceil - 0.23 m。
            # 设 1.5 → 实际峰值上限 ≈ 1.27 m (仍高于 takeoff_altitude=1.0)。
            'virtual_ceil_height': '1.5',
            # ===== 方案三 (221710 bag 复盘后加入): 阻止 -Z 下钻, 强制 ±Y 侧绕 =====
            # virtual_ground 把 z<=0.45m 的 inflate 全部填占据, 关掉优化器 {p,v} 在 -Z 的逃逸捷径。
            # enable_height=0.55m: 飞机起飞爬到 0.55m 之上才启用 (避免起飞时把自己包死)。
            # 实际飞行带 ≈ z ∈ [0.55+0.08, 1.0] m (留 dist0 软余量, 上限受 virtual_ceil 影响)。
            'virtual_ground_height':         '0.45',
            'virtual_ground_enable_height':  '0.55',
            # altitude penalty: z_ref 由 planner_manager 用 start_pt.z 自动设, lambda_z 越大越偏好水平飞行。
            # 5.0 是初始建议值 (相对 lambda_smooth=1.0 / lambda_collision=0.5 来定标), 实飞后视 z 跟踪表现调。
            'lambda_z':                      '5.0',
            'use_distinctive_trajs': 'False',     # ros2_version fork 里这个分支(distinctiveTrajs)有 SIGSEGV bug,先关
            'flight_type': '1',                       # 1 = MANUAL_TARGET, 等 RViz Nav Goal
            'point_num': '1',
            'point0_x': '0.0', 'point0_y': '0.0', 'point0_z': '1.0',
            'obj_num_set': '0',
        }.items()
    )

    # 5. traj_server:Bspline → PositionCommand,默认发布 /position_cmd
    traj_server = Node(
        package='ego_planner', executable='traj_server', name='traj_server',
        output='screen',
        remappings=[
            # 让它订到 ego_planner_node 实际 publish 的话题
            ('planning/bspline', '/drone_0_planning/bspline'),
        ],
        parameters=[{'traj_server/time_forward': 1.0}],
    )

    # 6. ENU PositionCommand → NED TrajectorySetpoint
    ego_adapter = Node(
        package='ego_px4_adapter', executable='ego_px4_adapter_node',
        name='ego_px4_adapter', output='screen',
        parameters=[{
            'input_topic': '/position_cmd',
            'output_topic': '/ego/trajectory_setpoint',
            'publish_velocity': True,
            'publish_acceleration': True,
        }],
    )

    # 7. PX4 offboard 控制器,enable_external_traj=True 把 ego 轨迹接进 FLY 阶段
    pkg_share = get_package_share_directory('px4_offboard_pkg')
    offboard_yaml = os.path.join(pkg_share, 'config', 'single_waypoint.yaml')
    px4_offboard = Node(
        package='px4_offboard_pkg', executable='offboard_node',
        name='px4_offboard_waypoint', output='screen',
        parameters=[
            offboard_yaml,
            {
                'enable_arm': LaunchConfiguration('enable_arm'),
                'enable_external_traj': True,
                'external_traj_topic': '/ego/trajectory_setpoint',
                'external_traj_timeout_sec': 0.5,
                'enable_final_correction': LaunchConfiguration('final_correction'),
                'goal_topic': '/move_base_simple/goal',
                'mid360_odom_topic': '/Odometry',
                'ego_goal_reach_xy_tolerance': 0.35,
                'ego_goal_reach_z_tolerance': 0.25,
                'ego_goal_velocity_tolerance': 0.15,
                'ego_goal_settle_seconds': 1.0,
                'precision_xy_tolerance': LaunchConfiguration('precision_xy_tolerance'),
                'precision_z_tolerance': LaunchConfiguration('precision_z_tolerance'),
                'precision_settle_seconds': 1.0,
                'mid360_odom_timeout_sec': 0.5,
                'auto_disarm_after_landing': True,
                'land_disarm_delay_sec': 1.0,
                'disarm_retry_seconds': 1.0,
            },
        ],
        emulate_tty=True,
    )

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('top_launch_pkg'), 'rviz', 'ego_debug.rviz'
    ])
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_ego_debug',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(LaunchConfiguration('debug_rviz')),
    )

    return LaunchDescription([
        rviz_arg, enable_arm_arg, debug_rviz_arg,
        final_correction_arg, precision_xy_arg, precision_z_arg,
        base_stack,
        tf_world_to_init, tf_map_to_init,
        relay_odom, relay_cloud,
        ego_advanced,
        traj_server,
        ego_adapter,
        px4_offboard,
        rviz_node,
    ])
