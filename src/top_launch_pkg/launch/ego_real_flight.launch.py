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
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ament_index_python.packages import get_package_share_directory
from ego_planner_params import ego_advanced_launch_arguments  # 改动 H: 共享 planner 参数段
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
        'precision_xy_tolerance', default_value='0.10',   # 220mm: 电机投影半径 0.11, KT 板 0.5 → 留余量保 4 电机进板(计划书 §4.1)
        description='MID360 二次校准的 XY 到点容差,米')
    precision_z_arg = DeclareLaunchArgument(
        'precision_z_tolerance', default_value='0.15',
        description='MID360 二次校准的 Z 到点容差,米')
    # E2: mission_file 非空 -> 拉起 mission_executor 自动发 goal(替代 RViz 手点);
    #     默认空 = 维持 RViz 手动模式, 日常调试不受影响。
    mission_file_arg = DeclareLaunchArgument(
        'mission_file', default_value='',
        description='mission.yaml 路径。空=RViz 手动发 goal;非空=mission_executor 自动任务调度')
    # E3: 参数档。conservative=standard=现值(保守), fast 提速(仅第一轮≥90 分时用, 现场执行预案 §5)。
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='standard',
        description='conservative | standard | fast')
    profile = LaunchConfiguration('profile')
    # fast 档: max_vel 0.8 / planning_horizon 5.0 / emergency_time 1.6(必须 > max_vel/max_acc)。
    is_fast = ["'", profile, "' == 'fast'"]
    max_vel_sub = PythonExpression(["'0.8' if "] + is_fast + [" else '0.6'"])
    planning_horizon_sub = PythonExpression(["'5.0' if "] + is_fast + [" else '4.0'"])
    emergency_time_sub = PythonExpression(["'1.6' if "] + is_fast + [" else '1.0'"])

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
        # 改动 H: planner 参数来自共享 ego_planner_params.py(单一真源)。profile 相关的三个
        # 动态量在本文件按档位构造后传入, 其余参数集中在共享模块, 改一处三 launch 生效。
        launch_arguments=ego_advanced_launch_arguments(
            max_vel=max_vel_sub,
            planning_horizon=planning_horizon_sub,
            emergency_time=emergency_time_sub,
        ).items()
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
                # 改动 I: FLY 段机头跟随飞行方向(速度死区+限速), 消除区域②→③右转后倒飞。
                # yaw_follow_speed 设极大值即退化为锁死 home_yaw_(省赛行为)。
                'yaw_follow_speed': 0.3,
                'yaw_rate_max_deg': 60.0,
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

    # E2: mission_executor(mission_file 非空时启动)。ExecuteProcess 跑 share/launch 下的脚本,
    # 与 ego_multi_goal.launch.py 同一模式。IfCondition 判 mission_file != ''。
    mission_script = os.path.join(
        get_package_share_directory('top_launch_pkg'), 'launch', 'mission_executor.py')
    mission_executor = ExecuteProcess(
        cmd=[
            'python3', mission_script,
            '--mission', LaunchConfiguration('mission_file'),
            '--odom_topic', '/Odometry',
            '--goal_topic', '/move_base_simple/goal',
            '--ego_node', 'drone_0_ego_planner_node',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mission_file'), "' != ''"])),
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
        mission_file_arg, profile_arg,
        base_stack,
        tf_world_to_init, tf_map_to_init,
        relay_odom, relay_cloud,
        ego_advanced,
        traj_server,
        ego_adapter,
        px4_offboard,
        mission_executor,
        rviz_node,
    ])
