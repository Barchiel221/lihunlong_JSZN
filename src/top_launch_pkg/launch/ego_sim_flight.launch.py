# 仿真变体 launch (改动 H, 路线B 里程碑1: 真值定位)。
#
# 与 ego_real_flight.launch.py 的差异(指南第 7 节):
#   - 去掉传感层(lihunlong_run: livox + FAST_LIO + lio_px4_bridge)——里程碑1 不上 FAST-LIO;
#   - relay_cloud 改订 Unity 点云 /drone_0_pcl_render_node/cloud -> /drone_0_cloud_registered;
#   - 额外起 pose_relay(PUBLISH_LIO=True), 真值当 /Odometry;
#   - ego_advanced / traj_server / ego_px4_adapter / px4_offboard —— 与实飞完全一致
#     (planner 参数来自共享 ego_planner_params.py, 不会与实飞漂移);
#   - mission_file / profile / enable_arm 与实飞同名, 仿真里跑的就是比赛当天的代码。
#
# ★ 点云 frame_id 须为 camera_init: topic_tools relay 不改 frame, 若 Unity 点云 frame 不对,
#   在 relay 前加改 frame 的小节点, 或按指南第 7 节对齐 grid_map。坐标符号见 pose_relay.py 警告。
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ament_index_python.packages import get_package_share_directory
from ego_planner_params import ego_advanced_launch_arguments
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_arm_arg = DeclareLaunchArgument(
        'enable_arm', default_value='false',
        description='True 才真切 Offboard/arm。先 false dry-run')
    final_correction_arg = DeclareLaunchArgument('final_correction', default_value='true')
    precision_xy_arg = DeclareLaunchArgument('precision_xy_tolerance', default_value='0.10')
    precision_z_arg = DeclareLaunchArgument('precision_z_tolerance', default_value='0.15')
    mission_file_arg = DeclareLaunchArgument(
        'mission_file', default_value='',
        description='mission.yaml 路径。空=RViz 手动;非空=mission_executor')
    profile_arg = DeclareLaunchArgument('profile', default_value='standard',
                                        description='conservative | standard | fast')
    cloud_topic_arg = DeclareLaunchArgument(
        'sim_cloud_topic', default_value='/drone_0_pcl_render_node/cloud',
        description='Unity 渲染点云话题(里程碑2 改回 FAST-LIO 的 /cloud_registered)')
    drone_model_arg = DeclareLaunchArgument(
        'drone_model', default_value='iris', description='SITL gazebo 机型名')

    profile = LaunchConfiguration('profile')
    is_fast = ["'", profile, "' == 'fast'"]
    max_vel_sub = PythonExpression(["'0.8' if "] + is_fast + [" else '0.6'"])
    planning_horizon_sub = PythonExpression(["'5.0' if "] + is_fast + [" else '4.0'"])
    emergency_time_sub = PythonExpression(["'1.6' if "] + is_fast + [" else '1.0'"])

    # identity TF: world/map -> camera_init (与实飞一致)
    tf_world_to_init = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_world_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'world'], output='log',
    )
    tf_map_to_init = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'map'], output='log',
    )

    # 真值位姿 -> /Odometry (里程碑1); 与实飞的 relay_odom 一致把 /Odometry -> /drone_0_Odometry
    pose_relay_script = os.path.join(
        get_package_share_directory('top_launch_pkg'), 'launch', 'pose_relay.py')
    pose_relay = ExecuteProcess(
        cmd=['python3', pose_relay_script, '--drone_model', LaunchConfiguration('drone_model')],
        output='screen',
    )
    relay_odom = Node(
        package='topic_tools', executable='relay', name='relay_odom',
        arguments=['/Odometry', '/drone_0_Odometry'], output='screen',
    )
    # Unity 点云 -> /drone_0_cloud_registered (EGO grid_map 直接吃)
    relay_cloud = Node(
        package='topic_tools', executable='relay', name='relay_cloud',
        arguments=[LaunchConfiguration('sim_cloud_topic'), '/drone_0_cloud_registered'],
        output='screen',
    )

    # ego_advanced: 与实飞/回放同源共享参数(改动 H)
    ego_advanced = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ego_planner'), 'launch', 'advanced_param.launch.py'
            ])
        ]),
        launch_arguments=ego_advanced_launch_arguments(
            max_vel=max_vel_sub,
            planning_horizon=planning_horizon_sub,
            emergency_time=emergency_time_sub,
        ).items()
    )

    traj_server = Node(
        package='ego_planner', executable='traj_server', name='traj_server', output='screen',
        remappings=[('planning/bspline', '/drone_0_planning/bspline')],
        parameters=[{'traj_server/time_forward': 1.0}],
    )
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
                # 改动 I: yaw 跟随(仿真里可验起飞无 snap / 转角单调, 与实飞同码)
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

    # mission_executor(与实飞同一脚本, mission_file 非空时启动)
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

    return LaunchDescription([
        enable_arm_arg, final_correction_arg, precision_xy_arg, precision_z_arg,
        mission_file_arg, profile_arg, cloud_topic_arg, drone_model_arg,
        tf_world_to_init, tf_map_to_init,
        pose_relay, relay_odom, relay_cloud,
        ego_advanced,
        traj_server,
        ego_adapter,
        px4_offboard,
        mission_executor,
    ])
