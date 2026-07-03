# Unity route-B milestone 2: Unity point cloud + PX4 IMU -> FAST-LIO -> PX4 vision.
#
# This is intentionally separate from ego_sim_flight.launch.py because FAST-LIO
# needs calibrated time/frame/extrinsic settings before it is trustworthy.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _ego_advanced():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ego_planner'),
                'launch', 'advanced_param.launch.py'
            ])
        ]),
        launch_arguments={
            'drone_id': '0',
            'odometry_topic': 'Odometry',
            'cloud_topic': 'cloud_registered',
            'camera_pose_topic': 'unused_pose',
            'depth_topic': 'unused_depth',
            'map_size_x_': '40.0', 'map_size_y_': '40.0', 'map_size_z_': '4.0',
            'obstacles_inflation': '0.28',
            'obstacles_inflation_z': '0.20',
            'dist0': '0.08',
            'max_vel': '0.6', 'max_acc': '0.5',
            'planning_horizon': '4.0',
            'emergency_time': '1.0',
            'local_update_range_x': '8.0',
            'local_update_range_y': '8.0',
            'local_update_range_z': '4.0',
            'virtual_ceil_height': '1.5',
            'virtual_ground_height': '0.45',
            'virtual_ground_enable_height': '0.55',
            'lambda_z': '5.0',
            'use_distinctive_trajs': 'False',
            'flight_type': '1',
            'point_num': '1',
            'point0_x': '0.0', 'point0_y': '0.0', 'point0_z': '1.0',
            'obj_num_set': '0',
        }.items()
    )


def generate_launch_description():
    enable_arm_arg = DeclareLaunchArgument('enable_arm', default_value='false')
    debug_rviz_arg = DeclareLaunchArgument('debug_rviz', default_value='false')
    fast_lio_rviz_arg = DeclareLaunchArgument('fast_lio_rviz', default_value='false')
    final_correction_arg = DeclareLaunchArgument('final_correction', default_value='true')
    precision_xy_arg = DeclareLaunchArgument('precision_xy_tolerance', default_value='0.12')
    precision_z_arg = DeclareLaunchArgument('precision_z_tolerance', default_value='0.15')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    gazebo_model_arg = DeclareLaunchArgument('gazebo_model', default_value='iris')
    pose_input_type_arg = DeclareLaunchArgument(
        'pose_input_type',
        default_value='gazebo_model_states',
        description='gazebo_model_states, nav_msgs_odometry, or px4_vehicle_odometry')
    pose_input_topic_arg = DeclareLaunchArgument(
        'pose_input_topic', default_value='/gazebo/model_states')
    pose_transform_arg = DeclareLaunchArgument(
        'pose_transform_mode', default_value='passthrough')
    px4_imu_topic_arg = DeclareLaunchArgument(
        'px4_sensor_combined_topic', default_value='/fmu/out/sensor_combined')

    pose_relay = Node(
        package='top_launch_pkg',
        executable='pose_relay.py',
        name='pose_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model_name': LaunchConfiguration('gazebo_model'),
            'input_type': LaunchConfiguration('pose_input_type'),
            'input_topic': LaunchConfiguration('pose_input_topic'),
            'publish_lio_odom': False,
            'transform_mode': LaunchConfiguration('pose_transform_mode'),
            'origin': [0.0, 0.0, 0.0],
            'frame_id': 'camera_init',
            'child_frame_id': 'body',
        }],
    )

    imu_converter = Node(
        package='top_launch_pkg',
        executable='px4_sensor_combined_to_imu.py',
        name='px4_sensor_combined_to_imu',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('px4_sensor_combined_topic'),
            'output_topic': '/sim/livox/imu',
            'frame_id': 'base_link',
            'acceleration_scale': 1.0,
        }],
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
            'config_file': 'unity_sim.yaml',
            'rviz': LaunchConfiguration('fast_lio_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    lio_px4_bridge = Node(
        package='lio_px4_bridge',
        executable='bridge_node',
        name='lio_px4_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lio_odom_topic': '/Odometry',
            'imu_topic': '/sim/livox/imu',
            'effect_cloud_topic': '/cloud_effected',
            'px4_vo_topic': '/fmu/in/vehicle_visual_odometry',
            'degraded_threshold': 100,
            'degraded_cov_scale': 100.0,
            'position_var': 0.30,
            'orientation_var': 0.05,
            'velocity_var': 1.00,
            'imu_max_dt': 0.05,
            'publish_on_imu': False,
            'imu_acc_in_g': False,
        }],
    )

    tf_world_to_init = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_world_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'world'], output='log',
    )
    tf_map_to_init = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'map'], output='log',
    )
    relay_odom = Node(
        package='topic_tools', executable='relay', name='relay_odom',
        arguments=['/Odometry', '/drone_0_Odometry'], output='screen',
    )
    relay_cloud = Node(
        package='topic_tools', executable='relay', name='relay_cloud',
        arguments=['/cloud_registered', '/drone_0_cloud_registered'], output='screen',
    )

    traj_server = Node(
        package='ego_planner', executable='traj_server', name='traj_server',
        output='screen',
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
        enable_arm_arg, debug_rviz_arg, fast_lio_rviz_arg, final_correction_arg,
        precision_xy_arg, precision_z_arg, use_sim_time_arg, gazebo_model_arg,
        pose_input_type_arg, pose_input_topic_arg, pose_transform_arg, px4_imu_topic_arg,
        pose_relay, imu_converter, fast_lio_launch, lio_px4_bridge,
        tf_world_to_init, tf_map_to_init, relay_odom, relay_cloud,
        _ego_advanced(), traj_server, ego_adapter, px4_offboard, rviz_node,
    ])
