# EGO-Planner bag-replay launch (no PX4, no FAST-LIO, no traj_server).
#
# 用途: 用 rosbag 录制的 /drone_0_Odometry + /drone_0_cloud_registered 作为输入,
# 让现版本的 ego_planner_node 重新规划, 不闭环执行。Goal 由 run_replay.sh
# 从 bag 提取并手动 publish 到 /move_base_simple/goal.
#
# 必须 use_sim_time=true, 因为 bag play 用 --clock 把 /clock 推到 sim time。
# 参数与 ego_real_flight.launch.py 中 ego_advanced 段保持一致, 任何修改请同步两边。
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 全局 use_sim_time, 让 ego_planner + tf publisher 都吃 /clock.
    use_sim_time = SetParameter(name='use_sim_time', value=True)

    # ego_planner 内部 marker 用 'world' / 'map' frame, FAST-LIO 录的 odom/cloud
    # 用 'camera_init'。两个 identity TF 把它们接上, RViz Fixed Frame 选 camera_init。
    tf_world_to_init = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_world_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'world'],
        output='log',
    )
    tf_map_to_init = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'map'],
        output='log',
    )

    # ego_planner advanced_param launch -- 参数和 ego_real_flight.launch.py 完全一致。
    ego_advanced = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ego_planner'),
                'launch', 'advanced_param.launch.py'
            ])
        ]),
        launch_arguments={
            'drone_id':          '0',
            'odometry_topic':    'Odometry',
            'cloud_topic':       'cloud_registered',
            'camera_pose_topic': 'unused_pose',
            'depth_topic':       'unused_depth',
            'map_size_x_': '40.0', 'map_size_y_': '40.0', 'map_size_z_': '4.0',
            'obstacles_inflation':   '0.28',
            'obstacles_inflation_z': '0.10',
            'dist0':                 '0.08',
            'max_vel': '0.6', 'max_acc': '0.5',
            'planning_horizon': '4.0',
            'emergency_time':   '1.0',
            'local_update_range_x': '8.0',
            'local_update_range_y': '8.0',
            'local_update_range_z': '4.0',
            'virtual_ceil_height':           '1.5',
            'virtual_ground_height':         '0.45',
            'virtual_ground_enable_height':  '0.55',
            'lambda_z':                      '5.0',
            'use_distinctive_trajs': 'False',
            'flight_type': '1',
            'point_num': '1',
            'point0_x': '0.0', 'point0_y': '0.0', 'point0_z': '1.0',
            'obj_num_set': '0',
        }.items()
    )

    return LaunchDescription([
        use_sim_time,
        tf_world_to_init,
        tf_map_to_init,
        ego_advanced,
    ])
