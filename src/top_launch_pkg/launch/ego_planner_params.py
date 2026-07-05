# EGO-Planner advanced_param 的共享参数段 (改动 H)。
#
# 单一真源: ego_real_flight / ego_replay / ego_sim_flight 三个 launch 都 import 这里,
# 替代原"人工双向同步"约定 (CLAUDE.md 里 ego_replay↔ego_real_flight 必须同步的已知坑)。
# 改 planner 参数只改这一个文件, 三处生效。
#
# 动态量 (max_vel / planning_horizon / emergency_time) 由调用方传入:
#   - real_flight / sim: 传 profile 相关的 PythonExpression substitution;
#   - replay: 传固定字符串。
# 其余量 (体积/地图/虚拟边界/lambda 等) 在此集中, 取值依据见行内注释与计划书 §4.1。
#
# 调用方式:
#   from ego_planner_params import ego_advanced_launch_arguments
#   IncludeLaunchDescription(..., launch_arguments=ego_advanced_launch_arguments(
#       max_vel=..., planning_horizon=..., emergency_time=...).items())


def ego_advanced_launch_arguments(max_vel='0.6', max_acc='0.5',
                                  planning_horizon='4.0', emergency_time='1.0',
                                  overrides=None):
    """返回传给 ego_planner/advanced_param.launch.py 的参数 dict。

    max_vel / planning_horizon / emergency_time 可为字符串或 launch substitution。
    overrides: 可选 dict, 覆盖/追加个别 key (如 sim 变体调 topic)。
    """
    args = {
        'drone_id':          '0',
        'odometry_topic':    'Odometry',           # 实际 sub: /drone_0_Odometry
        'cloud_topic':       'cloud_registered',   # 实际 sub: /drone_0_cloud_registered
        'camera_pose_topic': 'unused_pose',        # 不用深度相机
        'depth_topic':       'unused_depth',
        # 国赛 8×8×2m 场地 + 余量, 收小地图配合"边界硬约束"(计划书风险表)。原 40/40/4 是省赛遗留。
        'map_size_x_': '20.0', 'map_size_y_': '20.0', 'map_size_z_': '3.0',
        # ===== 机体物理参数(220mm X型, 含桨最宽约 30cm, R≈0.15 m; 计划书 §4.1) =====
        # grid_map 用精确圆柱膨胀: XY 按欧氏距离防墙, Z 已由 virtual_ground 硬挡, 单独 inflate 只兜 voxel 量化误差。
        # 0.20 = R0.15 + voxel 量化 0.05(resolution=0.1); 0.7m 拱门开口 0.7−2×0.20=0.30m 自由走廊, 机心可用 ±0.07m(dist0), 可行。
        # 0.15 (机高更矮) 替代旧 0.20; 历史(330mm)依据: 015940 replay 里 0.175 把虚地板顶推到 0.625m、离 goal 只剩 17.5cm。
        # 旧值 330mm: inflation 0.28 / inflation_z 0.20 (0.7m 拱门走廊只剩 0.14m 基本不可行 —— 改 220mm 的核心收益)。
        'obstacles_inflation':   '0.20',
        'obstacles_inflation_z': '0.15',
        # dist0 是硬膨胀之外的优化软余量, 与机体半径无关, 继续兜优化抖动, 不随改机变。
        'dist0':                 '0.08',
        # ===== 动力学: 由调用方按 profile 传入。standard/conservative=0.6, fast=0.8 (E3) =====
        'max_vel': max_vel, 'max_acc': max_acc,
        'planning_horizon': planning_horizon,   # standard 4.0 ≈ max_vel×4s; fast 5.0
        'emergency_time':   emergency_time,      # 必须 > max_vel/max_acc; standard 1.0 / fast 1.6
        # ===== 局部地图: ≥ planning_horizon + 1; 6.0 为区域③移动障碍残影对策(计划书 §4.4), 仍 ≥ 4.0+1 =====
        'local_update_range_x': '6.0',
        'local_update_range_y': '6.0',
        'local_update_range_z': '4.0',
        # ===== 虚拟天花板 =====
        # virtual_ceil 把 z >= ceil 的所有 voxel 全部标 occupied (填实)。实际峰值 ≈ ceil − 0.23 m。
        # 设 1.5 → 峰值上限 ≈ 1.27 m (仍高于 takeoff_altitude=1.0)。
        'virtual_ceil_height': '1.5',
        # ===== 方案三 (221710 bag 复盘后加入): 阻止 -Z 下钻, 强制 ±Y 侧绕 =====
        # virtual_ground 把 z<=0.45m 的 inflate 全部填占据; enable=0.55m 起飞爬过才启用(避免包死自己)。
        'virtual_ground_height':         '0.45',
        'virtual_ground_enable_height':  '0.55',
        # altitude penalty: lambda_z 越大越偏好水平飞行。5.0 初始建议值, 实飞后视 z 跟踪调。
        'lambda_z':                      '5.0',
        'use_distinctive_trajs': 'False',     # ros2 fork 里 distinctiveTrajs 分支有 SIGSEGV bug, 关
        'flight_type': '1',                    # 1 = MANUAL_TARGET, 等 RViz Nav Goal / mission_executor
        'point_num': '1',
        'point0_x': '0.0', 'point0_y': '0.0', 'point0_z': '1.0',
        'obj_num_set': '0',
    }
    if overrides:
        args.update(overrides)
    return args
