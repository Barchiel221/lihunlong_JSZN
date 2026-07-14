# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 这是什么

ROS 2 Humble colcon 工作区，运行在无人机机载计算机 Orange Pi 5 (aarch64) 上，控制**真实飞行器**：Livox MID360 LiDAR + FAST-LIO2 状态估计 + EGO-Planner-Swarm 轨迹规划 + PX4 飞控。没有仿真器；验证依靠 dry-run（不解锁）和 bag 离线回放。

## 常用命令

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# 构建（colcon.meta 已为 livox_ros_driver2 注入 -DHUMBLE_ROS=humble，勿绕过）
colcon build                                  # 全量
colcon build --packages-select top_launch_pkg # 单包；launch 文件是 CMake install 的，
                                              # 没用 --symlink-install，改 launch/.py 后必须重建该包

# 一键实飞栈（默认 dry-run：enable_arm=false 不会切 Offboard/解锁）
ros2 launch top_launch_pkg ego_real_flight.launch.py
ros2 launch top_launch_pkg ego_real_flight.launch.py enable_arm:=true   # 真飞

# 仅传感+SLAM+EKF2 桥（被 ego_real_flight 包含）
ros2 launch top_launch_pkg lihunlong_run_launch.py

# bag 开环回放（重跑当前版本 planner，不闭环执行）
bag_analysis/replay_*/run_replay.sh   # 环境变量 BAG/RATE/START_OFFSET/DURATION 可覆盖
```

仓库没有测试套件。实飞 bag 在 `/home/orangepi/bags/ego_real_flight_YYYYMMDD_HHMMSS/`，离线分析脚本与历次复盘结论在 `bag_analysis/`（rosbag2_py 直接读 sqlite3，见 `01_timeline.py` 等）。

## 启动硬件前提

- Livox SDK 的 `.so` 须在 `/usr/local/lib`，否则 dlopen error
- 网口 `enP3p49s0` 配 `192.168.1.5/24`（MID360 固定 `192.168.1.168`），否则 bind failed
- PX4 经 Micro-XRCE-DDS-Agent 走 `/dev/ttyS6`（曾用 `uart_to_stm32` 走同一串口，与 Agent 冲突，已移除）
  - Agent **不在 colcon 工作区**，源码在 `~/Micro-XRCE-DDS-Agent`，二进制已装到 `/usr/local/bin/MicroXRCEAgent`
  - 由 **systemd 常驻服务 `micro-xrce-agent.service`（enabled，开机自启、崩溃 5s 重启）** 拉起，无需手动启动；命令 `MicroXRCEAgent serial --dev /dev/ttyS6 -b 921600`
  - 波特率 921600 写死在 service 文件里，须与 PX4 端 `uxrce_dds_client` 一致；排查用 `systemctl status micro-xrce-agent` / `journalctl -u micro-xrce-agent -f`，抽风重启用 `sudo systemctl restart micro-xrce-agent`

## 架构：数据流（单向流水线）

```
livox_ros_driver2 ──► /livox/lidar (CustomMsg) + /livox/imu (200Hz)
FAST_LIO          ──► /Odometry + /cloud_registered     (frame=camera_init, ENU)
topic_tools relay ──► /drone_0_Odometry, /drone_0_cloud_registered  (适配 ego 的 drone_<id>_ 命名)
ego_planner       ──► /drone_0_planning/bspline
traj_server       ──► /position_cmd                     (PositionCommand, ENU)
ego_px4_adapter   ──► /ego/trajectory_setpoint          (TrajectorySetpoint, ENU→NED 转换)
px4_offboard_pkg  ──► /fmu/in/trajectory_setpoint       (状态机: 起飞→FLY 转发 ego→到点
                                                         FINAL_CORRECTION→LAND→DISARM)
lio_px4_bridge    ──► /fmu/in/vehicle_visual_odometry   (并行喂 PX4 EKF2)
```

Goal 来自 RViz "2D Goal Pose" → `/move_base_simple/goal`（frame=map, ENU）。

### 坐标系

FAST-LIO 的世界系是 `camera_init`；ego-planner 内部混用 `world`/`map`。launch 里用两个 identity static TF 把三者钉成同一系，RViz Fixed Frame 选 `camera_init`。ENU↔NED 转换只发生在 `ego_px4_adapter` 和 `lio_px4_bridge` 两处。

### 包的归属

- **自研集成代码**（改动主要发生在这里）：`lio_px4_bridge`、`ego_px4_adapter`、`px4_offboard_pkg`、`top_launch_pkg`（各自只有一个主源文件/launch 目录）
- **上游 fork**：`FAST_LIO`、`ego_swarm`（ROS2 fork，planner 在 `src/ego_swarm/src/planner/` 下：`plan_env` 栅格地图 / `path_searching` A* 前端 / `bspline_opt` 后端优化 / `plan_manage` 调度 + traj_server）、`livox_ros_driver2`、`px4_msgs`
- **已移除**（2026-07-14 清理，曾在 launch 中注释停用，源码见 `.backups/cleanup_unused_pkgs_20260714_104852.tar.gz`）：`uart_to_stm32`、`pid_control_pkg`、`activity_control_pkg`、`serial_comm`

## 关键约束与已知坑

- **planner 参数集中在 `top_launch_pkg/launch/ego_planner_params.py`（单一真源）**，`ego_real_flight` / `ego_replay` / `ego_sim_flight` 三个 launch 都 import 它，改一处三处生效（改动 H 后取代了原"人工双向同步"约定）。profile 相关的 max_vel/planning_horizon/emergency_time 由各 launch 按档位构造后传入。
- `use_distinctive_trajs` 必须为 `False`：此 ROS2 fork 的 distinctiveTrajs 分支有 SIGSEGV bug。
- 这个 EGO-Planner v1 **没有 ESDF**、不做多候选打分，只优化一条 B 样条；obstacle cost 是 inflated occupancy + `{p,v}` 径向搜索（`bspline_opt/src/bspline_optimizer.cpp`）。分析飞行行为时以此为准，勿按 ESDF 版叙事。
- 调参（inflation / virtual_ground / virtual_ceil / lambda_z 等）的取值依据都写在 `ego_planner_params.py` 的行内注释里，且多数来自具体 bag 复盘；改参数前先读注释和对应 `bag_analysis/FINDINGS_*.md`。当前为 220mm 机型参数（inflation 0.20、map 20×20×3、local_update_range 6.0 等），330mm 旧值见注释。
- 回放模式下出现 EMERGENCY 是历史轨迹与新参数 mismatch 的预期现象，不预示闭环风险。
- 分析飞行问题必须以 bag 数据为证据，不要从代码"一般行为"外推；不同 bag 的证据不要混搭。

`技术报告_附录_关键源码.md`（仓库根目录）按数据流方向摘录了各环节关键源码，是快速理解全链路的最佳入口。
