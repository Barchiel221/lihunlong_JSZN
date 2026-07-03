# 空中具身智能挑战赛 · Unity 仿真器 × 你的 ROS2/PX4 全栈接入指南（路线 B，已按实际代码修订）

> 目标：在 Ubuntu 22.04 + ROS2 Humble 上，用官方 Unity 仿真器当**场景 + 雷达点云渲染器**，
> 把你真实的整套 stack（`ego_swarm`(EGO-Planner v1) → `ego_px4_adapter` → `px4_offboard_pkg` →
> PX4 SITL，经 **uXRCE-DDS**）原样跑一遍，用于赛前的移动障碍 / 窄缝 / 得分环调试。
>
> 适用组别：创意组（150–330mm 自制四旋翼，PX4 系闭环）。判分环境是真机，本仿真器只做算法验证。

---

## 修订说明（相对上一版，依据 `src/` 实际代码）

上一版把接入面写成 **MAVROS**，但你的真实集成代码全部走 **uXRCE-DDS + px4_msgs**，
两者话题名、消息类型、坐标转换位置都不同。本版据此重写，主要更正：

1. **接入面 MAVROS → uXRCE-DDS**。没有任何一个自研节点用 MAVROS。轨迹下行是
   `/position_cmd`(ENU) → `ego_px4_adapter` → `/ego/trajectory_setpoint`(NED) →
   `px4_offboard_pkg`(FLY 转发) → `/fmu/in/trajectory_setpoint`；视觉里程计上行是
   `lio_px4_bridge` → `/fmu/in/vehicle_visual_odometry`。见 `ego_px4_adapter_node.cpp`、
   `offboard_node.cpp`、`bridge_node.cpp`。
2. **EGO-Planner 是 v1**（无 ESDF、单条 B 样条、`use_distinctive_trajs=False` 有 SIGSEGV），
   不是 v2。z 方向靠 `lambda_z / virtual_ceil_height / virtual_ground_height` 参数，**没有**所谓"z 坐标补丁"。
3. **目标输入是 RViz `2D Goal Pose` → `/move_base_simple/goal`（单目标，`flight_type=1`）**，
   仓库里没有 waypoint 序列调度器。用 Unity 得分环序列要**新写**一个适配节点（第 9 节）。
4. **FAST-LIO 与 EGO 之间有 `topic_tools relay`**：`/Odometry`→`/drone_0_Odometry`、
   `/cloud_registered`→`/drone_0_cloud_registered`。EGO 订的是 `drone_0_` 前缀话题。
5. 新增两个**真实代码暴露、上一版完全漏掉**的硬约束——见下节红线 4、5。

---

## 0. 为什么是路线 B

仿真器自带控制链是 `PositionCommand → cascadePID → RPM → mars_drone_sim 动力学`，**没有 PX4**。
你的真实系统是 PX4 uXRCE-DDS 闭环。所以**丢掉仿真器的动力学和控制**
（`mars_drone_sim` + `cascadePID` + `diff_planner`），只留 Unity 做渲染，其余换成你自己的 stack。

---

## 1. 总体架构（uXRCE-DDS 版）

```
┌───────────────────────── ROS2 Humble (宿主机) ───────────────────────────┐
│                                                                          │
│  PX4 SITL (Gazebo Classic)                                               │
│   ├─ 动力学 / 姿态控制 / EKF2                                            │
│   ├─ uxrce_dds_client ──UDP──► Micro-XRCE-DDS-Agent ──► /fmu/in|out/*     │
│   └─ ground-truth 位姿 ──► [pose_relay] ──► /quad_0/lidar_slam/odom (真值) │
│                                                                          │
│  [里程碑1] pose_relay 同时发 /Odometry(map/NWU 真值) ──┐                   │
│  [里程碑2] Unity点云 ──► FAST-LIO ──► /Odometry ───────┤                  │
│                                                        ▼                  │
│  topic_tools relay ──► /drone_0_Odometry, /drone_0_cloud_registered       │
│                          │                                                │
│  ego_planner(v1) ──► /drone_0_planning/bspline ──► traj_server            │
│                          └─► /position_cmd (PositionCommand, ENU)          │
│  ego_px4_adapter ──► /ego/trajectory_setpoint (TrajectorySetpoint, NED)   │
│  px4_offboard_pkg(FLY 转发) ──► /fmu/in/trajectory_setpoint               │
│  [里程碑2] lio_px4_bridge ──► /fmu/in/vehicle_visual_odometry (喂 EKF2)    │
└──────────────────────────────────────────────────────────────────────────┘
        ▲ 真值 odom (ROS2→ROS1)          │ 点云 (ROS1→ROS2)
        │                                ▼
┌────────────────────────── ros1_bridge (dynamic_bridge) ──────────────────┐
│   只桥 2~3 个标准话题，无需自定义消息、无需重编译                          │
└──────────────────────────────────────────────────────────────────────────┘
        ▲                                │
        │                                ▼
┌────────────────────── ROS1 Noetic (Docker 容器) ─────────────────────────┐
│   ros_tcp_endpoint  ◄──TCP──►  Unity 仿真器二进制                          │
│   Unity 发布: /drone_0_pcl_render_node/cloud   (+ 得分环 UI，话题名待确认)  │
│   Unity 订阅: /quad_0/lidar_slam/odom  (据此渲染无人机与点云)              │
└──────────────────────────────────────────────────────────────────────────┘
```

### 跨边界话题（全部标准消息，已向仓库确认）

| 话题 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/drone_0_pcl_render_node/cloud` | `sensor_msgs/PointCloud2` | ROS1 → ROS2 | Unity 渲染的雷达点云（里程碑2 喂 FAST-LIO）|
| `/quad_0/lidar_slam/odom` | `nav_msgs/Odometry` | ROS2 → ROS1 | **PX4 真值位姿**，Unity 据此摆放无人机并渲染点云 |
| 得分环序列（UI "Target the ring"）| 疑为 `geometry_msgs/PoseArray` | ROS1 → ROS2 | 话题名仓库 README 未标，**需在 `rostopic list` 里现场确认**（上一版假设 `/publish_point`）|

> 仿真器是 **ROS1 Noetic**、MARSIM 血统；已确认它**只发点云、不发 IMU**（这决定了第 7、8 节的关键处理）。

---

## 2. 五条设计红线（前 3 条同上一版，后 2 条为本次据代码新增）

1. **喂 Unity 的 `/quad_0/lidar_slam/odom` 必须是 PX4 真值位姿，绝不能是 FAST-LIO 估计值。**
   否则「渲染→点云→估计→渲染」成闭环，误差被反馈放大直接发散。真值走一条线（→Unity），
   估计走另一条线（→EGO）。里程碑1 里两条线可以都用真值（因为还没上 FAST-LIO）。

2. **障碍碰撞不做物理仿真。** 无人机实际在 Gazebo 空世界里飞，障碍只存在于 Unity（点云形式）。
   撞不撞不由物理引擎判定，看规划/飞出的轨迹有没有穿过点云。评估：rviz 看轨迹清障，或用已知障碍坐标后处理 odom。

3. **坐标系对齐。** 但注意：**你的代码里 map 是 NWU（x 北 / y 西 / z 上），不是 ENU**——
   `ego_px4_adapter_node.cpp` 明写 `(x,y,z)_NED = (x_map, -y_map, -z_map)`、`yaw_NED = -yaw_map`；
   `bridge_node.cpp` 用绕 X 轴 180°（`kQ_MAP2NED`）做同一件事。所以 `pose_relay` 从 PX4 NED 真值
   转到 map/Unity 世界时用 **NED→NWU 即 `(x,-y,-z)`**，不是 ENU 的 `(y,x,-z)`。起飞点要对齐到 Unity 原点。

4. **【新】仿真器不提供 IMU，而 FAST-LIO 和 `lio_px4_bridge` 都强依赖 `/livox/imu`。**
   里程碑1 靠真值 odom 绕开；里程碑2 必须自建 IMU 源（第 8 节）。

5. **【新】`lio_px4_bridge` 把加速度 `×9.80665`，因为 Livox IMU 输出单位是 g。**
   （`bridge_node.cpp:159-163` 注释：`livox_ros_driver2` 没做单位换算。）任何标准 `sensor_msgs/Imu`
   （m/s²）原样喂进去会**大 9.8 倍直接发散**。里程碑2 上 FAST-LIO 时必须处理（第 8 节）。

---

## 3. 环境与前提

- Ubuntu 22.04 + ROS2 Humble（宿主机，已装你的 `ego_swarm` / `FAST_LIO` / 自研包）
- NVIDIA 显卡 + `nvidia-container-toolkit`（Unity 渲染要 GPU）、Docker
- PX4-Autopilot 源码（宿主机编译 SITL）
- **Micro-XRCE-DDS-Agent**（宿主机，SITL 与 px4_msgs 的桥）
- 仿真器二进制：按官方 README 从网盘下载，解压到仓库 `src/`

> **【关键】px4_msgs 版本必须与 PX4 SITL 固件对齐。** 你的 `offboard_node.cpp` 订的是
> **带版本后缀**的 `/fmu/out/vehicle_local_position_v1`、`/fmu/out/vehicle_status_v3`、
> 以及 `/fmu/out/failsafe_flags`、`/fmu/out/vehicle_land_detected`。这些后缀跟随 PX4 固件的 uXRCE
> 消息定义（`_v1/_v3` 是较新固件，约 v1.15+）。**SITL 的 PX4 版本要和仓库 `src/px4_msgs` 同源**，
> 否则 uXRCE 话题名/哈希对不上，`px4_offboard` 会一直卡在 INIT（`pos_valid_` 永远 false）。
> 先在 SITL 上 `ros2 topic list | grep fmu` 核对这四个话题名和你代码里写的完全一致。

> **PX4 后端**：默认 Gazebo Classic（真值位姿好取、Humble 最成熟）。gz Harmonic 请把第 6 节真值
> 来源改成 gz 桥的 `/model/x500/pose`；jMAVSim 不推荐。

---

## 4. Phase 1 — ROS1 侧：Docker 里只跑 Unity + ros_tcp_endpoint

**关键认知**：路线 B 里 ROS1 侧不编译整个工作空间，只要 `ros_tcp_endpoint`（纯 rospy）+ Unity 二进制。
`diff_planner / mars_drone_sim / cascadePID` 全部不编译、不启动。

### 4.1 起容器（GPU + X11）

宿主机放行 X11：`xhost +local:root`

```bash
docker run -it --name aac_sim \
  --gpus all --network host \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume $HOME/Aerial_Autonomy_Challenge:/root/ws \
  osrf/ros:noetic-desktop-full bash
```

### 4.2 容器内：只编 ros_tcp_endpoint

```bash
cd /root/ws
touch src/Utils/CATKIN_IGNORE
touch src/diff_planner/CATKIN_IGNORE
touch src/uav_simulator/CATKIN_IGNORE
touch src/user_command/CATKIN_IGNORE
touch src/Simulator-bridge/unity_robotics_demo/CATKIN_IGNORE
touch src/Simulator-bridge/unity_robotics_demo_msgs/CATKIN_IGNORE
source /opt/ros/noetic/setup.bash
catkin_make -j1          # 实际只编 ros_tcp_endpoint
source devel/setup.bash
```

### 4.3 启动 Unity + endpoint

```bash
# 终端A: roscore
# 终端B: source devel/setup.bash && roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000
# 终端C: ./sh_files/run_unity.sh   (或直接跑解压出的 .x86_64)
```

`rostopic list` 应能看到 `/drone_0_pcl_render_node/cloud`。**顺手确认得分环 UI 的话题名和类型**
（点一次 "Target the ring" 看 `rostopic list` 多出什么），填回第 1 节表格。

临时验证点云随位姿刷新：
```bash
rostopic pub -r 30 /quad_0/lidar_slam/odom nav_msgs/Odometry \
  '{header: {frame_id: "world"}, pose: {pose: {position: {x: 0,y: 0,z: 1}, orientation: {w: 1}}}}'
```

---

## 5. Phase 2 — ros1_bridge（宿主机）

用预编译动态桥。里程碑1 只需桥 `/quad_0/lidar_slam/odom`(ROS2→ROS1) 和点云(ROS1→ROS2)。

```bash
sudo apt install ros-humble-ros1-bridge
# 同时 source ROS1(能连容器 master) 与 ROS2
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
source /opt/ros/humble/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

> `dynamic_bridge` 要同时加载 ROS1+ROS2 环境。宿主机通常没 noetic，最省心是**再起一个专跑 bridge 的容器**
> （镜像同时含 noetic+humble），或用带 ros1_bridge 的双环境镜像。精确只桥这几条见附录 A。

验证：`ros2 topic hz /drone_0_pcl_render_node/cloud`（ROS2 侧看得到点云在动）。

---

## 6. Phase 3 — PX4 SITL + uXRCE-DDS + 真值位姿

### 6.1 起 PX4 SITL 与 Agent

```bash
# 终端1: Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888
# 终端2: PX4 SITL (会自动起 uxrce_dds_client 连 8888)
cd ~/PX4-Autopilot && make px4_sitl gazebo-classic
```

起来后核对话题（**必须和 offboard_node.cpp 里的名字一字不差**）：
```bash
ros2 topic list | grep fmu
# 期望看到: /fmu/out/vehicle_local_position_v1  /fmu/out/vehicle_status_v3
#           /fmu/out/failsafe_flags  /fmu/out/vehicle_land_detected
#           /fmu/in/trajectory_setpoint  /fmu/in/offboard_control_mode
#           /fmu/in/vehicle_command  /fmu/in/vehicle_visual_odometry
```
对不上 → 见第 3 节版本对齐警告。

### 6.2 GPS-denied（里程碑2 才需要，里程碑1 可跳过）

真机上你把 FAST-LIO odom 经 `lio_px4_bridge` 喂 `/fmu/in/vehicle_visual_odometry` 代替 GPS。SITL 要一致：
- 关 GPS、开外部视觉：`EKF2_EV_CTRL`（位姿/速度位）+ `EKF2_HGT_REF=Vision`（PX4≥v1.14）；
  `GPS_1_CONFIG=0` 或不喂 fake GPS。参数掩码以你真机上验证过的那套为基准搬过来最快。
- 里程碑1 不上 FAST-LIO，**可以直接用 SITL 自带 GPS/EKF2**（不触发红线1，因为喂 Unity 的是 ground-truth 另一条线）。

### 6.3 真值位姿 → Unity + （里程碑1）→ EGO（pose_relay 节点）

真值来源（Gazebo Classic）：`/gazebo/model_states`（或加 p3d ground-truth 插件）。
写一个 ROS2 节点，做 **NED→NWU** 转换后，**里程碑1 同时发两路**：

- `/quad_0/lidar_slam/odom`（给 Unity 渲染）
- `/Odometry`（`nav_msgs/Odometry`, frame=`camera_init`，给 EGO 当定位；里程碑2 关掉这一路，交给 FAST-LIO）

```python
# pose_relay.py (ROS2 Humble, rclpy)  —— 里程碑1 用
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

DRONE_MODEL = "iris"          # 按 SITL 机型名改
ORIGIN = (0.0, 0.0, 0.0)      # 起飞点相对 Unity 原点偏置，按需减掉
PUBLISH_LIO = True            # 里程碑1=True(真值当定位); 里程碑2=False(用 FAST-LIO)

class PoseRelay(Node):
    def __init__(self):
        super().__init__("pose_relay")
        self.pub_unity = self.create_publisher(Odometry, "/quad_0/lidar_slam/odom", 10)
        self.pub_lio   = self.create_publisher(Odometry, "/Odometry", 10) if PUBLISH_LIO else None
        self.create_subscription(ModelStates, "/gazebo/model_states", self.cb, 10)

    def cb(self, msg: ModelStates):
        if DRONE_MODEL not in msg.name:
            return
        i = msg.name.index(DRONE_MODEL)
        p, q = msg.pose[i].position, msg.pose[i].orientation
        # gazebo classic 默认输出 ENU/FLU。你的 map 是 NWU: 由 ENU→NWU 需 (x,y)->( y, -x)?
        # ★ 不要照抄符号：真值的原始系(取决于插件)务必现场用 rviz 标定后再定转换，
        #   目标是"无人机在 Unity/EGO 里的朝向、平移和实际一致"。先给恒等，标定后填。
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = "camera_init"
        od.child_frame_id = "body"
        od.pose.pose = msg.pose[i]
        od.pose.pose.position.x = p.x - ORIGIN[0]
        od.pose.pose.position.y = p.y - ORIGIN[1]
        od.pose.pose.position.z = p.z - ORIGIN[2]
        od.twist.twist = msg.twist[i]
        self.pub_unity.publish(od)
        if self.pub_lio: self.pub_lio.publish(od)

def main():
    rclpy.init(); rclpy.spin(PoseRelay()); rclpy.shutdown()
if __name__ == "__main__":
    main()
```

> ⚠️ 坐标转换的符号**必须在 rviz 里现场标定**：让无人机沿 Gazebo +x 平移一段，确认 EGO 的 map
> 里也朝对应方向动、Unity 里画面一致，再固化符号。别照抄——真值原始系随插件/机型而变，
> 而你的 map 是 NWU（第 2 节红线 3），错一个符号轨迹就镜像翻转。

---

## 7. 里程碑 1 — 真值定位，先把 planner→PX4 闭环 + 避障调通（推荐先做）

**思路**：暂时不上 FAST-LIO，用 `pose_relay` 的真值当 `/Odometry`，先验证
`ego_planner → traj_server → ego_px4_adapter → px4_offboard → PX4` 全链路和避障。
这样绕开仿真器无 IMU + Livox g 单位两个坑，最快看到无人机在 Unity 里避障飞行。

复用真实 launch，但**去掉传感器采集层**。`ego_real_flight.launch.py` 里第 1 段
`lihunlong_run_launch.py` 会起 `livox_ros_driver2 + FAST_LIO + lio_px4_bridge`——里程碑1 这三个都不要。
做一个 sim 变体 launch（建议新增 `top_launch_pkg/launch/ego_sim_flight.launch.py`），保留：

- 两个 identity static TF（`world`/`map`→`camera_init`）——原样
- `relay_odom`：`/Odometry`→`/drone_0_Odometry`——原样
- `relay_cloud`：`/cloud_registered`→`/drone_0_cloud_registered`——**里程碑1 没有 FAST-LIO 的
  `/cloud_registered`**，改成把 **Unity 点云** relay 过去：
  `/drone_0_pcl_render_node/cloud`→`/drone_0_cloud_registered`（EGO 的 grid_map 直接吃 Unity 点云）。
  注意点云 frame_id 要是 `camera_init`（可在 relay 前加个改 frame 的小节点，或让 grid_map 参数对齐）。
- `ego_advanced`（第 8.x 段参数）、`traj_server`、`ego_px4_adapter`、`px4_offboard`——**全部原样**
  （参数必须与 `ego_real_flight.launch.py` 的 `ego_advanced` 段保持一致，见 CLAUDE.md 约束）
- 额外起 `pose_relay`（第 6.3，`PUBLISH_LIO=True`）
- **不要** `lio_px4_bridge`（里程碑1 用 SITL 自带 EKF2）

`px4_offboard` 的 `enable_arm` 一样先 `false` 做 dry-run，再 `true`。目标仍走 RViz `2D Goal Pose`
（`/move_base_simple/goal`，frame=map），`flight_type=1`。

**验收**：RViz 发一个 goal，看 Unity 里无人机绕开点云障碍飞到目标、`px4_offboard` 走到
FLY→HOLD→（enable_arm 时）LAND/DISARM。这一步通了，再进里程碑2。

---

## 8. 里程碑 2 — 接 Unity 点云 + FAST-LIO（真实 SLAM，需先解决 IMU）

### 8.1 补 IMU 源（仿真器不发 IMU）

FAST-LIO 和 `lio_px4_bridge` 都要 `/livox/imu`。写一个转换节点，从 PX4 uXRCE 的
`/fmu/out/sensor_combined`（`px4_msgs/SensorCombined`，~200Hz，body-FRD，acc 单位 **m/s²**）
生成 `sensor_msgs/Imu`：

- FRD→FLU：`(x,-y,-z)`（和代码里 `kQ_FLU2FRD` 反向一致）。
- **单位**：`sensor_combined` 已是 m/s²。而 `lio_px4_bridge` 会 `×9.80665`（当 Livox g 处理）。二选一：
  - (a) 转换节点里把 acc **÷9.80665** 伪装成 g，喂原版 bridge（最省改动，但不直观）；**或**
  - (b) 给 `lio_px4_bridge` 加参数 `imu_acc_in_g`（默认 true 保真机行为，sim 置 false 不乘），转换节点直接给 m/s²。**推荐 (b)**，改动小且语义清晰。
- FAST-LIO 侧同理：`mid360.yaml` 假定 Livox 的量纲/时间戳，sim 要另建 config（下节）。

> 若不想碰 IMU，可以停在里程碑1（真值定位）就已经能测 planner+控制+避障——这是本方案推荐的主战场。
> 里程碑2 的价值仅在于额外验证 FAST-LIO 本身，成本高（IMU 时序/单位/外参都要标）。

### 8.2 FAST-LIO 接 Unity 点云

新建 `FAST_LIO/config/unity_sim.yaml`（**不要**改 `mid360.yaml`，实机在用）：

- `common/lid_topic: "/drone_0_pcl_render_node/cloud"`，`imu_topic:` 指到 8.1 的转换话题
- `preprocess/lidar_type: 2`（Velodyne 分支）或 `4`（通用点云）——别选 `1`(Livox)
- **核对点云字段**：有无 `ring`/`time`（逐点时间戳）。无 `time` 用无逐点时间分支，否则去畸变出错。
- `common/time_sync_en`：Unity 时钟与 PX4 SITL 不同源，先开 `use_sim_time`，必要时开时间同步。
- `extrinsic_T/R`：仿真里雷达≈IMU 同位，先给近似单位阵（`mid360.yaml` 里那组 `[-0.011,-0.02329,0.04412]`
  是实机标定值，sim 不适用），再按漂移微调。

然后把 `pose_relay` 的 `PUBLISH_LIO=False`（真值不再当定位，交给 FAST-LIO 的 `/Odometry`），
并把 sim launch 里的 `relay_cloud` 改回订 FAST-LIO 的 `/cloud_registered`。
可选：加回 `lio_px4_bridge`（已按 8.1 处理单位）喂 EKF2，复现真机 GPS-denied。

---

## 9. 得分环序列（Unity UI → EGO）

真实系统是**单目标** RViz 驱动，仓库里没有序列调度器。两种做法：

- **最简**：赛前已知 4 个得分环坐标，手动在 RViz 逐个发 `2D Goal Pose`；或写个几行的脚本按顺序
  发 `/move_base_simple/goal`。`offboard_node.cpp:on_goal` 已支持**收到新 goal 时从 HOLD/FINAL_CORRECTION
  回到 FLY**，所以逐点推进是现成的。
- **用 Unity UI**：新写一个 rclpy 节点，订阅得分环话题（第 4.3 现场确认的那个，疑为 `PoseArray`）→
  用 `/Odometry` 做到点检测 → 逐个把 `PoseStamped` 发到 `/move_base_simple/goal`。
  这是**新增代码**，不是"移植现有的 sequential_waypoint_bridge"（你仓库里没有这个文件）。

> 得分环有高度差（限高 2m、环半径 0.5m、窄缝 0.7m）：确认 goal 的 z 生效、`virtual_ceil_height`
> 和 `virtual_ground_height` 给的飞行带能覆盖各环高度（当前 `ego_real_flight.launch.py` 注释里
> 实际飞行带约 z∈[0.63,1.0]m，得分环若更高要相应放宽这些参数并同步 `ego_replay.launch.py`）。

---

## 10. 启动顺序 Checklist

**里程碑1（真值定位，先做）**
```
① Docker: roscore
② Docker: ros_tcp_endpoint
③ Docker: Unity 二进制
④ 宿主机: ros1_bridge —— 确认 cloud 桥到 ROS2、odom 能桥回 ROS1
⑤ 宿主机: MicroXRCEAgent udp4 -p 8888
⑥ 宿主机: PX4 SITL (gazebo-classic) —— ros2 topic list|grep fmu 核对话题名
⑦ 宿主机: pose_relay (PUBLISH_LIO=True) —— Unity 里无人机被摆到起飞点、点云刷新；rviz 标定坐标符号
⑧ 宿主机: ego_sim_flight.launch.py enable_arm:=false —— dry-run 看状态机与轨迹
⑨ RViz 2D Goal Pose 发目标；确认 Unity 里避障飞行；再 enable_arm:=true 真闭环
```

**里程碑2（+FAST-LIO）**：在上面基础上加 ⑦b IMU 转换节点、把 pose_relay 改 `PUBLISH_LIO=False`、
换 FAST-LIO `unity_sim.yaml`、relay_cloud 改回 `/cloud_registered`、（可选）加 `lio_px4_bridge`。

---

## 11. 常见坑速查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `px4_offboard` 一直卡 INIT | `/fmu/out/vehicle_local_position_v1` 等话题名/版本对不上，`pos_valid_` 永假 | 核对 px4_msgs 与 SITL 固件同源（第 3 节）；`ros2 topic list\|grep fmu` |
| ROS2 侧看不到 cloud | bridge 没同时 source 两套环境 / master URI 不对 | 见第 5 节，确认 `ROS_MASTER_URI` 指容器 master |
| Unity 里无人机不动 | 没人发 `/quad_0/lidar_slam/odom` 或没桥回 ROS1 | 查 pose_relay + bridge 的 ROS2→ROS1 方向 |
| 轨迹在 Unity 里镜像/翻转 | NED↔map 符号错（map 是 NWU 不是 ENU）| 第 2 节红线 3 + 第 6.3 rviz 标定 |
| FAST-LIO 一上来就发散 | 标准 IMU(m/s²) 被 `lio_px4_bridge` 又乘了 9.8 | 第 8.1，用 `imu_acc_in_g=false` 或÷9.8 |
| FAST-LIO 里程计抖/漂 | 点云无 `time` 却按逐点去畸变 / 时间不同步 | 无逐点时间分支 + `use_sim_time` |
| 估计越飞越飘 + Unity 画面异常 | 把估计 odom 误喂了 Unity（闭环）| 红线1：Unity 只吃真值 |
| 穿过障碍没"撞" | 障碍只在 Unity，Gazebo 空世界 | 正常，按轨迹清障判成功（红线2）|
| Unity 黑屏 | GPU 直通/X11 没配好 | `xhost +local:root` + `--gpus all` + nvidia-container-toolkit |

---

## 附录 A — parameter_bridge 精确桥接

```yaml
# bridge.yaml
topics:
  - {topic: /drone_0_pcl_render_node/cloud, type: sensor_msgs/msg/PointCloud2, queue_size: 10}
  - {topic: /quad_0/lidar_slam/odom,        type: nav_msgs/msg/Odometry,      queue_size: 10}
  # 若得分环序列走话题，确认类型后加一条
```
```bash
ros2 run ros1_bridge parameter_bridge --ros-args --params-file bridge.yaml
```

---

## 附录 B — 抗风扰测试

仿真器原生风扰在被丢掉的 `mars_drone_sim` 里，路线 B 用不到。要测抗风，在 **PX4 SITL 物理端加风**
（Gazebo Classic 的 `wind` 插件 / 世界文件 `<wind>`），而非仿真器的风。

---

*说明：本文命令为可运行脚手架。PX4 版本、EKF2 参数掩码、点云字段、坐标符号等随环境不同需现场标定，
文中已在对应处标注 ⚠️/★。ego_planner 参数以 `ego_real_flight.launch.py` 的 `ego_advanced` 段为准，
改任一处须同步 `ego_replay.launch.py`（CLAUDE.md 约束）。有拿不准处以你真机验证过的配置为基准。*
