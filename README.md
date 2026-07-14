# lihunlong_JSZN

ROS 2 Humble workspace for the aerial embodied-intelligence challenge stack. The workspace integrates Livox MID360 sensing, FAST-LIO odometry, EGO-Planner local planning, PX4 Offboard control, and mission-level waypoint execution for GPS-denied flight.

## What This Workspace Does

The main real-flight data flow is:

```text
Livox MID360
  -> livox_ros_driver2
  -> FAST-LIO (/Odometry, /cloud_registered)
  -> lio_px4_bridge (/fmu/in/vehicle_visual_odometry)
  -> EGO-Planner (/drone_0_planning/bspline)
  -> traj_server (/position_cmd)
  -> ego_px4_adapter (/ego/trajectory_setpoint)
  -> px4_offboard_pkg (/fmu/in/trajectory_setpoint)
```

Mission goals are published to `/move_base_simple/goal` in ENU `map` frame. `px4_offboard_pkg` handles Offboard mode, optional arming, external EGO trajectory forwarding, final MID360-based correction, landing, and disarm.

## Repository Layout

```text
.
├── src/
│   ├── FAST_LIO/              # LiDAR-inertial odometry
│   ├── ego_swarm/             # EGO-Planner and simulator-related packages
│   ├── ego_px4_adapter/       # ENU PositionCommand -> PX4 NED TrajectorySetpoint
│   ├── lio_px4_bridge/        # FAST-LIO odometry -> PX4 visual odometry bridge
│   ├── livox_ros_driver2/     # Livox MID360 ROS 2 driver
│   ├── px4_msgs/              # PX4 message definitions
│   ├── px4_offboard_pkg/      # PX4 Offboard flight-state controller
│   └── top_launch_pkg/        # Top-level launch/config/orchestration package
├── scripts/                   # One-key launch, mission, replay, and helper scripts
├── hunlong/                   # Competition notes, plans, manuals, and reports
├── colcon.meta                # Workspace build metadata
├── build/                     # Local build output, not for Git
├── install/                   # Local install output, not for Git
└── log/                       # Local colcon/runtime logs, not for Git
```

## Requirements

- Ubuntu with ROS 2 Humble
- PX4 side running Micro XRCE-DDS client
- Micro-XRCE-DDS-Agent on the companion computer
- Livox SDK and MID360 network configuration
- Common ROS dependencies: `colcon`, `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2`, `pcl_ros`, `pcl_conversions`, `topic_tools`, `rviz2`
- Python dependencies used by helper scripts: `yaml`

For the current race scripts, the expected defaults are:

- MID360 network interface: `enP3p49s0`
- MID360 local IP: `192.168.1.5`
- PX4 XRCE serial device: `/dev/ttyS6`

These can be overridden by environment variables in `scripts/race_start.sh`.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you only changed launch/config/scripts inside `top_launch_pkg`, a smaller rebuild is usually enough:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select top_launch_pkg
source install/setup.bash
```

## Main Launch Files

### Sensor, SLAM, and PX4 Bridge

```bash
ros2 launch top_launch_pkg lihunlong_run_launch.py
```

Starts the base sensing stack around Livox, FAST-LIO, and the LIO-to-PX4 visual odometry bridge.

### Real-Flight EGO Stack

Dry-run first:

```bash
ros2 launch top_launch_pkg ego_real_flight.launch.py enable_arm:=false
```

Real arming only after hardware checks:

```bash
ros2 launch top_launch_pkg ego_real_flight.launch.py enable_arm:=true
```

Useful arguments:

```text
enable_arm:=false|true
rviz:=false|true
debug_rviz:=false|true
profile:=standard|fast
mission_file:=/path/to/mission.yaml
max_state_seconds:=30.0
final_correction:=true|false
precision_xy_tolerance:=0.10
precision_z_tolerance:=0.15
```

### Easy Manual/Auto Goal Flight

```bash
ros2 launch top_launch_pkg ego_real_flight_easy.launch.py enable_arm:=false debug_rviz:=true
```

This wrapper can optionally record bags and publish a simple automatic goal.

## One-Key Scripts

The `scripts/` directory provides thin wrappers around `top_launch_pkg`.

```bash
scripts/onekey_sense_only.sh
```

Starts MID360, FAST-LIO, and the PX4 odometry bridge only.

```bash
scripts/onekey_ego_dryrun.sh
```

Starts the full EGO + PX4 stack with `enable_arm:=false`.

```bash
scripts/onekey_ego_arm.sh
```

Starts the EGO + PX4 stack with arming enabled, but without automatically publishing mission waypoints.

```bash
scripts/onekey_start_mission.sh
```

Publishes the configured `mission.yaml` to an already-running stack.

```bash
scripts/race_start.sh
```

Competition-oriented entrypoint with hardware prechecks and full rosbag recording.

Examples:

```bash
scripts/race_start.sh
scripts/race_start.sh --arm
scripts/race_start.sh --arm --takeoff-only
scripts/race_start.sh --arm --profile fast
MISSION=/tmp/mission.yaml scripts/race_start.sh --arm
```

See [scripts/README_onekey.md](scripts/README_onekey.md) for the script-specific interface.

## Mission File

The main mission file is:

```text
src/top_launch_pkg/config/mission.yaml
```

It is read by `mission_executor.py` and publishes goals in order to `/move_base_simple/goal`.

Each waypoint uses ENU coordinates:

```yaml
waypoints:
  - pos: [2.1, -1.4, 1.3]
  - pos: [5.1, 0.18, 1.3]
    fly_through: true
    tolerance: {xy: 0.20, z: 0.15}
```

Supported waypoint fields:

- `pos`: required `[x, y, z]` in meters, ENU, frame `map`
- `fly_through`: optional, for pass-through gates or guide points
- `tolerance`: optional per-point tolerance override
- `segment_params`: optional planner/runtime parameter changes before a segment

For race-site tuning, edit `mission.yaml` first. Avoid changing flight code during field setup unless the behavior itself needs to change.

## Real-Flight Checklist

Before `enable_arm:=true`:

1. Confirm props, battery, frame, and kill/disarm path.
2. Confirm `/usr/local/lib/liblivox*.so` exists.
3. Confirm MID360 interface and IP:

   ```bash
   ip addr show enP3p49s0
   ```

4. Confirm PX4 XRCE serial device:

   ```bash
   ls /dev/ttyS6
   ```

5. Build and source the workspace:

   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

6. Run dry-run first:

   ```bash
   scripts/race_start.sh
   ```

7. Arm only after setpoints, odometry, and planner visualization look correct:

   ```bash
   scripts/race_start.sh --arm
   ```

## Rosbag and Replay

`race_start.sh` records all topics to:

```text
/home/orangepi/bags/ego_real_flight_<timestamp>
```

Replay the latest bag with RViz helper:

```bash
scripts/replay_latest_bag_rviz.sh
```

## Common Topics

```text
/livox/lidar
/livox/imu
/Odometry
/cloud_registered
/drone_0_Odometry
/drone_0_cloud_registered
/move_base_simple/goal
/position_cmd
/ego/trajectory_setpoint
/fmu/in/vehicle_visual_odometry
/fmu/in/trajectory_setpoint
```

## Notes

- `build/`, `install/`, and `log/` are generated locally and should not be uploaded as source.
- `enable_arm:=false` is the default safe mode for dry-run testing.
- `profile:=fast` should only be used after the standard profile is stable.
- The final landing point should be a normal waypoint, not `fly_through`, so `px4_offboard_pkg` can run final correction, landing, and disarm.
