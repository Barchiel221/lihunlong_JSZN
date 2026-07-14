# One-Key Launch Scripts

These scripts are thin wrappers around `top_launch_pkg` launch files.

## Common entries

- `scripts/onekey_sense_only.sh`
  Starts MID360, FAST-LIO, and the LIO-to-PX4 bridge only.

- `scripts/onekey_ego_dryrun.sh`
  Starts the full EGO + PX4 stack with `enable_arm:=false`, using `src/top_launch_pkg/config/mission.yaml`.

- `scripts/onekey_ego_arm.sh`
  Starts the race stack with arming enabled, but does not publish waypoints. It takes off and waits in hover.

- `scripts/onekey_start_mission.sh`
  Publishes `mission.yaml` waypoints to an already-running stack.

- `scripts/onekey_start_waypoints.sh`
  Publishes an inline `WAYPOINTS='x,y,z;...'` list to an already-running stack.

- `scripts/onekey_ego_manual.sh`
  Starts `ego_real_flight_easy.launch.py` for RViz manual-goal or simple auto-goal testing.

## Environment overrides

Examples:

```bash
PROFILE=fast scripts/onekey_ego_dryrun.sh
MISSION=/tmp/mission.yaml PROFILE=standard scripts/onekey_ego_arm.sh
RVIZ=true scripts/onekey_sense_only.sh
AUTO_GOAL=true GOAL_X=2.0 GOAL_Y=0.0 GOAL_Z=0.8 scripts/onekey_ego_manual.sh
MISSION=/tmp/mission.yaml scripts/onekey_start_mission.sh
WAYPOINTS='1,0,1;2,0,1;2,1,1' scripts/onekey_start_waypoints.sh
```

`onekey_ego_dryrun.sh` and `onekey_start_mission.sh` validate the mission file before launch.
