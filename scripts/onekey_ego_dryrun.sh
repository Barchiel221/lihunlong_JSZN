#!/usr/bin/env bash
# Start the full EGO + PX4 stack without arming. Uses mission.yaml by default.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/onekey_common.sh"

MISSION_PATH="$(onekey_default_mission)"
PROFILE="${PROFILE:-standard}"
RVIZ="${RVIZ:-false}"
DEBUG_RVIZ="${DEBUG_RVIZ:-false}"
FINAL_CORRECTION="${FINAL_CORRECTION:-true}"
PRECISION_XY="${PRECISION_XY:-0.12}"
PRECISION_Z="${PRECISION_Z:-0.15}"

onekey_validate_mission "$MISSION_PATH"
onekey_source_ros

onekey_info "launch: ego_real_flight.launch.py enable_arm:=false profile:=$PROFILE"
onekey_info "mission: $MISSION_PATH"

exec ros2 launch top_launch_pkg ego_real_flight.launch.py \
  enable_arm:=false \
  rviz:="$RVIZ" \
  debug_rviz:="$DEBUG_RVIZ" \
  final_correction:="$FINAL_CORRECTION" \
  precision_xy_tolerance:="$PRECISION_XY" \
  precision_z_tolerance:="$PRECISION_Z" \
  profile:="$PROFILE" \
  mission_file:="$MISSION_PATH"
