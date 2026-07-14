#!/usr/bin/env bash
# Start the packaged helper launch for manual RViz goal or simple auto-goal tests.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/onekey_common.sh"

ENABLE_ARM="${ENABLE_ARM:-false}"
RECORD="${RECORD:-true}"
RVIZ="${RVIZ:-false}"
DEBUG_RVIZ="${DEBUG_RVIZ:-true}"
AUTO_GOAL="${AUTO_GOAL:-false}"
GOAL_DELAY="${GOAL_DELAY:-2.0}"
GOAL_X="${GOAL_X:-2.5}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_Z="${GOAL_Z:-0.8}"
BAG_DIR="${BAG_DIR:-$HOME/bags}"

onekey_source_ros

onekey_info "launch: ego_real_flight_easy.launch.py enable_arm:=$ENABLE_ARM record:=$RECORD auto_goal:=$AUTO_GOAL"

exec ros2 launch top_launch_pkg ego_real_flight_easy.launch.py \
  enable_arm:="$ENABLE_ARM" \
  record:="$RECORD" \
  rviz:="$RVIZ" \
  debug_rviz:="$DEBUG_RVIZ" \
  auto_goal:="$AUTO_GOAL" \
  goal_delay:="$GOAL_DELAY" \
  goal_x:="$GOAL_X" \
  goal_y:="$GOAL_Y" \
  goal_z:="$GOAL_Z" \
  bag_dir:="$BAG_DIR"
