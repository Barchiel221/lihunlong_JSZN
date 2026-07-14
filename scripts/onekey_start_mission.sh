#!/usr/bin/env bash
# Publish mission.yaml waypoints to an already-running EGO flight stack.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/onekey_common.sh"

MISSION_PATH="$(onekey_default_mission)"
WAIT_SUBSCRIBERS="${WAIT_SUBSCRIBERS:-2}"
VEL_TOL="${VEL_TOL:-0.20}"
SETTLE_SEC="${SETTLE_SEC:-1.0}"

onekey_validate_mission "$MISSION_PATH"
onekey_source_ros

onekey_info "starting mission publisher only"
onekey_info "mission: $MISSION_PATH"

exec ros2 launch top_launch_pkg mission_start.launch.py \
  mission_file:="$MISSION_PATH" \
  wait_subscribers:="$WAIT_SUBSCRIBERS" \
  vel_tol:="$VEL_TOL" \
  settle_sec:="$SETTLE_SEC"
