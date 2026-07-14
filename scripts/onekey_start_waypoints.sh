#!/usr/bin/env bash
# Publish an inline multi-waypoint list to an already-running EGO flight stack.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/onekey_common.sh"

WAYPOINTS="${WAYPOINTS:-2,0.5,1;2,2,1;4,4,1}"
WAIT_SUBSCRIBERS="${WAIT_SUBSCRIBERS:-2}"
XY_TOL="${XY_TOL:-0.35}"
Z_TOL="${Z_TOL:-0.25}"
VEL_TOL="${VEL_TOL:-0.20}"
SETTLE_SEC="${SETTLE_SEC:-1.0}"

onekey_source_ros

onekey_info "starting inline waypoint publisher only"
onekey_info "waypoints: $WAYPOINTS"

exec ros2 launch top_launch_pkg ego_multi_goal.launch.py \
  waypoints:="$WAYPOINTS" \
  wait_subscribers:="$WAIT_SUBSCRIBERS" \
  xy_tol:="$XY_TOL" \
  z_tol:="$Z_TOL" \
  vel_tol:="$VEL_TOL" \
  settle_sec:="$SETTLE_SEC"
