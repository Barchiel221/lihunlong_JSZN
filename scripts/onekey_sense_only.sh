#!/usr/bin/env bash
# Start MID360 + FAST-LIO + PX4 visual odometry bridge only.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/onekey_common.sh"

RVIZ="${RVIZ:-false}"

onekey_source_ros
onekey_info "launch: rc_altitude_mid360_velocity.launch.py rviz:=$RVIZ"

exec ros2 launch top_launch_pkg rc_altitude_mid360_velocity.launch.py \
  rviz:="$RVIZ"
