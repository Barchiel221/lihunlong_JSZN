#!/usr/bin/env bash
# Open RViz and replay the newest ROS 2 bag under /bags.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/onekey_common.sh"

BAG_ROOT="${BAG_ROOT:-/home/orangepi/bags}"
BAG_PATH="${BAG_PATH:-}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$ONEKEY_WS_DIR/src/top_launch_pkg/rviz/ego_debug.rviz}"
RATE="${RATE:-1.0}"
LOOP="${LOOP:-false}"
START_EGO_REPLAY="${START_EGO_REPLAY:-false}"
PLAY_DELAY="${PLAY_DELAY:-2}"

usage() {
  cat <<EOF
Usage:
  scripts/replay_latest_bag_rviz.sh [options]

Options:
  --bag-root DIR       Directory containing bag folders (default: /home/orangepi/bags)
  --bag PATH           Replay this bag instead of auto-selecting newest
  --rviz-config FILE   RViz config file (default: top_launch_pkg/rviz/ego_debug.rviz)
  --rate VALUE         Playback rate, for example 0.5 or 2.0 (default: 1.0)
  --loop               Loop playback
  --ego-replay         Also launch top_launch_pkg ego_replay.launch.py
  -h, --help           Show this help

Environment overrides:
  BAG_ROOT=/home/orangepi/bags BAG_PATH=/path/to/bag RVIZ_CONFIG=/path/to/file.rviz
  RATE=1.0 LOOP=true START_EGO_REPLAY=true PLAY_DELAY=2
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-root)
      BAG_ROOT="$2"
      shift 2
      ;;
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --rviz-config)
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    --rate)
      RATE="$2"
      shift 2
      ;;
    --loop)
      LOOP=true
      shift
      ;;
    --ego-replay)
      START_EGO_REPLAY=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      onekey_fail "unknown arg: $1"
      ;;
  esac
done

find_latest_bag() {
  local root="$1"
  local latest=""

  latest="$(
    find "$root" -mindepth 2 -maxdepth 2 -name metadata.yaml -printf '%T@ %h\n' 2>/dev/null \
      | sort -nr \
      | head -n 1 \
      | cut -d' ' -f2-
  )"
  if [[ -n "$latest" ]]; then
    echo "$latest"
    return 0
  fi

  find "$root" -mindepth 1 -maxdepth 1 \( -name '*.db3' -o -name '*.mcap' \) -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr \
    | head -n 1 \
    | cut -d' ' -f2-
}

cleanup_pids=()
cleanup() {
  local pid
  for pid in "${cleanup_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

onekey_source_ros
command -v ros2 >/dev/null 2>&1 || onekey_fail "ros2 command not found after sourcing environment"
command -v rviz2 >/dev/null 2>&1 || onekey_fail "rviz2 command not found after sourcing environment"

if [[ -z "$BAG_PATH" ]]; then
  [[ -d "$BAG_ROOT" ]] || onekey_fail "bag root does not exist: $BAG_ROOT"
  BAG_PATH="$(find_latest_bag "$BAG_ROOT")"
fi

[[ -n "$BAG_PATH" ]] || onekey_fail "no ROS 2 bag found under: $BAG_ROOT"
[[ -e "$BAG_PATH" ]] || onekey_fail "bag path does not exist: $BAG_PATH"

PLAY_ARGS=(play "$BAG_PATH" --clock -r "$RATE")
if [[ "$LOOP" == true ]]; then
  PLAY_ARGS+=(--loop)
fi

onekey_info "bag: $BAG_PATH"
onekey_info "rate: $RATE  loop: $LOOP"

if [[ "$START_EGO_REPLAY" == true ]]; then
  onekey_info "launch: top_launch_pkg ego_replay.launch.py"
  ros2 launch top_launch_pkg ego_replay.launch.py &
  cleanup_pids+=("$!")
fi

RVIZ_ARGS=()
if [[ -f "$RVIZ_CONFIG" ]]; then
  RVIZ_ARGS=(-d "$RVIZ_CONFIG")
  onekey_info "rviz config: $RVIZ_CONFIG"
else
  onekey_info "rviz config not found, starting empty RViz: $RVIZ_CONFIG"
fi

rviz2 "${RVIZ_ARGS[@]}" --ros-args -p use_sim_time:=true &
cleanup_pids+=("$!")

sleep "$PLAY_DELAY"
onekey_info "play: ros2 bag ${PLAY_ARGS[*]}"
ros2 bag "${PLAY_ARGS[@]}"
