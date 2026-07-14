#!/usr/bin/env bash
# Shared helpers for one-key launch scripts.

set -euo pipefail

onekey_script_dir() {
  cd "$(dirname "${BASH_SOURCE[0]}")" && pwd
}

ONEKEY_SCRIPT_DIR="$(onekey_script_dir)"
ONEKEY_WS_DIR="$(dirname "$ONEKEY_SCRIPT_DIR")"

onekey_fail() {
  echo "[onekey] ERROR: $*" >&2
  exit 1
}

onekey_info() {
  echo "[onekey] $*"
}

onekey_source_ros() {
  [[ -f /opt/ros/humble/setup.bash ]] || onekey_fail "missing /opt/ros/humble/setup.bash"
  [[ -f "$ONEKEY_WS_DIR/install/setup.bash" ]] || onekey_fail "missing $ONEKEY_WS_DIR/install/setup.bash; run colcon build first"

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$ONEKEY_WS_DIR/install/setup.bash"
  set -u
}

onekey_default_mission() {
  echo "${MISSION:-$ONEKEY_WS_DIR/src/top_launch_pkg/config/mission.yaml}"
}

onekey_validate_mission() {
  local mission="$1"
  [[ -f "$mission" ]] || onekey_fail "mission file does not exist: $mission"

  python3 - "$mission" <<'PY'
import sys
import yaml

path = sys.argv[1]
try:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
except Exception as exc:
    print(f"[onekey] ERROR: invalid mission yaml: {path}: {exc}", file=sys.stderr)
    sys.exit(1)

waypoints = (data or {}).get("waypoints")
if not isinstance(waypoints, list) or not waypoints:
    print(f"[onekey] ERROR: mission has no waypoint list: {path}", file=sys.stderr)
    sys.exit(1)

for index, waypoint in enumerate(waypoints, start=1):
    pos = waypoint.get("pos") if isinstance(waypoint, dict) else None
    if not isinstance(pos, list) or len(pos) != 3:
        print(f"[onekey] ERROR: waypoint {index} must contain pos: [x, y, z]", file=sys.stderr)
        sys.exit(1)
    try:
        [float(value) for value in pos]
    except Exception:
        print(f"[onekey] ERROR: waypoint {index} pos must be numeric", file=sys.stderr)
        sys.exit(1)

print(f"[onekey] mission OK: {path} ({len(waypoints)} waypoints)")
PY
}
