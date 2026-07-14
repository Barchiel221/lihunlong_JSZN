#!/usr/bin/env bash
# Start the flight stack with arming enabled, then wait in takeoff hover.
# Publish waypoints separately with onekey_start_mission.sh or onekey_start_waypoints.sh.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/onekey_common.sh"

PROFILE="${PROFILE:-standard}"
MAX_STATE_SECONDS="${MAX_STATE_SECONDS:-300.0}"

onekey_info "delegating to race_start.sh --arm --takeoff-only"
onekey_info "profile: $PROFILE"
onekey_info "max_state_seconds: $MAX_STATE_SECONDS"

exec "$SCRIPT_DIR/race_start.sh" \
  --arm \
  --takeoff-only \
  --profile "$PROFILE" \
  --max-state-seconds "$MAX_STATE_SECONDS" \
  "$@"
