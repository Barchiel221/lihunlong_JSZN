#!/usr/bin/env bash
# 国赛一键启动脚本 (改动 G)。
#
# 预检硬件前提 -> 全量录 bag(检录/申诉证据) -> ros2 launch 实飞栈。
# 目标: 上电到 setpoint 流出 < 3min(计划书风险清单第 6 条)。
#
# 用法:
#   scripts/race_start.sh                      # dry-run(enable_arm:=false), 用默认 mission.yaml
#   scripts/race_start.sh --arm                # 真飞(enable_arm:=true)
#   scripts/race_start.sh --arm --profile fast # 提速档真飞
#   MISSION=/path/to/mission.yaml scripts/race_start.sh --arm
#   scripts/race_start.sh --skip-checks        # 跳过硬件预检(仅调试用)
#
# 环境变量: MISSION / PROFILE / NET_IF / NET_IP / SERIAL_DEV 可覆盖默认。
set -euo pipefail

# ---- 定位工作区根(脚本在 <ws>/scripts/ 下) ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# ---- 默认参数 ----
ARM=false
PROFILE="${PROFILE:-standard}"
MISSION="${MISSION:-$WS_DIR/install/top_launch_pkg/share/top_launch_pkg/config/mission.yaml}"
NET_IF="${NET_IF:-enP3p49s0}"
NET_IP="${NET_IP:-192.168.1.5}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyS6}"
SKIP_CHECKS=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm)          ARM=true; shift;;
    --profile)      PROFILE="$2"; shift 2;;
    --mission)      MISSION="$2"; shift 2;;
    --skip-checks)  SKIP_CHECKS=true; shift;;
    -h|--help)      grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

fail() { echo "[race_start] ✗ $1" >&2; exit 1; }
ok()   { echo "[race_start] ✓ $1"; }

# ---- 硬件预检(CLAUDE.md 启动前提) ----
if [[ "$SKIP_CHECKS" != true ]]; then
  echo "[race_start] 预检硬件前提..."
  # 1. Livox SDK .so 须在 /usr/local/lib (否则 dlopen error)
  if ls /usr/local/lib/liblivox_sdk* >/dev/null 2>&1 || ls /usr/local/lib/liblivox* >/dev/null 2>&1; then
    ok "Livox SDK .so 在 /usr/local/lib"
  else
    fail "未找到 /usr/local/lib/liblivox*.so —— 会 dlopen error。装 SDK 或 --skip-checks"
  fi
  # 2. 网口 IP (否则 MID360 bind failed)
  if ip addr show "$NET_IF" 2>/dev/null | grep -q "$NET_IP"; then
    ok "$NET_IF 已配 $NET_IP"
  else
    fail "$NET_IF 未配 $NET_IP —— MID360 会 bind failed。sudo ip addr add $NET_IP/24 dev $NET_IF"
  fi
  # 3. PX4 串口 (Micro-XRCE-DDS-Agent 走 ttyS6)
  if [[ -e "$SERIAL_DEV" ]]; then
    ok "$SERIAL_DEV 存在"
  else
    fail "$SERIAL_DEV 不存在 —— PX4 XRCE 桥无法连接"
  fi
fi

# ---- mission 文件检查 ----
if [[ ! -f "$MISSION" ]]; then
  fail "mission 文件不存在: $MISSION (先 colcon build --packages-select top_launch_pkg)"
fi
ok "mission 文件: $MISSION"

# ---- source 环境 ----
# ROS 2 的 setup.bash 会引用未定义变量(AMENT_TRACE_SETUP_FILES 等)，
# 与 set -u(nounset) 冲突会直接退出，故 source 期间临时关闭 nounset。
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u
ok "ROS 2 环境已 source"

# ---- 全量录 bag ----
TS="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="/home/orangepi/bags/ego_real_flight_${TS}"
echo "[race_start] 开始全量录 bag -> $BAG_DIR"
ros2 bag record -a -o "$BAG_DIR" >/dev/null 2>&1 &
BAG_PID=$!
# launch 退出时一并停 bag
trap 'echo "[race_start] 停止 bag ($BAG_PID)"; kill -INT "$BAG_PID" 2>/dev/null || true; wait "$BAG_PID" 2>/dev/null || true' EXIT

echo "[race_start] ============================================="
echo "[race_start]  enable_arm = $ARM   (false=dry-run 不解锁)"
echo "[race_start]  profile    = $PROFILE"
echo "[race_start]  mission    = $MISSION"
echo "[race_start]  bag        = $BAG_DIR"
echo "[race_start] ============================================="

# ---- 拉起实飞栈 ----
# 不用 exec: exec 会替换本 shell 进程,使上面注册的 EXIT trap 失效,录 bag 子进程
# 变成孤儿不被回收。保留为普通子进程,launch 退出后 trap 停 bag。
ros2 launch top_launch_pkg ego_real_flight.launch.py \
  enable_arm:="$ARM" \
  profile:="$PROFILE" \
  mission_file:="$MISSION"
