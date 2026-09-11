#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONVERTER_PID=""
TELEOP_PID=""

cleanup() {
  trap - EXIT INT TERM
  [[ -z "$TELEOP_PID" ]] || kill -TERM "$TELEOP_PID" 2>/dev/null || true
  [[ -z "$CONVERTER_PID" ]] || kill -TERM "$CONVERTER_PID" 2>/dev/null || true
  [[ -z "$TELEOP_PID" ]] || wait "$TELEOP_PID" 2>/dev/null || true
  [[ -z "$CONVERTER_PID" ]] || wait "$CONVERTER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -r "$ROOT_DIR/install/setup.bash" ]]; then
  echo "错误：工程尚未编译，请先运行 src/scripts/build_xrtele.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-16}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

echo "真机模式：ROS_DOMAIN_ID=$ROS_DOMAIN_ID，RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "确认机器人 SDK 已启动、双臂周围无障碍物并准备好急停。"

ros2 run topic_convertor topic_converter_node --ros-args \
  -p expected_motor_count:=26 \
  -p enable_state_bridge:=true \
  -p enable_command_bridge:=true \
  -p strict_command_size:=true &
CONVERTER_PID=$!

state_wait_sec="${QILING_STATE_WAIT_SEC:-30}"
if ! timeout "$state_wait_sec" ros2 topic echo /human_lower_state --once >/dev/null 2>&1; then
  echo "错误：${state_wait_sec} 秒内未收到 /human_lower_state。请检查 SDK、DDS 和 ROS_DOMAIN_ID。" >&2
  exit 2
fi

echo "已收到真机状态，即将自动执行：当前姿态 → 过渡点 → home → 遥操。"
ros2 launch qiling_kinematics_real xr_teleop_real.launch.py &
TELEOP_PID=$!

echo "真机遥操已启动。按 Ctrl+C 停止遥操和 topic_convertor。"
set +e
wait -n "$CONVERTER_PID" "$TELEOP_PID"
status=$?
set -e
exit "$status"
