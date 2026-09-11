#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MUJOCO_PID=""
TELEOP_PID=""

cleanup() {
  trap - EXIT INT TERM
  [[ -z "$TELEOP_PID" ]] || kill -TERM "$TELEOP_PID" 2>/dev/null || true
  [[ -z "$MUJOCO_PID" ]] || kill -TERM "$MUJOCO_PID" 2>/dev/null || true
  [[ -z "$TELEOP_PID" ]] || wait "$TELEOP_PID" 2>/dev/null || true
  [[ -z "$MUJOCO_PID" ]] || wait "$MUJOCO_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ -z "${DISPLAY:-}" ]] && [[ -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "错误：未检测到图形显示会话。仿真遥操推荐在连接 HDMI 显示器的本地主机运行。" >&2
  exit 1
fi
if [[ ! -r "$ROOT_DIR/install/setup.bash" ]]; then
  echo "错误：工程尚未编译，请先运行 src/scripts/build_xrtele.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

ros2 launch mujoco_simulator simulate.launch.py &
MUJOCO_PID=$!

for _ in $(seq 1 50); do
  if ros2 service type /unpause_mujoco >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$MUJOCO_PID" 2>/dev/null; then
    echo "错误：MuJoCo 在启动阶段退出。" >&2
    exit 1
  fi
  sleep 0.2
done

if ! ros2 service type /unpause_mujoco >/dev/null 2>&1; then
  echo "错误：10 秒内未发现 /unpause_mujoco。" >&2
  exit 1
fi

ros2 service call /unpause_mujoco std_srvs/srv/Empty '{}' >/dev/null
ros2 launch qiling_kinematics xr_teleop_real.launch.py &
TELEOP_PID=$!

echo "仿真遥操已启动。Quest 连接后，先释放 Grip，再按住对应 Grip 控制手臂。"
echo "按 Ctrl+C 停止 MuJoCo 和遥操节点。"

set +e
wait -n "$MUJOCO_PID" "$TELEOP_PID"
status=$?
set -e
exit "$status"
