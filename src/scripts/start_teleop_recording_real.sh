#!/usr/bin/env bash

# Start the real-robot ROS stack in its required order:
#   1. topic converter
#   2. Quest teleoperation
#   3. RGB episode recorder
#
# XRoboToolkit itself is intentionally not started here.  Start it first with
# start_xrobotoolkit_real.sh and confirm that Quest controller topics are live.

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

# These may be overridden without editing the script, for example:
#   CONVERTER_DELAY_SEC=3 TELEOP_DELAY_SEC=5 ./src/scripts/start_teleop_recording_real.sh
CONVERTER_DELAY_SEC="${CONVERTER_DELAY_SEC:-2}"
TELEOP_DELAY_SEC="${TELEOP_DELAY_SEC:-2}"

if [[ ! -r "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "错误：找不到 ${WORKSPACE_ROOT}/install/setup.bash。请先在项目根目录完成 colcon build。" >&2
  exit 1
fi

if ! command -v setsid >/dev/null 2>&1; then
  echo "错误：未找到 setsid，无法为 ROS launch 建立独立进程组。" >&2
  exit 1
fi

# Colcon-generated setup scripts may read COLCON_TRACE before it exists.  Keep
# strict mode for this launcher, but temporarily disable nounset while sourcing
# the overlay.
set +u
# shellcheck disable=SC1090
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

declare -a CHILD_PIDS=()
declare -a CHILD_LABELS=()

start_child() {
  local label="$1"
  shift

  echo "启动 ${label}：$*"
  # Every launch owns a separate session/process group.  This lets cleanup
  # terminate a ros2 launch and every node that it spawned, including a node
  # configured with launch respawn=True.
  setsid "$@" &
  CHILD_PIDS+=("$!")
  CHILD_LABELS+=("${label}")
}

group_is_running() {
  local group_leader_pid="$1"
  kill -0 -- "-${group_leader_pid}" 2>/dev/null
}

signal_groups() {
  local signal_name="$1"
  local index
  local pid

  for ((index=${#CHILD_PIDS[@]} - 1; index>=0; --index)); do
    pid="${CHILD_PIDS[index]}"
    if group_is_running "${pid}"; then
      echo "向 ${CHILD_LABELS[index]} 进程组发送 SIG${signal_name}，PGID=${pid}"
      kill -s "${signal_name}" -- "-${pid}" 2>/dev/null || true
    fi
  done
}

any_group_running() {
  local pid
  for pid in "${CHILD_PIDS[@]}"; do
    if group_is_running "${pid}"; then
      return 0
    fi
  done
  return 1
}

wait_for_groups() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))

  while any_group_running; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 0.1
  done

  return 0
}

cleanup() {
  local exit_code=$?
  local pid
  trap - EXIT INT TERM

  echo
  echo "正在停止本脚本启动的 ROS 节点..."
  signal_groups INT
  wait_for_groups 5 || true

  # A launch process may be blocked while a respawned child is starting.  Do
  # not leave that process tree alive after the graceful interrupt window.
  if any_group_running; then
    signal_groups TERM
    wait_for_groups 3 || true
  fi
  if any_group_running; then
    signal_groups KILL
  fi

  for pid in "${CHILD_PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done

  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

echo "=== 启动真机遥操与录制 ROS 节点 ==="
echo "topic converter 等待：${CONVERTER_DELAY_SEC}s；遥操到录制等待：${TELEOP_DELAY_SEC}s"

start_child "topic converter" \
  ros2 run topic_convertor topic_converter_node
sleep "${CONVERTER_DELAY_SEC}"

start_child "Quest 真机遥操" \
  ros2 launch qiling_kinematics_real xr_teleop_real.launch.py pxrea_adapter_respawn:=false
sleep "${TELEOP_DELAY_SEC}"

start_child "三相机 episode 录制" \
  ros2 launch qiling_recording_real real_recording.launch.py

echo "=== 三个节点均已启动；按 Ctrl-C 将一并停止它们 ==="

# Any child exiting is unexpected during an active session.  Exit through the
# trap so the other two children are not left behind.
set +e
wait -n "${CHILD_PIDS[@]}"
child_status=$?
set -e
echo "检测到一个子进程退出，状态码=${child_status}。"
exit "${child_status}"
