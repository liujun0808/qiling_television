#!/usr/bin/env bash

set -Eeuo pipefail

# Start the three local RealSense RGB streams first, then start the rollout
# JPEG transport after a fixed warm-up delay. Keep this script in the
# foreground so Ctrl+C shuts down both launch processes together.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"

CAMERA_WARMUP_SEC=5
JPEG_OUTPUT_RATE_HZ="${ROLLOUT_IMAGE_RATE_HZ:-15.0}"
JPEG_QUALITY="${ROLLOUT_JPEG_QUALITY:-80}"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "错误：找不到 ROS 2 Humble 环境：${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -r "${WORKSPACE_SETUP}" ]]; then
  echo "错误：工作空间尚未构建或无法读取：${WORKSPACE_SETUP}" >&2
  echo "请先在 ${PROJECT_ROOT} 执行 colcon build。" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"

camera_launch_pid=""
compressor_launch_pid=""

cleanup() {
  trap - EXIT INT TERM HUP
  local pid
  for pid in "${compressor_launch_pid}" "${camera_launch_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${compressor_launch_pid}" "${camera_launch_pid}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
}

on_signal() {
  echo
  echo "收到停止信号，正在关闭压缩节点和三路相机……"
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM HUP

echo "项目目录：${PROJECT_ROOT}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "启动三路 RealSense：640x480 @ 30 Hz"
ros2 launch qiling_recording_real tri_camera.launch.py &
camera_launch_pid=$!

echo "等待相机初始化 ${CAMERA_WARMUP_SEC} 秒……"
sleep "${CAMERA_WARMUP_SEC}"
if ! kill -0 "${camera_launch_pid}" 2>/dev/null; then
  wait "${camera_launch_pid}" || true
  echo "错误：三路相机 launch 在等待期间已经退出，未启动压缩节点。" >&2
  exit 1
fi

echo "启动三路 rollout JPEG 传输：${JPEG_OUTPUT_RATE_HZ} Hz，quality=${JPEG_QUALITY}"
ros2 launch qiling_rollout_ros rollout_image_transport.launch.py \
  output_rate_hz:="${JPEG_OUTPUT_RATE_HZ}" \
  jpeg_quality:="${JPEG_QUALITY}" &
compressor_launch_pid=$!

# Give launch a brief chance to report an immediate configuration error.
sleep 1
if ! kill -0 "${compressor_launch_pid}" 2>/dev/null; then
  wait "${compressor_launch_pid}" || true
  echo "错误：rollout 图像压缩 launch 启动失败。" >&2
  exit 1
fi

echo "三路相机与压缩传输均已启动。按 Ctrl+C 可同时停止。"
echo "压缩话题："
echo "  /qiling_rollout/camera_head/image/compressed"
echo "  /qiling_rollout/camera_left/image/compressed"
echo "  /qiling_rollout/camera_right/image/compressed"

set +e
wait -n "${camera_launch_pid}" "${compressor_launch_pid}"
exit_status=$?
set -e
echo "某个 launch 进程已退出，正在关闭另一进程。" >&2
exit "${exit_status}"
