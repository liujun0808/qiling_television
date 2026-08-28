#!/usr/bin/env bash

set -Eeuo pipefail

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/qiling_xrobotoolkit"
SERVICE_PID_FILE="${RUNTIME_DIR}/robotics_service.pid"
DEMO_PID_FILE="${RUNTIME_DIR}/robot_linux_demo.pid"

stop_from_pid_file() {
  local label="$1"
  local pid_file="$2"
  local expected_name="$3"
  local pid
  local waited

  if [[ ! -r "${pid_file}" ]]; then
    echo "${label}：没有 PID 文件，跳过。"
    return 0
  fi

  read -r pid < "${pid_file}" || pid=""
  if [[ ! "${pid}" =~ ^[0-9]+$ ]] || ! kill -0 "${pid}" 2>/dev/null; then
    echo "${label}：进程已经退出。"
    rm -f "${pid_file}"
    return 0
  fi

  if [[ ! -r "/proc/${pid}/cmdline" ]] || \
     ! tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -Fq "${expected_name}"; then
    echo "警告：PID ${pid} 不匹配 ${expected_name}，为安全起见不终止。" >&2
    return 1
  fi

  echo "停止 ${label}，PID=${pid}"
  kill -TERM "${pid}" 2>/dev/null || true

  waited=0
  while kill -0 "${pid}" 2>/dev/null && (( waited < 50 )); do
    sleep 0.1
    ((waited += 1))
  done

  if kill -0 "${pid}" 2>/dev/null; then
    echo "${label} 未在 5 秒内退出，发送 SIGKILL。" >&2
    kill -KILL "${pid}" 2>/dev/null || true
  fi

  rm -f "${pid_file}"
}

echo "=== 停止 XRoboToolkit 真机 PC 后台服务 ==="

# Stop the Unity client-side process first, then stop the PC service.
stop_from_pid_file "RobotLinuxDemo.x86_64" "${DEMO_PID_FILE}" "RobotLinuxDemo.x86_64"
stop_from_pid_file "RoboticsServiceProcess" "${SERVICE_PID_FILE}" "RoboticsServiceProcess"

echo "=== XRoboToolkit 真机 PC 后台服务已停止 ==="
