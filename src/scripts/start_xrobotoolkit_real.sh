#!/usr/bin/env bash

set -Eeuo pipefail

# XRoboToolkit runtime for the real-robot PC.
#
# This script intentionally does not launch a visible Qt window.  The PC
# service is started with Qt's offscreen platform, while the Unity demo is
# started in batch/no-graphics mode.  The Unity demo is kept because the
# Quest client documentation expects the 3D application to be running before
# the headset client connects.

SERVICE_ROOT="/opt/apps/roboticsservice"
DEMO_ROOT="${SERVICE_ROOT}/SDKDemo/RobotUnityDemo"
SERVICE_BIN="${SERVICE_ROOT}/RoboticsServiceProcess"
DEMO_BIN="${DEMO_ROOT}/RobotLinuxDemo.x86_64"

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/qiling_xrobotoolkit"
SERVICE_PID_FILE="${RUNTIME_DIR}/robotics_service.pid"
DEMO_PID_FILE="${RUNTIME_DIR}/robot_linux_demo.pid"
SERVICE_LOG="${RUNTIME_DIR}/robotics_service.log"
DEMO_LOG="${RUNTIME_DIR}/robot_linux_demo.log"

mkdir -p "${RUNTIME_DIR}"
umask 077

if [[ ! -x "${SERVICE_BIN}" ]]; then
  echo "错误：找不到或无法执行 ${SERVICE_BIN}" >&2
  exit 1
fi

if [[ ! -x "${DEMO_BIN}" ]]; then
  echo "错误：找不到或无法执行 ${DEMO_BIN}" >&2
  exit 1
fi

is_pid_running_for() {
  local pid_file="$1"
  local expected_name="$2"
  local pid

  [[ -r "${pid_file}" ]] || return 1
  read -r pid < "${pid_file}" || return 1
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -Fq "${expected_name}"
}

if is_pid_running_for "${SERVICE_PID_FILE}" "RoboticsServiceProcess"; then
  echo "XRoboToolkit PC Service 已在运行。"
else
  rm -f "${SERVICE_PID_FILE}"

  export LD_LIBRARY_PATH="${SERVICE_ROOT}:${SERVICE_ROOT}/lib:${SERVICE_ROOT}/SDK/x64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export QT_PLUGIN_PATH="${SERVICE_ROOT}/plugins/:${QT_PLUGIN_PATH:-}"
  export QT_QML_PATH="${SERVICE_ROOT}/qml/:${QT_QML_PATH:-}"
  export QT_QPA_PLATFORM="offscreen"
  unset DISPLAY WAYLAND_DISPLAY

  cd "${SERVICE_ROOT}"
  "${SERVICE_BIN}" >>"${SERVICE_LOG}" 2>&1 &
  service_pid=$!
  echo "${service_pid}" > "${SERVICE_PID_FILE}"
  sleep 1

  if ! is_pid_running_for "${SERVICE_PID_FILE}" "RoboticsServiceProcess"; then
    echo "错误：RoboticsServiceProcess 启动失败，请查看：${SERVICE_LOG}" >&2
    rm -f "${SERVICE_PID_FILE}"
    exit 1
  fi
  echo "RoboticsServiceProcess 已启动，PID=${service_pid}"
fi

if is_pid_running_for "${DEMO_PID_FILE}" "RobotLinuxDemo.x86_64"; then
  echo "RobotLinuxDemo.x86_64 已在运行。"
else
  rm -f "${DEMO_PID_FILE}"

  export LD_LIBRARY_PATH="${SERVICE_ROOT}:${SERVICE_ROOT}/lib:${SERVICE_ROOT}/SDK/x64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  unset DISPLAY WAYLAND_DISPLAY

  cd "${DEMO_ROOT}"
  "${DEMO_BIN}" \
    -batchmode \
    -nographics \
    -logFile "${DEMO_LOG}" \
    >>"${DEMO_LOG}" 2>&1 &
  demo_pid=$!
  echo "${demo_pid}" > "${DEMO_PID_FILE}"
  sleep 1

  if ! is_pid_running_for "${DEMO_PID_FILE}" "RobotLinuxDemo.x86_64"; then
    echo "错误：RobotLinuxDemo.x86_64 启动失败，请查看：${DEMO_LOG}" >&2
    rm -f "${DEMO_PID_FILE}"
    exit 1
  fi
  echo "RobotLinuxDemo.x86_64 已启动，PID=${demo_pid}"
fi

echo "XRoboToolkit 真机 PC 后台服务已启动。"
echo "服务日志：${SERVICE_LOG}"
echo "Demo 日志：${DEMO_LOG}"
