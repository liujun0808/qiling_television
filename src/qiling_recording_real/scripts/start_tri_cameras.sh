#!/usr/bin/env bash
# Start head D435i + left/right D405 RGB only at the profile enforced by the launch file.
set -euo pipefail

CONFIG_FILE="${1:-}"
if [[ -n "${CONFIG_FILE}" ]]; then
  exec ros2 launch qiling_recording_real tri_camera.launch.py "config_file:=${CONFIG_FILE}"
fi
exec ros2 launch qiling_recording_real tri_camera.launch.py
