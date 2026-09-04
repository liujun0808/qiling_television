#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/ub/program/qiling_television"
CONFIG_PATH="${1:-${PROJECT_ROOT}/rollout/config/xvla_rollout.yaml}"
PYTHON_BIN="/home/ub/miniconda3/envs/lerobot051/bin/python"

export HF_HOME="${HF_HOME:-${PROJECT_ROOT}/training/cache/huggingface}"
exec "${PYTHON_BIN}" "${PROJECT_ROOT}/rollout/worker/xvla_worker.py" --config "${CONFIG_PATH}"
