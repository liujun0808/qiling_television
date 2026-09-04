#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec /home/ub/miniconda3/envs/lerobot051/bin/python "$root/training/scripts/run_xvla_train.py" --config "$root/training/configs/xvla_full_finetune.yaml" "$@"
