#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
export PYTHONPATH="$PWD/.deps:$PYTHONPATH"
exec "${AIC_VISION_PYTHON:-/home/ubuntu/.local/share/aic-vision-venv/bin/python}" -m community_demo.perception "$@"
