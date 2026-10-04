#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv --system-site-packages .venv-vision
.venv-vision/bin/python -m pip install "torch==${AIC_TORCH_VERSION:-2.7.1}" "torchvision==${AIC_TORCHVISION_VERSION:-0.22.1}" --index-url "${AIC_TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"
.venv-vision/bin/python -m pip install 'ultralytics==8.4.164'
AIC_VISION_PYTHON="$PWD/.venv-vision/bin/python" bash tools/install_vision.sh
echo 'Created project-local CUDA environment. Run launch_full.sh inside the ROS container.'
