#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source tools/vision_env.sh
vision_python="$AIC_VISION_PYTHON"
if ! "$vision_python" -c 'import torch,ultralytics; assert torch.cuda.is_available()'; then
  echo 'Provide the GPU virtual environment with AIC_VISION_PYTHON; see README.' >&2
  exit 1
fi
"$vision_python" -m pip install --target .deps --no-deps -r requirements-ocr.txt
