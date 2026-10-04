#!/usr/bin/env bash
# Source this helper after sourcing ROS. An explicit interpreter must be valid.
if [[ -n "${AIC_VISION_PYTHON:-}" ]]; then
  [[ -x "$AIC_VISION_PYTHON" ]] || { echo "AIC_VISION_PYTHON is not executable: $AIC_VISION_PYTHON" >&2; return 1; }
else
  for candidate_python in "$PWD/.venv-vision/bin/python" "${HOME}/.local/share/aic-vision-venv/bin/python"; do
    if [[ -x "$candidate_python" ]]; then AIC_VISION_PYTHON="$candidate_python"; break; fi
  done
  [[ -n "${AIC_VISION_PYTHON:-}" ]] || { echo 'Run bash tools/bootstrap_vision.sh or set AIC_VISION_PYTHON to a CUDA-enabled environment.' >&2; return 1; }
fi
export AIC_VISION_PYTHON
export PYTHONPATH="$PWD/.deps:${PYTHONPATH:-}"
