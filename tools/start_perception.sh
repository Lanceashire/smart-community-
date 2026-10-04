#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
source tools/vision_env.sh
source tools/ros_transport.sh
exec "$AIC_VISION_PYTHON" -m community_demo.perception "$@"
