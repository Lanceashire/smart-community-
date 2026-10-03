#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
mkdir -p log
bash tools/start_perception.sh --ros-args -p use_sim_time:=true \
  -p model:="${AIC_PERCEPTION_MODEL:-src/community_demo/models/perception/yolo11s-comparison.pt}" > log/perception-terminal.log 2>&1 &
perception_pid=$!
cleanup() { kill -INT "$perception_pid" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
ros2 launch community_demo demo.launch.py vision:=false "$@"
