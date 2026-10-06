#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
mkdir -p log
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
source tools/vision_env.sh
source tools/ros_transport.sh
"$AIC_VISION_PYTHON" tools/preflight.py --model "${AIC_PERCEPTION_MODEL:-src/community_demo/models/perception/yolo11s-comparison.pt}"
sensor_remaps=()
for launch_argument in "$@"; do
  if [[ "$launch_argument" == 'stress:=true' ]]; then sensor_remaps=(-r /camera/image:=/stress/camera/image); fi
done
env --default-signal=INT,QUIT bash tools/start_perception.sh --ros-args "${sensor_remaps[@]}" -p use_sim_time:=true \
  -p model:="${AIC_PERCEPTION_MODEL:-src/community_demo/models/perception/yolo11s-comparison.pt}" > log/perception-terminal.log 2>&1 &
perception_pid=$!
ros_pid=''
cleanup_done=false
cleanup() {
  $cleanup_done && return
  cleanup_done=true
  kill -INT "$perception_pid" ${ros_pid:+"$ros_pid"} 2>/dev/null || true
  # A deadlocked recognition process must not block shutdown forever.
  for _ in {1..30}; do
    kill -0 "$perception_pid" 2>/dev/null || return
    sleep .1
  done
  kill -TERM "$perception_pid" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "$perception_pid" 2>/dev/null || return
    sleep .1
  done
  kill -KILL "$perception_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
env --default-signal=INT,QUIT ros2 launch community_demo demo.launch.py vision:=false "$@" &
ros_pid=$!
set +e
wait -n "$perception_pid" "$ros_pid"
launch_status=$?
cleanup
wait "$perception_pid" "$ros_pid" 2>/dev/null
exit "$launch_status"
