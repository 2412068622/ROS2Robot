#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

LOG_DIR="$(mktemp -d /tmp/robot_gazebo_slam_runtime.XXXXXX)"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  rm -rf "${LOG_DIR}"
}
trap cleanup EXIT

start_bg() {
  local name="$1"
  shift
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

start_bg sim ros2 launch robot_simulation sim.launch.py gui:=false
start_bg slam ros2 launch robot_navigation slam.launch.py use_sim_time:=true

wait_topic_type /clock "rosgraph_msgs/msg/Clock" 30 "${LOG_DIR}/wait_clock.err"
wait_topic_type /scan "sensor_msgs/msg/LaserScan" 30 "${LOG_DIR}/wait_scan.err"
wait_topic_type /odom "nav_msgs/msg/Odometry" 30 "${LOG_DIR}/wait_odom.err"
wait_lifecycle_active /slam_toolbox/get_state 60 "${LOG_DIR}/slam_state.txt"
wait_topic_type /map "nav_msgs/msg/OccupancyGrid" 60 "${LOG_DIR}/wait_map.err"
wait_topic_sample /map nav_msgs/msg/OccupancyGrid 60 "${LOG_DIR}/map.txt"

rg "width: [1-9]" "${LOG_DIR}/map.txt"
rg "height: [1-9]" "${LOG_DIR}/map.txt"

echo "Gazebo SLAM runtime passed"
