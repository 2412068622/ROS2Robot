#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

LOG_DIR="$(mktemp -d /tmp/robot_gazebo_sensor_stack.XXXXXX)"
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

wait_topic_type /clock "rosgraph_msgs/msg/Clock" 30 "${LOG_DIR}/wait_clock.err"
wait_topic_type /sim/scan "sensor_msgs/msg/LaserScan" 30 "${LOG_DIR}/wait_sim_scan.err"
wait_topic_type /scan "sensor_msgs/msg/LaserScan" 30 "${LOG_DIR}/wait_scan.err"
wait_topic_type /sim/imu "sensor_msgs/msg/Imu" 30 "${LOG_DIR}/wait_sim_imu.err"
wait_topic_type /imu/data "sensor_msgs/msg/Imu" 30 "${LOG_DIR}/wait_imu.err"
wait_topic_type /odom "nav_msgs/msg/Odometry" 30 "${LOG_DIR}/wait_odom.err"

wait_topic_sample /sim/scan sensor_msgs/msg/LaserScan 20 "${LOG_DIR}/sim_scan.txt"
wait_topic_sample /scan sensor_msgs/msg/LaserScan 20 "${LOG_DIR}/scan.txt"
wait_topic_sample /sim/imu sensor_msgs/msg/Imu 20 "${LOG_DIR}/sim_imu.txt"
wait_topic_sample /imu/data sensor_msgs/msg/Imu 20 "${LOG_DIR}/imu.txt"
wait_topic_sample /odom nav_msgs/msg/Odometry 20 "${LOG_DIR}/odom.txt"

timeout 8s ros2 run tf2_ros tf2_echo odom base_footprint \
  >"${LOG_DIR}/odom_base_tf.txt" 2>&1 || true
timeout 8s ros2 run tf2_ros tf2_echo base_footprint lidar_link \
  >"${LOG_DIR}/lidar_tf.txt" 2>&1 || true

rg "frame_id: lidar_link" "${LOG_DIR}/scan.txt"
rg "frame_id: imu_link" "${LOG_DIR}/imu.txt"
rg "child_frame_id: base_footprint" "${LOG_DIR}/odom.txt"
rg "At time" "${LOG_DIR}/odom_base_tf.txt"
rg "At time" "${LOG_DIR}/lidar_tf.txt"

echo "Gazebo sensor stack passed"
