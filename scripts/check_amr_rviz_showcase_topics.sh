#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"
export GZ_PARTITION="${GZ_PARTITION:-robot_amr_rviz_showcase_${ROS_DOMAIN_ID}_$$}"

LOG_DIR="$(mktemp -d /tmp/robot_amr_rviz_showcase.XXXXXX)"
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

wait_path_has_pose() {
  local timeout_sec="${1:-60}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/odom_path.txt"
  while true; do
    if timeout 8s ros2 topic echo --once /odom/path nav_msgs/msg/Path \
        >"${sample}" 2>"${sample}.err" &&
      rg "position:" "${sample}" >/dev/null; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "odom path did not contain any pose" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${sample}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

start_bg amr_demo ros2 launch robot_bringup amr_demo.launch.py gui:=false use_rviz:=false

wait_topic_type /clock "rosgraph_msgs/msg/Clock" 30 "${LOG_DIR}/wait_clock.err"
wait_topic_type /map "nav_msgs/msg/OccupancyGrid" 60 "${LOG_DIR}/wait_map.err"
wait_topic_type /tf "tf2_msgs/msg/TFMessage" 30 "${LOG_DIR}/wait_tf.err"
wait_topic_type /scan "sensor_msgs/msg/LaserScan" 30 "${LOG_DIR}/wait_scan.err"
wait_topic_type /odom "nav_msgs/msg/Odometry" 30 "${LOG_DIR}/wait_odom.err"
wait_topic_type /odom/path "nav_msgs/msg/Path" 60 "${LOG_DIR}/wait_odom_path.err"
wait_topic_type /navigate_sequence/current_path "nav_msgs/msg/Path" 90 \
  "${LOG_DIR}/wait_nav_sequence_path.err"
wait_topic_type /navigate_sequence/current_goal "geometry_msgs/msg/PoseStamped" 90 \
  "${LOG_DIR}/wait_nav_sequence_goal.err"
wait_topic_type /initialpose "geometry_msgs/msg/PoseWithCovarianceStamped" 60 \
  "${LOG_DIR}/wait_initialpose.err"
wait_topic_type /amr_simulation/demo_timeline "std_msgs/msg/String" 60 \
  "${LOG_DIR}/wait_timeline.err"
wait_topic_type /amr_simulation/markers "visualization_msgs/msg/MarkerArray" 60 \
  "${LOG_DIR}/wait_markers.err"

wait_service_type /amr_demo_map_server/get_state "lifecycle_msgs/srv/GetState" 60 \
  "${LOG_DIR}/wait_map_server.err"
wait_lifecycle_active /amr_demo_map_server/get_state 60 "${LOG_DIR}/map_server_state.err"

wait_topic_sample /map nav_msgs/msg/OccupancyGrid 60 "${LOG_DIR}/map.txt"
rg "width: [1-9]" "${LOG_DIR}/map.txt"
rg "height: [1-9]" "${LOG_DIR}/map.txt"

wait_path_has_pose 90

wait_topic_sample /navigate_sequence/current_path nav_msgs/msg/Path 90 \
  "${LOG_DIR}/nav_sequence_path.txt"
rg "poses:|position:" "${LOG_DIR}/nav_sequence_path.txt"
wait_topic_sample /navigate_sequence/current_goal geometry_msgs/msg/PoseStamped 90 \
  "${LOG_DIR}/nav_sequence_goal.txt"
rg "position:" "${LOG_DIR}/nav_sequence_goal.txt"

wait_topic_sample /amr_simulation/demo_timeline std_msgs/msg/String 60 \
  "${LOG_DIR}/timeline.txt"
rg "当前阶段:|下一阶段:|进度:" "${LOG_DIR}/timeline.txt"

wait_topic_sample /amr_simulation/markers visualization_msgs/msg/MarkerArray 60 \
  "${LOG_DIR}/markers.txt"
rg "mission_panel|station_routes|safety|localization|当前阶段:|下一阶段:" "${LOG_DIR}/markers.txt"

echo "AMR RViz showcase topics passed"
