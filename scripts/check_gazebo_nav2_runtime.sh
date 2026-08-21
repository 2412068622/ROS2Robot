#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

LOG_DIR="$(mktemp -d /tmp/robot_gazebo_nav2_runtime.XXXXXX)"
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

read_odom_x() {
  local output_file="$1"
  timeout 8s ros2 topic echo --once /odom nav_msgs/msg/Odometry \
    >"${output_file}" 2>"${output_file}.err" || return 1
  awk '
    $1 == "position:" { in_position = 1; next }
    in_position && $1 == "x:" { print $2; exit }
  ' "${output_file}" | rg -m 1 '^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$'
}

wait_odom_x() {
  local label="$1"
  local timeout_sec="${2:-30}"
  local deadline=$((SECONDS + timeout_sec))
  local value
  while true; do
    value="$(read_odom_x "${LOG_DIR}/odom_${label}.txt" || true)"
    if [[ -n "${value}" ]]; then
      echo "${value}"
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "could not read odometry x for ${label}" >&2
      cat "${LOG_DIR}/odom_${label}.txt" >&2 2>/dev/null || true
      cat "${LOG_DIR}/odom_${label}.txt.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

call_service_checked() {
  local name="$1"
  shift
  local output_file="${LOG_DIR}/${name}.txt"
  if ! timeout 15s ros2 service call "$@" >"${output_file}" 2>&1; then
    echo "service call failed: ${name}" >&2
    cat "${output_file}" >&2
    return 1
  fi
}

start_bg sim ros2 launch robot_simulation sim.launch.py gui:=false
start_bg nav2 ros2 launch robot_navigation nav.launch.py \
  use_sim_time:=true navigation_start_delay:=5.0

wait_topic_type /clock "rosgraph_msgs/msg/Clock" 30 "${LOG_DIR}/wait_clock.err"
wait_topic_type /scan "sensor_msgs/msg/LaserScan" 30 "${LOG_DIR}/wait_scan.err"
wait_topic_type /odom "nav_msgs/msg/Odometry" 30 "${LOG_DIR}/wait_odom.err"
wait_topic_type /map "nav_msgs/msg/OccupancyGrid" 60 "${LOG_DIR}/wait_map.err"

for service in \
  /map_server/get_state \
  /amcl/get_state \
  /planner_server/get_state \
  /controller_server/get_state \
  /behavior_server/get_state \
  /bt_navigator/get_state; do
  node_name="$(basename "$(dirname "${service}")")"
  wait_service_type "${service}" "lifecycle_msgs/srv/GetState" \
    60 "${LOG_DIR}/${node_name}_service.err"
  wait_lifecycle_active "${service}" 60 "${LOG_DIR}/${node_name}_state.err"
done

wait_topic_type /nav2_cmd_vel "geometry_msgs/msg/Twist" \
  30 "${LOG_DIR}/wait_nav2_cmd_vel.err"
wait_action_type /navigate_to_pose "nav2_msgs/action/NavigateToPose" \
  30 "${LOG_DIR}/wait_navigate_to_pose.err"
wait_service_type /set_manual_takeover "std_srvs/srv/SetBool" \
  20 "${LOG_DIR}/wait_manual_takeover.err"
wait_service_type /v2/set_cmd_source "robot_interfaces_core/srv/SetControlMode" \
  20 "${LOG_DIR}/wait_cmd_source.err"

wait_topic_sample /scan sensor_msgs/msg/LaserScan 20 "${LOG_DIR}/scan.txt"
wait_topic_sample /odom nav_msgs/msg/Odometry 20 "${LOG_DIR}/odom.txt"
wait_topic_sample /map nav_msgs/msg/OccupancyGrid 20 "${LOG_DIR}/map.txt"

rg "frame_id: lidar_link" "${LOG_DIR}/scan.txt"
rg "child_frame_id: base_footprint" "${LOG_DIR}/odom.txt"
rg "width: [1-9]" "${LOG_DIR}/map.txt"
rg "height: [1-9]" "${LOG_DIR}/map.txt"

call_service_checked manual_takeover_call /set_manual_takeover \
  std_srvs/srv/SetBool "{data: false}"
call_service_checked cmd_source_call /v2/set_cmd_source \
  robot_interfaces_core/srv/SetControlMode "{mode: nav2}"

publish_initial_pose() {
  local attempt="$1"
  timeout 10s ros2 topic pub --once /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
    >"${LOG_DIR}/initialpose_${attempt}.txt" 2>&1
}

run_nav_goal_attempt() {
  local attempt="$1"
  local start_x
  local end_x
  local goal_log="${LOG_DIR}/nav2_goal_${attempt}.txt"
  start_x="$(wait_odom_x "start_${attempt}" 30)"
  if ! publish_initial_pose "${attempt}"; then
    echo "failed to publish initial pose for attempt ${attempt}" >&2
    cat "${LOG_DIR}/initialpose_${attempt}.txt" >&2
    return 1
  fi
  sleep 3
  if ! timeout 120s ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, behavior_tree: ''}" \
      >"${goal_log}" 2>&1; then
    echo "Nav2 goal command failed or timed out on attempt ${attempt}" >&2
    cat "${goal_log}" >&2
    return 1
  fi
  end_x="$(wait_odom_x "end_${attempt}" 30)"
  rg "Result:" "${goal_log}"
  if awk -v sx="${start_x}" -v ex="${end_x}" \
      'BEGIN {
         dx = ex - sx;
         exit !(dx > 0.45 && ex > 0.45)
       }'; then
    return 0
  fi
  echo "Nav2 goal attempt ${attempt} did not move Gazebo robot enough: start_x=${start_x} end_x=${end_x}" >&2
  cat "${goal_log}" >&2
  return 1
}

for attempt in 1 2 3; do
  if run_nav_goal_attempt "${attempt}"; then
    echo "Gazebo Nav2 runtime passed"
    exit 0
  fi
  sleep 4
done

echo "Nav2 goal did not move Gazebo robot enough after 3 attempts" >&2
for attempt in 1 2 3; do
  cat "${LOG_DIR}/nav2_goal_${attempt}.txt" >&2 2>/dev/null || true
done
  exit 1
