#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"
export GZ_PARTITION="${GZ_PARTITION:-robot_amr_robot2_namespace_${ROS_DOMAIN_ID}_$$}"

LOG_DIR="$(mktemp -d /tmp/robot_amr_robot2_namespace.XXXXXX)"
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

read_robot2_position_xy() {
  local sample="$1"
  if ! timeout 10s ros2 topic echo --once /robot_2/odom \
      nav_msgs/msg/Odometry --field pose.pose.position >"${sample}" \
      2>"${sample}.err"; then
    return 1
  fi
  local x=""
  local y=""
  x="$(awk '/^x:/ {print $2; exit}' "${sample}")"
  y="$(awk '/^y:/ {print $2; exit}' "${sample}")"
  [[ -n "${x}" && -n "${y}" ]] || return 1
  printf '%s %s\n' "${x}" "${y}"
}

model_pose_xyz() {
  local model="$1"
  timeout 8s gz model -m "${model}" -p 2>"${LOG_DIR}/${model}.err" \
    | awk '/Pose \[ XYZ/ {getline; gsub(/[][]/, "", $0); print $1, $2, $3; exit}'
}

wait_model_pose() {
  local model="$1"
  local condition="$2"
  local timeout_sec="${3:-80}"
  local deadline=$((SECONDS + timeout_sec))
  local pose=""
  while true; do
    pose="$(model_pose_xyz "${model}" || true)"
    if [[ -n "${pose}" ]] &&
      awk -v x="$(awk '{print $1}' <<<"${pose}")" \
          -v y="$(awk '{print $2}' <<<"${pose}")" \
          -v z="$(awk '{print $3}' <<<"${pose}")" \
          "BEGIN { exit !(${condition}) }"; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "model ${model} did not satisfy '${condition}', latest=${pose:-unavailable}" >&2
      cat "${LOG_DIR}/${model}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

wait_robot2_odom_displacement() {
  local start_x="$1"
  local start_y="$2"
  local min_distance="$3"
  local timeout_sec="${4:-80}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/robot2_odom_position.txt"
  local position=""
  while true; do
    position="$(read_robot2_position_xy "${sample}" || true)"
    if [[ -n "${position}" ]] &&
      awk -v x="$(awk '{print $1}' <<<"${position}")" \
          -v y="$(awk '{print $2}' <<<"${position}")" \
          -v sx="${start_x}" \
          -v sy="${start_y}" \
          -v min_distance="${min_distance}" \
          'BEGIN { dx = x - sx; dy = y - sy; exit !(sqrt(dx * dx + dy * dy) >= min_distance) }'; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "robot_2 odom displacement did not reach ${min_distance} m, start=(${start_x}, ${start_y}) latest=${position:-unavailable}" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${sample}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

start_bg amr_demo ros2 launch robot_bringup amr_demo.launch.py \
  gui:=false use_rviz:=false auto_demo:=false

wait_service_type /v2/list_stations "robot_interfaces_mission/srv/ListStations" 30 \
  "${LOG_DIR}/wait_stations.err"
wait_service_type /v2/list_fleet_robots "robot_interfaces_fleet/srv/ListFleetRobots" 30 \
  "${LOG_DIR}/wait_fleet.err"
wait_service_type /v2/update_fleet_robot_state "robot_interfaces_fleet/srv/UpdateFleetRobotState" 30 \
  "${LOG_DIR}/wait_update_fleet.err"

wait_topic_type /robot_2/cmd_vel "geometry_msgs/msg/Twist" 30 \
  "${LOG_DIR}/wait_robot2_cmd_vel.err"
wait_topic_type /model/mobile_robot_2/odometry "nav_msgs/msg/Odometry" 30 \
  "${LOG_DIR}/wait_robot2_raw_odom.err"
wait_topic_type /robot_2/odom "nav_msgs/msg/Odometry" 30 \
  "${LOG_DIR}/wait_robot2_odom.err"
wait_topic_type /robot_2/sim/scan "sensor_msgs/msg/LaserScan" 30 \
  "${LOG_DIR}/wait_robot2_raw_scan.err"
wait_topic_type /robot_2/scan "sensor_msgs/msg/LaserScan" 30 \
  "${LOG_DIR}/wait_robot2_scan.err"
wait_topic_type /robot_2/robot_description "std_msgs/msg/String" 30 \
  "${LOG_DIR}/wait_robot2_description.err"

wait_topic_sample /robot_2/odom nav_msgs/msg/Odometry 20 "${LOG_DIR}/robot2_odom.txt"
wait_topic_sample /robot_2/scan sensor_msgs/msg/LaserScan 20 "${LOG_DIR}/robot2_scan.txt"
rg "frame_id: robot_2/lidar_link" "${LOG_DIR}/robot2_scan.txt"
initial_position="$(read_robot2_position_xy "${LOG_DIR}/robot2_initial_odom.txt")"
read -r initial_x initial_y <<<"${initial_position}"

timeout 10s ros2 run tf2_ros tf2_echo robot_2/odom robot_2/base_footprint \
  >"${LOG_DIR}/robot2_odom_base_tf.txt" 2>&1 || true
timeout 10s ros2 run tf2_ros tf2_echo robot_2/base_footprint robot_2/lidar_link \
  >"${LOG_DIR}/robot2_lidar_tf.txt" 2>&1 || true
rg "At time" "${LOG_DIR}/robot2_odom_base_tf.txt"
rg "At time" "${LOG_DIR}/robot2_lidar_tf.txt"

ros2 service call /v2/update_fleet_robot_state robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_2, enabled: true, state: MOVING, battery_voltage: 24.0, current_station_id: packing, update_enabled: false, update_state: true, update_battery_voltage: false, update_current_station_id: true}" \
  >"${LOG_DIR}/update_robot2_packing.txt" 2>&1
rg "success=True|success: true" "${LOG_DIR}/update_robot2_packing.txt"

wait_robot2_odom_displacement "${initial_x}" "${initial_y}" 1.0 90
wait_model_pose "mobile_robot_2" "x > 1.0 && y < -1.0 && z > 0.05" 90

echo "AMR robot_2 namespace simulation passed"
