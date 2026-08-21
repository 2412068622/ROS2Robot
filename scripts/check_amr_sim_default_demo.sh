#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

LOG_DIR="$(mktemp -d /tmp/robot_amr_sim_default_demo.XXXXXX)"
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

wait_marker_text() {
  local pattern="$1"
  local timeout_sec="${2:-30}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/markers.txt"
  while true; do
    if timeout 10s ros2 topic echo --once /amr_simulation/markers \
        visualization_msgs/msg/MarkerArray >"${sample}" 2>"${LOG_DIR}/markers.err" &&
      rg "${pattern}" "${sample}" >/dev/null; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "marker text did not match pattern: ${pattern}" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${LOG_DIR}/markers.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

wait_station_finished() {
  wait_mission_finished "station_order_sim_auto_transport" "${1:-180}" "station transport did not finish in default demo"
}

wait_mission_finished() {
  local mission_id="$1"
  local timeout_sec="${2:-180}"
  local message="${3:-mission did not finish}"
  wait_mission_state "${mission_id}" "FINISHED" "${timeout_sec}" "${message}"
}

wait_mission_state() {
  local mission_id="$1"
  local state="$2"
  local timeout_sec="${3:-180}"
  local message="${4:-mission did not reach state}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/station_events.txt"
  while true; do
    if timeout 6s ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
        "{limit: 10, state_filter: ${state}, mission_id_filter: ${mission_id}}" \
        >"${sample}" 2>"${LOG_DIR}/station_events.err" &&
      rg "mission_ids=\\['${mission_id}'\\]" "${sample}" >/dev/null &&
      rg "states=\\['${state}'\\]" "${sample}" >/dev/null; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "${message}" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${LOG_DIR}/station_events.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 2
  done
}

read_odom_field() {
  local field="$1"
  timeout 5s ros2 topic echo --once /model/mobile_robot/odometry \
    nav_msgs/msg/Odometry --field "${field}" 2>"${LOG_DIR}/odom_${field//./_}.err" \
    | rg -m 1 '^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$'
}

read_odom_position_xy() {
  local sample="$1"
  if ! timeout 10s ros2 topic echo --once /model/mobile_robot/odometry \
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

wait_startup_drive_moved() {
  local timeout_sec="${1:-60}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/startup_drive_odom.txt"
  local position=""
  local start_x=""
  local start_y=""
  while [[ -z "${start_x}" || -z "${start_y}" ]]; do
    position="$(read_odom_position_xy "${sample}" || true)"
    if [[ -n "${position}" ]]; then
      read -r start_x start_y <<<"${position}"
      break
    fi
    if ((SECONDS >= deadline)); then
      echo "startup blue robot drive could not read initial odometry" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${sample}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done

  while true; do
    position="$(read_odom_position_xy "${sample}" || true)"
    if [[ -n "${position}" ]]; then
      local x=""
      local y=""
      read -r x y <<<"${position}"
      if awk -v sx="${start_x}" -v sy="${start_y}" -v x="${x}" -v y="${y}" \
          'BEGIN { dx = x - sx; dy = y - sy; exit !(sqrt(dx * dx + dy * dy) > 0.8) }'; then
        return 0
      fi
    fi
    if ((SECONDS >= deadline)); then
      echo "startup blue robot drive did not move far enough from start: start=(${start_x}, ${start_y}) latest=${position:-unavailable}" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${sample}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

wait_odom_near_storage_b() {
  local timeout_sec="${1:-60}"
  local deadline=$((SECONDS + timeout_sec))
  local x=""
  local y=""
  local sample="${LOG_DIR}/startup_odom_position.txt"
  while true; do
    if timeout 5s ros2 topic echo --once /model/mobile_robot/odometry \
        nav_msgs/msg/Odometry --field pose.pose.position >"${sample}" \
        2>"${LOG_DIR}/startup_odom_position.err"; then
      x="$(awk '/^x:/ {print $2; exit}' "${sample}")"
      y="$(awk '/^y:/ {print $2; exit}' "${sample}")"
    fi
    if [[ -n "${x}" && -n "${y}" ]] &&
      awk -v x="${x}" -v y="${y}" 'BEGIN { exit !(x < -1.2 && y > 1.2) }'; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "startup blue robot drive did not move Gazebo robot near storage_b: x=${x:-unavailable}, y=${y:-unavailable}" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${LOG_DIR}/startup_odom_position.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

wait_route_resource_absent() {
  local resource_id="$1"
  local timeout_sec="${2:-120}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/traffic_reservations.txt"
  while true; do
    if timeout 6s ros2 service call /v2/list_traffic_reservations \
        robot_interfaces_mission/srv/ListTrafficReservations "{}" \
        >"${sample}" 2>"${LOG_DIR}/traffic_reservations.err" &&
      rg "success=True" "${sample}" >/dev/null &&
      ! rg "${resource_id}" "${sample}" >/dev/null; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "traffic resource still present: ${resource_id}" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${LOG_DIR}/traffic_reservations.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 2
  done
}

wait_safety_state_speed_restored() {
  local timeout_sec="${1:-120}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/safety_state.txt"
  while true; do
    if timeout 5s ros2 topic echo --once /safety_state \
        robot_interfaces/msg/SafetyState >"${sample}" 2>"${LOG_DIR}/safety_state.err" &&
      rg "state: OK" "${sample}" >/dev/null &&
      rg "runtime_speed_limit_mps: 0" "${sample}" >/dev/null; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "dynamic speed limit did not clear in default demo" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${LOG_DIR}/safety_state.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 2
  done
}

start_bg amr_demo ros2 launch robot_bringup amr_demo.launch.py gui:=false use_rviz:=false

wait_topic_type /amr_simulation/markers "visualization_msgs/msg/MarkerArray" 30 \
  "${LOG_DIR}/wait_markers.err"
wait_topic_type /safety_state "robot_interfaces/msg/SafetyState" 30 \
  "${LOG_DIR}/wait_safety_state.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 30 \
  "${LOG_DIR}/wait_events.err"
wait_service_type /v2/list_traffic_reservations "robot_interfaces_mission/srv/ListTrafficReservations" 30 \
  "${LOG_DIR}/wait_traffic.err"

wait_startup_drive_moved 60

wait_marker_text "payload on robot: sim_auto_tote" 150
wait_marker_text "confirm: load_tote" 150
wait_marker_text "elevator: lift_session" 150
wait_marker_text "speed limit: 0.45 mps" 150
wait_mission_state "docking_request_sim_auto" "CHARGING" 180 \
  "docking did not reach CHARGING in default demo"
wait_odom_near_storage_b 240
wait_station_finished 60

wait_mission_state "payload_sim_auto_tote" "PAYLOAD_UNLOADED" 90 \
  "payload unload did not finish in default demo"
wait_mission_finished "charging_request_sim_auto_opportunity" 240 \
  "opportunity charging did not finish in default demo"
wait_marker_text "blocked: sim_auto_obstacle" 90
wait_mission_state "safety_sim_auto_obstacle" "OBSTACLE_CLEARED" 120 \
  "obstacle clear did not finish in default demo"
wait_route_resource_absent "route_edge:receiving__storage_a" 120
wait_safety_state_speed_restored 120

echo "amr simulation default demo passed"
