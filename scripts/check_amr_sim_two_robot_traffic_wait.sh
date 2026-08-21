#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"
export GZ_PARTITION="${GZ_PARTITION:-robot_two_robot_traffic_${ROS_DOMAIN_ID}_$$}"

LOG_DIR="$(mktemp -d /tmp/robot_two_robot_traffic.XXXXXX)"
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

call_service() {
  local name="$1"
  local service="$2"
  local type="$3"
  local request="$4"
  local attempt
  for attempt in 1 2 3; do
    if timeout -k 5s 35s ros2 service call "${service}" "${type}" "${request}" \
        >"${LOG_DIR}/${name}.out" 2>"${LOG_DIR}/${name}.err"; then
      return 0
    fi
    echo "${service} call attempt ${attempt} failed; retrying" >&2
    sleep 2
  done
  echo "${service} call failed after retries" >&2
  cat "${LOG_DIR}/${name}.out" >&2 2>/dev/null || true
  cat "${LOG_DIR}/${name}.err" >&2 2>/dev/null || true
  return 1
}

model_pose_xyz() {
  local model="$1"
  timeout 8s gz model -m "${model}" -p 2>"${LOG_DIR}/${model}.err" \
    | awk '/Pose \[ XYZ/ {getline; gsub(/[][]/, "", $0); print $1, $2, $3; exit}'
}

wait_model_pose() {
  local model="$1"
  local condition="$2"
  local timeout_sec="${3:-60}"
  local deadline=$((SECONDS + timeout_sec))
  local pose
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
      echo "model ${model} did not satisfy '${condition}', last pose: ${pose}" >&2
      cat "${LOG_DIR}/${model}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

assert_no_owner() {
  local owner="$1"
  local file="$2"
  if rg "${owner}" "${file}" >/dev/null; then
    echo "unexpected traffic owner still present: ${owner}" >&2
    cat "${file}" >&2
    return 1
  fi
}

start_bg amr_demo ros2 launch robot_bringup amr_demo.launch.py \
  gui:=false \
  use_rviz:=false \
  auto_demo:=false

wait_submit_order 40 "${LOG_DIR}/wait_submit.err"
wait_service_type /v2/list_traffic_reservations \
  "robot_interfaces_mission/srv/ListTrafficReservations" 40 \
  "${LOG_DIR}/wait_list_traffic.err"
wait_service_type /v2/clear_traffic_reservation \
  "robot_interfaces_mission/srv/ClearTrafficReservation" 40 \
  "${LOG_DIR}/wait_clear_traffic.err"
wait_service_type /v2/cancel_queued_mission \
  "robot_interfaces_mission/srv/CancelQueuedMission" 40 \
  "${LOG_DIR}/wait_cancel.err"
wait_service_type /v2/update_fleet_robot_state \
  "robot_interfaces_fleet/srv/UpdateFleetRobotState" 40 \
  "${LOG_DIR}/wait_fleet_update.err"
wait_service_type /v2/list_stations "robot_interfaces_mission/srv/ListStations" 40 \
  "${LOG_DIR}/wait_stations.err"
wait_topic_type /robot_2/odom "nav_msgs/msg/Odometry" 40 \
  "${LOG_DIR}/wait_robot2_odom.err"
wait_service_type /world/indoor_room/set_pose "ros_gz_interfaces/srv/SetEntityPose" 40 \
  "${LOG_DIR}/wait_set_pose.err"

call_service move_main_robot /world/indoor_room/set_pose \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: mobile_robot, type: 2}, pose: {position: {x: -4.2, y: 3.8, z: 0.24}, orientation: {w: 1.0}}}"
rg "success=True" "${LOG_DIR}/move_main_robot.out"
wait_model_pose "mobile_robot" "x < -3.5 && y > 3.0" 40

call_submit_order traffic_alpha station_transport 28 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false" \
  station_transport_order >"${LOG_DIR}/alpha.out" 2>"${LOG_DIR}/alpha.err"
rg "accepted=True" "${LOG_DIR}/alpha.out"
rg "station_order_traffic_alpha" "${LOG_DIR}/alpha.out"

call_service traffic_alpha /v2/list_traffic_reservations \
  robot_interfaces_mission/srv/ListTrafficReservations "{}"
rg "success=True" "${LOG_DIR}/traffic_alpha.out"
rg "route_edge:receiving__storage_a" "${LOG_DIR}/traffic_alpha.out"
rg "route_node:storage_a" "${LOG_DIR}/traffic_alpha.out"
rg "station_order_traffic_alpha" "${LOG_DIR}/traffic_alpha.out"
rg "mission" "${LOG_DIR}/traffic_alpha.out"

call_service robot2_wait /v2/update_fleet_robot_state \
  robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_2, enabled: true, state: WAITING_TRAFFIC, battery_voltage: 24.0, current_station_id: traffic_wait, update_enabled: false, update_state: true, update_battery_voltage: true, update_current_station_id: true}"
rg "success=True" "${LOG_DIR}/robot2_wait.out"
wait_model_pose "mobile_robot_2" "x > -1.6 && x < -0.4 && y < -3.6" 120
wait_model_pose "traffic_yield_wait_marker" "x > -1.6 && x < -0.4 && y < -3.6 && z > 0.1" 40

call_submit_order traffic_beta station_transport 27 \
  "pickup_station=storage_a;dropoff_station=packing;start_if_idle=false;preempt_current=false" \
  station_transport_order >"${LOG_DIR}/beta_blocked.out" 2>"${LOG_DIR}/beta_blocked.err"
rg "accepted=False" "${LOG_DIR}/beta_blocked.out"
rg "route resource locked: route_node:storage_a by station_order_traffic_alpha" \
  "${LOG_DIR}/beta_blocked.out"

call_service clear_alpha /v2/clear_traffic_reservation \
  robot_interfaces_mission/srv/ClearTrafficReservation \
  "{resource_id: route_edge:receiving__storage_a}"
rg "success=True" "${LOG_DIR}/clear_alpha.out"
rg "station_order_traffic_alpha" "${LOG_DIR}/clear_alpha.out"

call_service cancel_alpha /v2/cancel_queued_mission \
  robot_interfaces_mission/srv/CancelQueuedMission \
  "{mission_id: station_order_traffic_alpha, cancel_active: false}"
rg "success=True" "${LOG_DIR}/cancel_alpha.out"

call_service traffic_after_alpha /v2/list_traffic_reservations \
  robot_interfaces_mission/srv/ListTrafficReservations "{}"
assert_no_owner "station_order_traffic_alpha" "${LOG_DIR}/traffic_after_alpha.out"

call_submit_order traffic_beta_after_release station_transport 29 \
  "pickup_station=storage_a;dropoff_station=packing;start_if_idle=false;preempt_current=false" \
  station_transport_order >"${LOG_DIR}/beta_after_release.out" 2>"${LOG_DIR}/beta_after_release.err"
rg "accepted=True" "${LOG_DIR}/beta_after_release.out"
rg "station_order_traffic_beta_after_release" "${LOG_DIR}/beta_after_release.out"

call_service traffic_beta /v2/list_traffic_reservations \
  robot_interfaces_mission/srv/ListTrafficReservations "{}"
rg "route_edge:packing__storage_a" "${LOG_DIR}/traffic_beta.out"
rg "route_node:storage_a" "${LOG_DIR}/traffic_beta.out"
rg "station_order_traffic_beta_after_release" "${LOG_DIR}/traffic_beta.out"

call_service robot2_go /v2/update_fleet_robot_state \
  robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_2, enabled: true, state: MOVING, battery_voltage: 24.0, current_station_id: packing, update_enabled: false, update_state: true, update_battery_voltage: true, update_current_station_id: true}"
rg "success=True" "${LOG_DIR}/robot2_go.out"
wait_model_pose "traffic_release_go_marker" "x > 1.2 && x < 2.2 && y > -0.5 && y < 1.0 && z > 0.2" 40
wait_model_pose "mobile_robot_2" "x > 1.0 && y < -0.5" 120

call_service clear_beta /v2/clear_traffic_reservation \
  robot_interfaces_mission/srv/ClearTrafficReservation \
  "{resource_id: route_edge:packing__storage_a}"
rg "success=True" "${LOG_DIR}/clear_beta.out"
rg "station_order_traffic_beta_after_release" "${LOG_DIR}/clear_beta.out"

call_service cancel_beta /v2/cancel_queued_mission \
  robot_interfaces_mission/srv/CancelQueuedMission \
  "{mission_id: station_order_traffic_beta_after_release, cancel_active: false}"
rg "success=True" "${LOG_DIR}/cancel_beta.out"

call_service traffic_final /v2/list_traffic_reservations \
  robot_interfaces_mission/srv/ListTrafficReservations "{}"
assert_no_owner "station_order_traffic_alpha" "${LOG_DIR}/traffic_final.out"
assert_no_owner "station_order_traffic_beta_after_release" "${LOG_DIR}/traffic_final.out"

echo "two robot traffic wait passed"
