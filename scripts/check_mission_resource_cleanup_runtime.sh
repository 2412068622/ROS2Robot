#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_mission_resource_cleanup.XXXXXX)"
PIDS=()

cleanup() {
  local status=$?
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  if [[ ${status} -eq 0 ]]; then
    rm -rf "${LOG_DIR}"
  else
    echo "preserving logs in ${LOG_DIR}" >&2
  fi
  return "${status}"
}
trap cleanup EXIT

start_bg() {
  local name="$1"
  shift
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

wait_finished_event() {
  local mission_id="$1"
  local event_file="${LOG_DIR}/${mission_id}_finished_event.out"
  local deadline=$((SECONDS + 45))
  while true; do
    ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 80, state_filter: FINISHED, mission_id_filter: ${mission_id}}" \
      >"${event_file}" 2>"${event_file}.err" || true
    if rg "success=True" "${event_file}" >/dev/null &&
        rg "loaded [1-9][0-9]* mission event" "${event_file}" >/dev/null &&
        rg "mission_ids=.*${mission_id}" "${event_file}" >/dev/null &&
        rg "states=.*FINISHED" "${event_file}" >/dev/null; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "mission ${mission_id} did not publish FINISHED event" >&2
      cat "${event_file}" >&2 || true
      return 1
    fi
    sleep 1
  done
}

assert_no_route_owner() {
  local mission_id="$1"
  local traffic_file="${LOG_DIR}/${mission_id}_traffic_after_finish.out"
  ros2 service call /v2/list_traffic_reservations robot_interfaces_mission/srv/ListTrafficReservations "{}" \
    >"${traffic_file}" 2>"${traffic_file}.err"
  rg "success=True" "${traffic_file}"
  if rg "${mission_id}|route_edge:receiving__storage_a|route_node:storage_a" "${traffic_file}" \
      >/dev/null; then
    echo "route locks were not released for ${mission_id}" >&2
    cat "${traffic_file}" >&2
    return 1
  fi
}

start_bg navigate_sequence_server ros2 run robot_tasks navigate_sequence_server_node --ros-args \
  -p use_nav2_action:=false \
  -p simulate_without_nav2:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  server_timeout_ms:=3000 \
  return_to_dock_on_low_battery:=false \
  traffic_intersection_locks_enabled:=true

wait_service_type /v2/submit_order \
  "robot_interfaces_business/srv/SubmitOrder" 20 "${LOG_DIR}/wait_submit.err"
wait_service_type /start_mission_profile \
  "std_srvs/srv/Trigger" 20 "${LOG_DIR}/wait_start.err"
wait_service_type /v2/execute_facility_action \
  "robot_interfaces_facility/srv/ExecuteFacilityAction" 20 "${LOG_DIR}/wait_facility.err"
wait_service_type /v2/list_traffic_reservations \
  "robot_interfaces_mission/srv/ListTrafficReservations" 20 "${LOG_DIR}/wait_traffic.err"
wait_service_type /v2/list_facility_resources \
  "robot_interfaces_facility/srv/ListFacilityResources" 20 "${LOG_DIR}/wait_facility_list.err"
wait_service_type /v2/list_mission_events \
  "robot_interfaces_mission/srv/ListMissionEvents" 20 "${LOG_DIR}/wait_events.err"

ros2 service call /v2/submit_order robot_interfaces_business/srv/SubmitOrder \
  "{order_id: cleanup_route_alpha, order_type: station_transport, priority: 35, payload_json: 'pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false', tags: [runtime_smoke, resource_cleanup]}" \
  >"${LOG_DIR}/submit_route.out"
rg "accepted=True" "${LOG_DIR}/submit_route.out"
rg "station_order_cleanup_route_alpha" "${LOG_DIR}/submit_route.out"
rg "queue_size=1" "${LOG_DIR}/submit_route.out"

ros2 service call /v2/list_traffic_reservations robot_interfaces_mission/srv/ListTrafficReservations "{}" \
  >"${LOG_DIR}/traffic_before_route_start.out"
rg "success=True" "${LOG_DIR}/traffic_before_route_start.out"
rg "station_order_cleanup_route_alpha" "${LOG_DIR}/traffic_before_route_start.out"
rg "route_edge:receiving__storage_a" "${LOG_DIR}/traffic_before_route_start.out"

ros2 service call /start_mission_profile std_srvs/srv/Trigger "{}" \
  >"${LOG_DIR}/start_route.out"
rg "success=True" "${LOG_DIR}/start_route.out"

wait_finished_event "station_order_cleanup_route_alpha"
assert_no_route_owner "station_order_cleanup_route_alpha"

ros2 service call /v2/execute_facility_action robot_interfaces_facility/srv/ExecuteFacilityAction \
  "{request_id: cleanup_elevator_alpha, resource_id: freight_elevator, resource_type: elevator, action: call, priority: 34, start_if_idle: true, preempt_current: false, hold_after_action: false}" \
  >"${LOG_DIR}/execute_elevator.out"
rg "success=True" "${LOG_DIR}/execute_elevator.out"
rg "facility_action_cleanup_elevator_alpha" "${LOG_DIR}/execute_elevator.out"

wait_finished_event "facility_action_cleanup_elevator_alpha"

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: elevator, include_disabled: false}" \
  >"${LOG_DIR}/facility_after_elevator.out"
rg "success=True" "${LOG_DIR}/facility_after_elevator.out"
rg "resource_ids=\\['freight_elevator'\\]" "${LOG_DIR}/facility_after_elevator.out"
rg "available=\\[True\\]" "${LOG_DIR}/facility_after_elevator.out"
if rg "facility_action_cleanup_elevator_alpha" "${LOG_DIR}/facility_after_elevator.out" >/dev/null; then
  echo "facility resource holder was not cleared after completion" >&2
  cat "${LOG_DIR}/facility_after_elevator.out" >&2
  exit 1
fi

echo "mission resource cleanup runtime passed"
