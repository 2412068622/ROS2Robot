#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_multi_floor_sequence.XXXXXX)"
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

wait_mission_finished() {
  local mission_id="$1"
  local deadline=$((SECONDS + 60))
  while true; do
    ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 30, state_filter: FINISHED, mission_id_filter: ${mission_id}}" \
      >"${LOG_DIR}/${mission_id}.event" 2>"${LOG_DIR}/${mission_id}.err" || true
    if rg "success=True" "${LOG_DIR}/${mission_id}.event" >/dev/null &&
        rg "${mission_id}" "${LOG_DIR}/${mission_id}.event" >/dev/null &&
        rg "FINISHED" "${LOG_DIR}/${mission_id}.event" >/dev/null; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "mission did not finish: ${mission_id}" >&2
      cat "${LOG_DIR}/${mission_id}.event" >&2 || true
      return 1
    fi
    sleep 1
  done
}

start_bg navigate_sequence_server ros2 run robot_tasks navigate_sequence_server_node --ros-args \
  -p use_nav2_action:=false \
  -p simulate_without_nav2:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  server_timeout_ms:=3000 \
  return_to_dock_on_low_battery:=false

wait_service_type /v2/request_lift_session "robot_interfaces_facility/srv/RequestLiftSession" 20 \
  "${LOG_DIR}/wait_lift_request.err"
wait_service_type /v2/release_lift_session "robot_interfaces_facility/srv/ReleaseLiftSession" 20 \
  "${LOG_DIR}/wait_lift_release.err"
wait_service_type /v2/get_lift_state "robot_interfaces_facility/srv/GetLiftState" 20 \
  "${LOG_DIR}/wait_lift_state.err"
wait_submit_order 20 "${LOG_DIR}/wait_sequence.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

ros2 service call /v2/request_lift_session robot_interfaces_facility/srv/RequestLiftSession \
  "{session_id: floor_sequence_lift_alpha, lift_id: freight_elevator, requester_id: robot_1, from_station: receiving, to_station: packing, release_existing_for_requester: true}" \
  >"${LOG_DIR}/request_lift.out"
rg "success=True" "${LOG_DIR}/request_lift.out"
rg "lift_id='freight_elevator'" "${LOG_DIR}/request_lift.out"
rg "status='lift_session'" "${LOG_DIR}/request_lift.out"

call_submit_order floor_alpha station_sequence 40 \
  "station_ids=receiving,packing,dock;start_if_idle=true;preempt_current=false" \
  station_sequence \
  >"${LOG_DIR}/submit_sequence.out"
rg "accepted=True" "${LOG_DIR}/submit_sequence.out"
rg "station_sequence_floor_alpha_leg_1" "${LOG_DIR}/submit_sequence.out"
rg "station_sequence_floor_alpha_leg_2" "${LOG_DIR}/submit_sequence.out"

wait_mission_finished "station_sequence_floor_alpha_leg_1"
wait_mission_finished "station_sequence_floor_alpha_leg_2"

ros2 service call /v2/get_lift_state robot_interfaces_facility/srv/GetLiftState \
  "{lift_id: freight_elevator}" >"${LOG_DIR}/lift_reserved.out"
rg "success=True" "${LOG_DIR}/lift_reserved.out"
rg "available=False" "${LOG_DIR}/lift_reserved.out"
rg "holder_id='robot_1'" "${LOG_DIR}/lift_reserved.out"
rg "status='lift_session'" "${LOG_DIR}/lift_reserved.out"

ros2 service call /v2/release_lift_session robot_interfaces_facility/srv/ReleaseLiftSession \
  "{session_id: floor_sequence_lift_alpha, lift_id: freight_elevator}" \
  >"${LOG_DIR}/release_lift.out"
rg "success=True" "${LOG_DIR}/release_lift.out"
rg "status='available'" "${LOG_DIR}/release_lift.out"

echo "multi-floor sequence passed"
