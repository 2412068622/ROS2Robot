#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_door_passage_workflow.XXXXXX)"
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

start_bg navigate_sequence_server ros2 run robot_tasks navigate_sequence_server_node --ros-args \
  -p use_nav2_action:=false \
  -p simulate_without_nav2:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  server_timeout_ms:=3000 \
  return_to_dock_on_low_battery:=false

wait_service_type /v2/execute_facility_action "robot_interfaces_facility/srv/ExecuteFacilityAction" 20 \
  "${LOG_DIR}/wait_execute.err"
wait_service_type /v2/get_door_state "robot_interfaces_facility/srv/GetDoorState" 20 \
  "${LOG_DIR}/wait_door_state.err"
wait_service_type /v2/release_facility_resource "robot_interfaces_facility/srv/ReleaseFacilityResource" 20 \
  "${LOG_DIR}/wait_release.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

ros2 service call /v2/execute_facility_action robot_interfaces_facility/srv/ExecuteFacilityAction \
  "{request_id: door_passage_alpha, resource_id: receiving_door, resource_type: door, action: pass, priority: 45, start_if_idle: true, preempt_current: false, hold_after_action: true}" \
  >"${LOG_DIR}/execute_door.out"
rg "success=True" "${LOG_DIR}/execute_door.out"
rg "mission_id='facility_action_door_passage_alpha'" "${LOG_DIR}/execute_door.out"
rg "resource_id='receiving_door'" "${LOG_DIR}/execute_door.out"
rg "status='reserved'" "${LOG_DIR}/execute_door.out"

deadline=$((SECONDS + 50))
while true; do
  ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 30, state_filter: FINISHED, mission_id_filter: facility_action_door_passage_alpha}" \
    >"${LOG_DIR}/door_event.out" 2>"${LOG_DIR}/door_event.err" || true
  if rg "success=True" "${LOG_DIR}/door_event.out" >/dev/null &&
      rg "facility_action_door_passage_alpha" "${LOG_DIR}/door_event.out" >/dev/null &&
      rg "FINISHED" "${LOG_DIR}/door_event.out" >/dev/null; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "door passage facility action did not finish" >&2
    cat "${LOG_DIR}/door_event.out" >&2 || true
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/get_door_state robot_interfaces_facility/srv/GetDoorState \
  "{door_id: receiving_door}" >"${LOG_DIR}/door_occupied.out"
rg "success=True" "${LOG_DIR}/door_occupied.out"
rg "available=False" "${LOG_DIR}/door_occupied.out"
rg "holder_id='facility_action_door_passage_alpha'" "${LOG_DIR}/door_occupied.out"
rg "status='occupied'" "${LOG_DIR}/door_occupied.out"

ros2 service call /v2/release_facility_resource robot_interfaces_facility/srv/ReleaseFacilityResource \
  "{resource_id: receiving_door, holder_id: facility_action_door_passage_alpha}" \
  >"${LOG_DIR}/release_door.out"
rg "success=True" "${LOG_DIR}/release_door.out"
rg "status='available'" "${LOG_DIR}/release_door.out"

ros2 service call /v2/get_door_state robot_interfaces_facility/srv/GetDoorState \
  "{door_id: receiving_door}" >"${LOG_DIR}/door_available.out"
rg "success=True" "${LOG_DIR}/door_available.out"
rg "available=True" "${LOG_DIR}/door_available.out"
rg "status='available'" "${LOG_DIR}/door_available.out"

echo "door passage workflow passed"
