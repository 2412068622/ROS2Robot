#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_lift_door.XXXXXX)"
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

start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  return_to_dock_on_low_battery:=false

wait_service_type /v2/get_lift_state "robot_interfaces_facility/srv/GetLiftState" 20 \
  "${LOG_DIR}/wait_lift_state.err"
wait_service_type /v2/get_door_state "robot_interfaces_facility/srv/GetDoorState" 20 \
  "${LOG_DIR}/wait_door_state.err"
wait_service_type /v2/request_lift_session "robot_interfaces_facility/srv/RequestLiftSession" 20 \
  "${LOG_DIR}/wait_lift_request.err"
wait_service_type /v2/release_lift_session "robot_interfaces_facility/srv/ReleaseLiftSession" 20 \
  "${LOG_DIR}/wait_lift_release.err"
wait_service_type /v2/reserve_facility_resource "robot_interfaces_facility/srv/ReserveFacilityResource" 20 \
  "${LOG_DIR}/wait_reserve.err"
wait_service_type /v2/release_facility_resource "robot_interfaces_facility/srv/ReleaseFacilityResource" 20 \
  "${LOG_DIR}/wait_release.err"

ros2 service call /v2/get_lift_state robot_interfaces_facility/srv/GetLiftState \
  "{lift_id: freight_elevator}" \
  | rg "success=True|lift_id='freight_elevator'|station_id='packing'|available=True|status='available'"

ros2 service call /v2/request_lift_session robot_interfaces_facility/srv/RequestLiftSession \
  "{session_id: lift_session_alpha, lift_id: freight_elevator, requester_id: robot_1, from_station: receiving, to_station: packing, release_existing_for_requester: true}" \
  | rg "success=True|session_id='lift_session_alpha'|lift_id='freight_elevator'|status='lift_session'"

ros2 service call /v2/get_lift_state robot_interfaces_facility/srv/GetLiftState \
  "{lift_id: freight_elevator}" \
  | rg "success=True|available=False|holder_id='robot_1'|status='lift_session'"

ros2 service call /v2/request_lift_session robot_interfaces_facility/srv/RequestLiftSession \
  "{session_id: lift_session_beta, lift_id: freight_elevator, requester_id: robot_2, from_station: receiving, to_station: packing, release_existing_for_requester: false}" \
  | rg "success=False|lift already reserved by robot_1"

ros2 service call /v2/release_lift_session robot_interfaces_facility/srv/ReleaseLiftSession \
  "{session_id: lift_session_alpha, lift_id: freight_elevator}" \
  | rg "success=True|status='available'"

ros2 service call /v2/get_lift_state robot_interfaces_facility/srv/GetLiftState \
  "{lift_id: freight_elevator}" \
  | rg "success=True|available=True|status='available'"

ros2 service call /v2/get_door_state robot_interfaces_facility/srv/GetDoorState \
  "{door_id: receiving_door}" \
  | rg "success=True|door_id='receiving_door'|station_id='receiving'|available=True|status='available'"

ros2 service call /v2/reserve_facility_resource robot_interfaces_facility/srv/ReserveFacilityResource \
  "{resource_id: receiving_door, holder_id: operator_a, mission_id: door_hold_alpha, release_existing_for_holder: true}" \
  | rg "success=True|status='reserved'"

ros2 service call /v2/get_door_state robot_interfaces_facility/srv/GetDoorState \
  "{door_id: receiving_door}" \
  | rg "success=True|available=False|holder_id='operator_a'|status='reserved'"

ros2 service call /v2/release_facility_resource robot_interfaces_facility/srv/ReleaseFacilityResource \
  "{resource_id: receiving_door, holder_id: operator_a}" \
  | rg "success=True|status='available'"

echo "lift and door state passed"
