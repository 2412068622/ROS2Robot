#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_station_confirmation.XXXXXX)"
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

wait_service_type /v2/request_station_confirmation "robot_interfaces_facility/srv/RequestStationConfirmation" 20 \
  "${LOG_DIR}/wait_request.err"
wait_service_type /v2/get_pending_confirmations "robot_interfaces_facility/srv/GetPendingConfirmations" 20 \
  "${LOG_DIR}/wait_pending.err"
wait_service_type /v2/confirm_station_action "robot_interfaces_facility/srv/ConfirmStationAction" 20 \
  "${LOG_DIR}/wait_confirm.err"
wait_service_type /v2/reject_station_action "robot_interfaces_facility/srv/RejectStationAction" 20 \
  "${LOG_DIR}/wait_reject.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

ros2 service call /v2/request_station_confirmation robot_interfaces_facility/srv/RequestStationConfirmation \
  "{confirmation_id: pickup_alpha, station_id: receiving, action: load_tote, mission_id: station_order_alpha, operator_hint: scan tote A}" \
  | rg "success=True|confirmation_id='pickup_alpha'|state='PENDING'"

ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: receiving}" \
  | rg "success=True|pickup_alpha|receiving|load_tote|station_order_alpha|PENDING"

ros2 service call /v2/confirm_station_action robot_interfaces_facility/srv/ConfirmStationAction \
  "{confirmation_id: pickup_alpha, operator_id: alice, note: tote scanned}" \
  | rg "success=True|confirmation_id='pickup_alpha'|state='CONFIRMED'"

ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: receiving}" \
  | rg "success=True|loaded 0 pending"

ros2 service call /v2/request_station_confirmation robot_interfaces_facility/srv/RequestStationConfirmation \
  "{confirmation_id: drop_alpha, station_id: packing, action: unload_tote, mission_id: station_order_alpha, operator_hint: place tote}" \
  | rg "success=True|confirmation_id='drop_alpha'|state='PENDING'"

ros2 service call /v2/reject_station_action robot_interfaces_facility/srv/RejectStationAction \
  "{confirmation_id: drop_alpha, operator_id: bob, reason: station occupied}" \
  | rg "success=True|confirmation_id='drop_alpha'|state='REJECTED'"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: CONFIRMATION_CONFIRMED, mission_id_filter: station_confirmation_pickup_alpha}" \
  | rg "success=True|CONFIRMATION_CONFIRMED|tote scanned by alice"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: CONFIRMATION_REJECTED, mission_id_filter: station_confirmation_drop_alpha}" \
  | rg "success=True|CONFIRMATION_REJECTED|station occupied by bob"

echo "station confirmation passed"
