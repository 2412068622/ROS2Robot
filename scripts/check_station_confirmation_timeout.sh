#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_station_confirmation_timeout.XXXXXX)"
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
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

SKIP_REQUEST_OUT="${LOG_DIR}/skip_request.out"
ros2 service call /v2/request_station_confirmation robot_interfaces_facility/srv/RequestStationConfirmation \
  "{confirmation_id: timeout_skip_alpha, station_id: receiving, action: pickup_scan, mission_id: station_order_timeout, operator_hint: scan tote, timeout_sec: 1, timeout_policy: skip}" \
  >"${SKIP_REQUEST_OUT}"
rg "success=True" "${SKIP_REQUEST_OUT}"
rg "confirmation_id='timeout_skip_alpha'" "${SKIP_REQUEST_OUT}"
rg "state='PENDING'" "${SKIP_REQUEST_OUT}"

sleep 2

SKIP_PENDING_OUT="${LOG_DIR}/skip_pending.out"
ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: receiving}" \
  >"${SKIP_PENDING_OUT}"
rg "success=True" "${SKIP_PENDING_OUT}"
rg "loaded 0 pending" "${SKIP_PENDING_OUT}"

SKIP_EVENTS_OUT="${LOG_DIR}/skip_events.out"
ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: CONFIRMATION_SKIPPED, mission_id_filter: station_confirmation_timeout_skip_alpha}" \
  >"${SKIP_EVENTS_OUT}"
rg "success=True" "${SKIP_EVENTS_OUT}"
rg "CONFIRMATION_SKIPPED" "${SKIP_EVENTS_OUT}"
rg "policy skip" "${SKIP_EVENTS_OUT}"

MANUAL_REQUEST_OUT="${LOG_DIR}/manual_request.out"
ros2 service call /v2/request_station_confirmation robot_interfaces_facility/srv/RequestStationConfirmation \
  "{confirmation_id: timeout_manual_alpha, station_id: packing, action: unload_scan, mission_id: station_order_timeout, operator_hint: place tote, timeout_sec: 1, timeout_policy: manual}" \
  >"${MANUAL_REQUEST_OUT}"
rg "success=True" "${MANUAL_REQUEST_OUT}"
rg "confirmation_id='timeout_manual_alpha'" "${MANUAL_REQUEST_OUT}"
rg "state='PENDING'" "${MANUAL_REQUEST_OUT}"

sleep 2

MANUAL_PENDING_OUT="${LOG_DIR}/manual_pending.out"
ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: packing}" \
  >"${MANUAL_PENDING_OUT}"
rg "success=True" "${MANUAL_PENDING_OUT}"
rg "loaded 0 pending" "${MANUAL_PENDING_OUT}"

MANUAL_EVENTS_OUT="${LOG_DIR}/manual_events.out"
ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: CONFIRMATION_TIMED_OUT, mission_id_filter: station_confirmation_timeout_manual_alpha}" \
  >"${MANUAL_EVENTS_OUT}"
rg "success=True" "${MANUAL_EVENTS_OUT}"
rg "CONFIRMATION_TIMED_OUT" "${MANUAL_EVENTS_OUT}"
rg "policy manual" "${MANUAL_EVENTS_OUT}"

echo "station confirmation timeout passed"
