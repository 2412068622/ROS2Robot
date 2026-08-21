#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_payload.XXXXXX)"
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
  return_to_dock_on_low_battery:=false \
  payload_capacity_kg:=30.0 \
  top_module_type:=manual_load

wait_service_type /v2/confirm_load "robot_interfaces_facility/srv/ConfirmLoad" 20 "${LOG_DIR}/wait_confirm_load.err"
wait_service_type /v2/confirm_unload "robot_interfaces_facility/srv/ConfirmUnload" 20 "${LOG_DIR}/wait_confirm_unload.err"
wait_service_type /v2/set_payload_loaded "robot_interfaces_facility/srv/SetPayloadLoaded" 20 "${LOG_DIR}/wait_set_payload.err"
wait_service_type /v2/execute_top_module_action "robot_interfaces_facility/srv/ExecuteTopModuleAction" 20 "${LOG_DIR}/wait_top_module.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 "${LOG_DIR}/wait_events.err"
wait_topic_type /payload/state "robot_interfaces/msg/PayloadState" 20 "${LOG_DIR}/wait_payload_topic.err"

ros2 service call /v2/confirm_load robot_interfaces_facility/srv/ConfirmLoad \
  "{payload_id: tote_heavy, weight_kg: 35.0, station_id: receiving}" \
  | rg "success=False|payload overweight|state='EMPTY'|loaded=False"

ros2 service call /v2/confirm_load robot_interfaces_facility/srv/ConfirmLoad \
  "{payload_id: tote_alpha, weight_kg: 12.5, station_id: receiving}" \
  | rg "success=True|payload_id='tote_alpha'|state='LOADED'|loaded=True"

ros2 topic echo --once /payload/state robot_interfaces/msg/PayloadState \
  | rg "loaded: true|payload_id: tote_alpha|state: LOADED|capacity_kg: 30"

ros2 service call /v2/confirm_load robot_interfaces_facility/srv/ConfirmLoad \
  "{payload_id: tote_beta, weight_kg: 8.0, station_id: receiving}" \
  | rg "success=False|payload already loaded: tote_alpha|payload_id='tote_alpha'|state='LOADED'|loaded=True"

ros2 topic echo --once /payload/state robot_interfaces/msg/PayloadState \
  | rg "loaded: true|payload_id: tote_alpha|state: LOADED"

ros2 service call /v2/execute_top_module_action robot_interfaces_facility/srv/ExecuteTopModuleAction \
  "{action: inspect, payload_id: tote_alpha, weight_kg: 12.5}" \
  | rg "success=True|action='inspect'|state='TOP_MODULE_ACTION'|loaded=True"

ros2 service call /v2/confirm_unload robot_interfaces_facility/srv/ConfirmUnload \
  "{payload_id: wrong_tote, station_id: packing}" \
  | rg "success=False|loaded payload mismatch|payload_id='tote_alpha'"

ros2 service call /v2/confirm_unload robot_interfaces_facility/srv/ConfirmUnload \
  "{payload_id: tote_alpha, station_id: packing}" \
  | rg "success=True|state='EMPTY'|loaded=False"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: PAYLOAD_LOADED, mission_id_filter: payload_tote_alpha}" \
  | rg "success=True|PAYLOAD_LOADED|loaded tote_alpha"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: PAYLOAD_UNLOADED, mission_id_filter: payload_tote_alpha}" \
  | rg "success=True|PAYLOAD_UNLOADED|unloaded tote_alpha"

echo "payload load unload passed"
