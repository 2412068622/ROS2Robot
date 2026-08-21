#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_top_module.XXXXXX)"
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
  payload_capacity_kg:=40.0 \
  top_module_type:=lift

wait_service_type /v2/set_payload_loaded "robot_interfaces_facility/srv/SetPayloadLoaded" 20 \
  "${LOG_DIR}/wait_set_payload.err"
wait_service_type /v2/execute_top_module_action "robot_interfaces_facility/srv/ExecuteTopModuleAction" 20 \
  "${LOG_DIR}/wait_top_module.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"
wait_topic_type /payload/state "robot_interfaces/msg/PayloadState" 20 \
  "${LOG_DIR}/wait_payload_state.err"

ros2 service call /v2/set_payload_loaded robot_interfaces_facility/srv/SetPayloadLoaded \
  "{loaded: true, payload_id: shelf_alpha, weight_kg: 18.0, message: shelf loaded for lift test}" \
  | rg "success=True|payload_id='shelf_alpha'|state='LOADED'|loaded=True"

ros2 service call /v2/execute_top_module_action robot_interfaces_facility/srv/ExecuteTopModuleAction \
  "{action: lift_up, payload_id: shelf_alpha, weight_kg: 18.0}" \
  | rg "success=True|action='lift_up'|state='TOP_MODULE_ACTION'|loaded=True|payload_id='shelf_alpha'"

ros2 service call /v2/execute_top_module_action robot_interfaces_facility/srv/ExecuteTopModuleAction \
  "{action: attach, payload_id: shelf_alpha, weight_kg: 18.0}" \
  | rg "success=False|unsupported top module action attach for lift"

ros2 topic echo --once /payload/state robot_interfaces/msg/PayloadState \
  | rg "loaded: true|payload_id: shelf_alpha|state: TOP_MODULE_ACTION|last_action: lift_up"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: TOP_MODULE_ACTION, mission_id_filter: top_module_lift_up}" \
  | rg "success=True|TOP_MODULE_ACTION|executed lift_up on lift"

echo "top module action passed"
