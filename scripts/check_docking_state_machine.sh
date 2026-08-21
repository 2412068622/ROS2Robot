#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_docking_state_machine.XXXXXX)"
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

wait_service_type /v2/list_docks "robot_interfaces_mission/srv/ListDocks" 20 "${LOG_DIR}/wait_list_docks.err"
wait_service_type /v2/request_docking "robot_interfaces_mission/srv/RequestDocking" 20 "${LOG_DIR}/wait_request_docking.err"
wait_service_type /v2/request_undocking "robot_interfaces_mission/srv/RequestUndocking" 20 "${LOG_DIR}/wait_request_undocking.err"
wait_service_type /v2/get_dock_state "robot_interfaces_mission/srv/GetDockState" 20 "${LOG_DIR}/wait_get_dock_state.err"
wait_service_type /v2/list_facility_resources "robot_interfaces_facility/srv/ListFacilityResources" 20 "${LOG_DIR}/wait_facilities.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 "${LOG_DIR}/wait_events.err"

ros2 service call /v2/list_docks robot_interfaces_mission/srv/ListDocks "{include_disabled: false}" \
  | rg "success=True|dock_ids=\\['main_dock'\\]|states=\\['AVAILABLE'\\]"

ros2 service call /v2/request_docking robot_interfaces_mission/srv/RequestDocking \
  "{dock_id: main_dock, request_id: alpha, priority: 50, start_if_idle: true, preempt_current: false, simulate_contact_success: true}" \
  | rg "success=True|dock_id='main_dock'|mission_id='docking_request_alpha'|state='APPROACHING'"

deadline=$((SECONDS + 40))
while true; do
  state_output="$(
    ros2 service call /v2/get_dock_state robot_interfaces_mission/srv/GetDockState "{dock_id: main_dock}" \
      2>"${LOG_DIR}/get_dock_state.err" || true
  )"
  if grep -q "success=True" <<<"${state_output}" && grep -q "state='CHARGING'" <<<"${state_output}"; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "dock did not reach CHARGING state: ${state_output}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: charger, include_disabled: false}" \
  | rg "success=True|resource_ids=\\['charger_main'\\]|status=\\['CHARGING'\\]"

ros2 service call /v2/request_docking robot_interfaces_mission/srv/RequestDocking \
  "{dock_id: main_dock, request_id: beta, priority: 50, start_if_idle: false, preempt_current: false, simulate_contact_success: true}" \
  | rg "success=False|dock unavailable|state='CHARGING'"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: CHARGING, mission_id_filter: docking_request_alpha}" \
  | rg "success=True|docking_request_alpha|CHARGING|docked and charging"

ros2 service call /v2/request_undocking robot_interfaces_mission/srv/RequestUndocking \
  "{dock_id: main_dock, request_id: alpha, release_charger: true}" \
  | rg "success=True|dock_id='main_dock'|state='UNDOCKED'"

ros2 service call /v2/get_dock_state robot_interfaces_mission/srv/GetDockState "{dock_id: main_dock}" \
  | rg "success=True|state='UNDOCKED'|available=True"

echo "docking state machine passed"
