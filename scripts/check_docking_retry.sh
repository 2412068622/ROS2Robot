#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_docking_retry.XXXXXX)"
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
  return_to_dock_on_low_battery:=false \
  failure_recovery_policy:=retry \
  failure_retry_limit:=1

wait_service_type /v2/request_docking "robot_interfaces_mission/srv/RequestDocking" 20 \
  "${LOG_DIR}/wait_request_docking.err"
wait_service_type /v2/get_dock_state "robot_interfaces_mission/srv/GetDockState" 20 \
  "${LOG_DIR}/wait_get_dock_state.err"
wait_service_type /v2/get_mission_recovery_state "robot_interfaces_mission/srv/GetMissionRecoveryState" 20 \
  "${LOG_DIR}/wait_recovery.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"
wait_service_type /v2/list_facility_resources "robot_interfaces_facility/srv/ListFacilityResources" 20 \
  "${LOG_DIR}/wait_facilities.err"

ros2 service call /v2/request_docking robot_interfaces_mission/srv/RequestDocking \
  "{dock_id: main_dock, request_id: retry_alpha, priority: 50, start_if_idle: true, preempt_current: false, simulate_contact_success: false}" \
  >"${LOG_DIR}/request_docking.out"
rg "success=True" "${LOG_DIR}/request_docking.out"
rg "dock_id='main_dock'" "${LOG_DIR}/request_docking.out"
rg "mission_id='docking_request_retry_alpha'" "${LOG_DIR}/request_docking.out"
rg "state='APPROACHING'" "${LOG_DIR}/request_docking.out"

deadline=$((SECONDS + 40))
while true; do
  ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 20, state_filter: DOCKING_CONTACT_FAILED, mission_id_filter: docking_request_retry_alpha}" \
    >"${LOG_DIR}/contact_failed.out" 2>"${LOG_DIR}/contact_failed.err" || true
  if rg "success=True" "${LOG_DIR}/contact_failed.out" >/dev/null &&
      rg "DOCKING_CONTACT_FAILED" "${LOG_DIR}/contact_failed.out" >/dev/null; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "docking contact failure event was not recorded" >&2
    cat "${LOG_DIR}/contact_failed.out" >&2 || true
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/get_mission_recovery_state robot_interfaces_mission/srv/GetMissionRecoveryState "{}" \
  >"${LOG_DIR}/recovery.out"
rg "success=True" "${LOG_DIR}/recovery.out"
rg "retry_count=(0|1)(,|\\)|$)" "${LOG_DIR}/recovery.out"
rg "last_recovery_action='retry'" "${LOG_DIR}/recovery.out"
rg "last_failure_message='docking contact failed'" "${LOG_DIR}/recovery.out"

deadline=$((SECONDS + 50))
while true; do
  ros2 service call /v2/get_dock_state robot_interfaces_mission/srv/GetDockState "{dock_id: main_dock}" \
    >"${LOG_DIR}/dock_state.out" 2>"${LOG_DIR}/dock_state.err" || true
  if rg "success=True" "${LOG_DIR}/dock_state.out" >/dev/null &&
      rg "state='CHARGING'" "${LOG_DIR}/dock_state.out" >/dev/null; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "dock did not reach CHARGING after retry" >&2
    cat "${LOG_DIR}/dock_state.out" >&2 || true
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: charger, include_disabled: false}" \
  >"${LOG_DIR}/facilities.out"
rg "success=True" "${LOG_DIR}/facilities.out"
rg "resource_ids=\\['charger_main'\\]" "${LOG_DIR}/facilities.out"
rg "status=\\['CHARGING'\\]" "${LOG_DIR}/facilities.out"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: CHARGING, mission_id_filter: docking_request_retry_alpha}" \
  >"${LOG_DIR}/charging_event.out"
rg "success=True" "${LOG_DIR}/charging_event.out"
rg "docking_request_retry_alpha" "${LOG_DIR}/charging_event.out"
rg "CHARGING" "${LOG_DIR}/charging_event.out"
rg "docked and charging" "${LOG_DIR}/charging_event.out"

echo "docking retry passed"
