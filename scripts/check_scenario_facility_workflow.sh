#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_scenario_tasks "robot_interfaces_business/srv/ListScenarioTasks" 10 /tmp/robot_scenario_facility_tasks.err
wait_submit_order 10 /tmp/robot_scenario_facility_submit.err
wait_service_type /v2/list_facility_resources "robot_interfaces_facility/srv/ListFacilityResources" 10 /tmp/robot_scenario_facility_list.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_scenario_facility_events.err

ros2 service call /v2/list_scenario_tasks robot_interfaces_business/srv/ListScenarioTasks \
  "{scenario_id: warehouse}" \
  | rg "open_receiving_door|facility_action|receiving_door|open|inbound_to_storage|station_transport"

call_submit_order facility_workflow_alpha scenario_workflow 0 \
  "scenario_id=warehouse;workflow_id=door_assisted_inbound;order_id=facility_workflow_alpha;start_if_idle=true;preempt_current=false" \
  scenario_workflow \
  | rg "accepted=True|facility_action_facility_workflow_alpha_step_1|station_order_fleet_facility_workflow_alpha_step_2"

for mission_id in facility_action_facility_workflow_alpha_step_1 station_order_fleet_facility_workflow_alpha_step_2; do
  deadline=$((SECONDS + 45))
  out_file="/tmp/robot_scenario_facility_${mission_id}.out"
  while true; do
    ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 5, state_filter: FINISHED, mission_id_filter: ${mission_id}}" \
      >"${out_file}" || true
    if rg "success=True" "${out_file}" >/dev/null && rg "states=\\['FINISHED'\\]" "${out_file}" >/dev/null; then
      break
    fi
    if (( SECONDS >= deadline )); then
      echo "scenario facility workflow mission ${mission_id} did not finish" >&2
      cat "${out_file}" >&2 || true
      exit 1
    fi
    sleep 1
  done
done

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: door, include_disabled: false}" \
  | rg "receiving_door|available=True"

echo "scenario facility workflow passed"
