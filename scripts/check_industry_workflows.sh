#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_scenario_workflows "robot_interfaces_business/srv/ListScenarioWorkflows" 10 /tmp/robot_industry_workflows_list.err
wait_submit_order 10 /tmp/robot_industry_workflow_submit.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_industry_workflow_events.err

ros2 service call /v2/list_scenario_workflows robot_interfaces_business/srv/ListScenarioWorkflows \
  "{scenario_id: ''}" \
  | rg "warehouse|manufacturing|hospital|office|hotel|laboratory|campus|inbound_crossdock|line_replenishment_loop|pharmacy_lab_round|workplace_services_round|guest_amenity_round|sample_processing_round|mail_distribution_round"

run_workflow() {
  local scenario_id="$1"
  local workflow_id="$2"
  local order_id="$3"
  local expected_first="station_order_fleet_${order_id}_step_1"
  local expected_second="station_order_fleet_${order_id}_step_2"

  call_submit_order "${order_id}" scenario_workflow 0 \
    "scenario_id=${scenario_id};workflow_id=${workflow_id};order_id=${order_id};start_if_idle=true;preempt_current=false" \
    scenario_workflow \
    | rg "accepted=True|${expected_first}|${expected_second}"

  for mission_id in "${expected_first}" "${expected_second}"; do
    local deadline=$((SECONDS + 45))
    local out_file="/tmp/robot_industry_workflow_${mission_id}.out"
    while true; do
      ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
        "{limit: 5, state_filter: FINISHED, mission_id_filter: ${mission_id}}" \
        >"${out_file}" || true
      if rg "success=True" "${out_file}" >/dev/null && rg "states=\\['FINISHED'\\]" "${out_file}" >/dev/null; then
        break
      fi
      if (( SECONDS >= deadline )); then
        echo "${scenario_id}/${workflow_id} mission ${mission_id} did not finish" >&2
        cat "${out_file}" >&2 || true
        exit 1
      fi
      sleep 1
    done
  done
}

run_workflow manufacturing line_replenishment_loop industry_manufacturing
run_workflow hospital pharmacy_lab_round industry_hospital
run_workflow office workplace_services_round industry_office
run_workflow hotel guest_amenity_round industry_hotel
run_workflow laboratory sample_processing_round industry_laboratory
run_workflow campus mail_distribution_round industry_campus

echo "industry scenario workflows passed"
