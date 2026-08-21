#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_scenario_workflows "robot_interfaces_business/srv/ListScenarioWorkflows" 10 /tmp/robot_more_workflows_list.err
wait_submit_order 10 /tmp/robot_more_workflows_submit.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_more_workflows_events.err

ros2 service call /v2/list_scenario_workflows robot_interfaces_business/srv/ListScenarioWorkflows \
  "{scenario_id: ''}" \
  | rg "retail|restaurant|library|airport|factory_qc|store_restock_and_recovery|service_and_dish_return|returns_and_floor_round|security_and_gate_supply|sample_and_rework_loop"

check_workflow() {
  local scenario_id="$1"
  local workflow_id="$2"
  local order_id="$3"
  local first_mission="$4"
  local final_mission="$5"

  call_submit_order "${order_id}" scenario_workflow 0 \
    "scenario_id=${scenario_id};workflow_id=${workflow_id};order_id=${order_id};start_if_idle=true;preempt_current=false" \
    scenario_workflow \
    | rg "accepted=True|${workflow_id}|${first_mission}|${final_mission}"

  wait_finished() {
    local mission_id="$1"
    local timeout_sec="$2"
    local deadline=$((SECONDS + timeout_sec))
    local out_file="/tmp/robot_more_workflows_${mission_id}.out"
    while true; do
      ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
        "{limit: 100, state_filter: FINISHED, mission_id_filter: ${mission_id}}" \
        >"${out_file}" 2>/tmp/robot_more_workflows_events.err || true
      if rg "mission_ids=\\['${mission_id}'\\]" "${out_file}" >/dev/null &&
        rg "states=\\['FINISHED'\\]" "${out_file}" >/dev/null; then
        return 0
      fi
      if (( SECONDS >= deadline )); then
        cat "${out_file}" >&2 || true
        return 1
      fi
      sleep 1
    done
  }

  wait_finished "${first_mission}" 75 || {
    echo "workflow first mission did not finish: ${scenario_id}/${workflow_id}/${first_mission}" >&2
    exit 1
  }
  wait_finished "${final_mission}" 90 || {
    echo "workflow final mission did not finish: ${scenario_id}/${workflow_id}/${final_mission}" >&2
    exit 1
  }
}

check_workflow retail store_restock_and_recovery retail_alpha station_order_fleet_retail_alpha_step_1 station_order_fleet_retail_alpha_step_2
check_workflow restaurant service_and_dish_return restaurant_alpha station_order_fleet_restaurant_alpha_step_1 station_sequence_restaurant_alpha_step_2_leg_2
check_workflow library returns_and_floor_round library_alpha station_sequence_library_alpha_step_1_leg_1 station_order_fleet_library_alpha_step_2
check_workflow airport security_and_gate_supply airport_alpha station_order_fleet_airport_alpha_step_1 station_sequence_airport_alpha_step_2_leg_3
check_workflow factory_qc sample_and_rework_loop factory_qc_alpha station_order_fleet_factory_qc_alpha_step_1 station_sequence_factory_qc_alpha_step_2_leg_3

echo "more industry workflows passed"
