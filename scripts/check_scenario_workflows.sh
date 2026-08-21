#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_scenario_workflows "robot_interfaces_business/srv/ListScenarioWorkflows" 10 /tmp/robot_scenario_workflows_list.err
wait_submit_order 10 /tmp/robot_scenario_workflow_submit.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_scenario_workflow_state_topic.err

ros2 service call /v2/list_scenario_workflows robot_interfaces_business/srv/ListScenarioWorkflows \
  "{scenario_id: warehouse}" \
  | rg "success=True|inbound_crossdock|inbound_to_storage,storage_to_packing"

call_submit_order workflow_alpha scenario_workflow 0 \
  "scenario_id=warehouse;workflow_id=inbound_crossdock;order_id=workflow_alpha;start_if_idle=true;preempt_current=false" \
  scenario_workflow \
  | rg "accepted=True|station_order_fleet_workflow_alpha_step_1|station_order_fleet_workflow_alpha_step_2"

seen_first=false
seen_second=false
deadline=$((SECONDS + 40))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_scenario_workflow_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_scenario_workflow_state.err || true
  )"

  if grep -q "station_order_fleet_workflow_alpha_step_1" <<<"${active_goal}"; then
    seen_first=true
  fi
  if grep -q "station_order_fleet_workflow_alpha_step_2" <<<"${active_goal}"; then
    seen_second=true
  fi
  if ${seen_first} && ${seen_second} && grep -q "FINISHED" <<<"${state}"; then
    echo "scenario workflow passed"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "scenario workflow failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "scenario workflow did not finish; seen_first=${seen_first}, seen_second=${seen_second}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
