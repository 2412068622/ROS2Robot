#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_scenario_tasks "robot_interfaces_business/srv/ListScenarioTasks" 10 /tmp/robot_scenario_list.err
wait_submit_order 10 /tmp/robot_scenario_submit.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_scenario_state_topic.err

ros2 service call /v2/list_scenario_tasks robot_interfaces_business/srv/ListScenarioTasks \
  "{scenario_id: warehouse}" \
  | rg "success=True|inbound_to_storage"

call_submit_order scenario_alpha scenario_task 0 \
  "scenario_id=warehouse;task_id=inbound_to_storage;order_id=scenario_alpha;start_if_idle=true;preempt_current=false" \
  scenario_task \
  | rg "accepted=True|station_order_fleet_scenario_alpha"

seen_order=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_scenario_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_scenario_state.err || true
  )"

  if grep -q "station_order_fleet_scenario_alpha" <<<"${active_goal}"; then
    seen_order=true
  fi
  if ${seen_order} && grep -q "FINISHED" <<<"${state}"; then
    echo "scenario task workflow passed"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "scenario task failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "scenario task did not finish; seen_order=${seen_order}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
