#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_scenario_tasks "robot_interfaces_business/srv/ListScenarioTasks" 10 /tmp/robot_scenario_sequence_tasks.err
wait_submit_order 10 /tmp/robot_scenario_sequence_submit.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_scenario_sequence_events.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_scenario_sequence_state_topic.err

ros2 service call /v2/list_scenario_tasks robot_interfaces_business/srv/ListScenarioTasks \
  "{scenario_id: campus}" \
  | rg "success=True|full_mail_route|station_sequence|dock,receiving,storage_a,packing,dock"

call_submit_order campus_sequence_alpha scenario_workflow 0 \
  "scenario_id=campus;workflow_id=full_mail_route;order_id=campus_sequence_alpha;start_if_idle=true;preempt_current=false" \
  scenario_workflow \
  | rg "accepted=True|station_sequence_campus_sequence_alpha_step_1_leg_1"

for mission in \
  station_sequence_campus_sequence_alpha_step_1_leg_1 \
  station_sequence_campus_sequence_alpha_step_1_leg_2 \
  station_sequence_campus_sequence_alpha_step_1_leg_3 \
  station_sequence_campus_sequence_alpha_step_1_leg_4; do
  deadline=$((SECONDS + 45))
  while true; do
    if ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 80, state_filter: FINISHED, mission_id_filter: ${mission}}" \
      2>/tmp/robot_scenario_sequence_events.err | rg "loaded 1 mission event|states=\\['FINISHED'\\]" >/dev/null; then
      break
    fi
    if (( SECONDS >= deadline )); then
      echo "scenario station sequence mission did not finish: ${mission}" >&2
      exit 1
    fi
    sleep 1
  done
done

echo "scenario station sequence passed"
