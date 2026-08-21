#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/execute_facility_action "robot_interfaces_facility/srv/ExecuteFacilityAction" 10 /tmp/robot_facility_action.err
wait_service_type /v2/list_facility_resources "robot_interfaces_facility/srv/ListFacilityResources" 10 /tmp/robot_facility_action_list.err
wait_service_type /v2/release_facility_resource "robot_interfaces_facility/srv/ReleaseFacilityResource" 10 /tmp/robot_facility_action_release.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_facility_action_state_topic.err

ros2 service call /v2/execute_facility_action robot_interfaces_facility/srv/ExecuteFacilityAction \
  "{request_id: door_alpha, resource_id: receiving_door, resource_type: door, action: open, priority: 70, start_if_idle: true, preempt_current: false, hold_after_action: true}" \
  | rg "success=True|facility_action_door_alpha"

seen_door=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_facility_action_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_facility_action_state.err || true
  )"

  if grep -q "facility_action_door_alpha" <<<"${active_goal}"; then
    seen_door=true
  fi
  if ${seen_door} && grep -q "FINISHED" <<<"${state}"; then
    break
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "door facility action failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "door facility action did not finish; seen_door=${seen_door}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: door, include_disabled: false}" \
  | rg "occupied|facility_action_door_alpha"

ros2 service call /v2/release_facility_resource robot_interfaces_facility/srv/ReleaseFacilityResource \
  "{resource_id: receiving_door, holder_id: facility_action_door_alpha}" \
  | rg "success=True"

ros2 service call /v2/execute_facility_action robot_interfaces_facility/srv/ExecuteFacilityAction \
  "{request_id: elevator_alpha, resource_id: freight_elevator, resource_type: elevator, action: call, priority: 65, start_if_idle: true, preempt_current: false, hold_after_action: false}" \
  | rg "success=True|facility_action_elevator_alpha"

seen_elevator=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_facility_action_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_facility_action_state.err || true
  )"

  if grep -q "facility_action_elevator_alpha" <<<"${active_goal}"; then
    seen_elevator=true
  fi
  if ${seen_elevator} && grep -q "FINISHED" <<<"${state}"; then
    break
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "elevator facility action failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "elevator facility action did not finish; seen_elevator=${seen_elevator}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: elevator, include_disabled: false}" \
  | rg "available=\\[True"

echo "facility action workflow passed"
