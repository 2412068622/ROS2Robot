#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_facility_resources "robot_interfaces_facility/srv/ListFacilityResources" 10 /tmp/robot_facility_list.err
wait_service_type /v2/reserve_facility_resource "robot_interfaces_facility/srv/ReserveFacilityResource" 10 /tmp/robot_facility_reserve.err
wait_service_type /v2/release_facility_resource "robot_interfaces_facility/srv/ReleaseFacilityResource" 10 /tmp/robot_facility_release.err
wait_service_type /v2/request_charging "robot_interfaces_mission/srv/RequestCharging" 10 /tmp/robot_charging_request.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_charging_state_topic.err

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: charger, include_disabled: false}" \
  | rg "success=True|charger_main|available=\\[True"

ros2 service call /v2/reserve_facility_resource robot_interfaces_facility/srv/ReserveFacilityResource \
  "{resource_id: receiving_door, holder_id: operator_a, mission_id: manual_door_check, release_existing_for_holder: true}" \
  | rg "success=True"

ros2 service call /v2/release_facility_resource robot_interfaces_facility/srv/ReleaseFacilityResource \
  "{resource_id: receiving_door, holder_id: operator_a}" \
  | rg "success=True"

ros2 service call /v2/request_charging robot_interfaces_mission/srv/RequestCharging \
  "{request_id: charge_alpha, charger_id: charger_main, priority: 80, start_if_idle: true, preempt_current: false}" \
  | rg "success=True"

seen_charging=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_charging_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_charging_state.err || true
  )"

  if grep -q "charging_request_charge_alpha" <<<"${active_goal}"; then
    seen_charging=true
  fi
  if ${seen_charging} && grep -q "FINISHED" <<<"${state}"; then
    break
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "charging request failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "charging request did not finish; seen_charging=${seen_charging}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: charger, include_disabled: false}" \
  | rg "occupied|charging_request_charge_alpha"

ros2 service call /v2/release_facility_resource robot_interfaces_facility/srv/ReleaseFacilityResource \
  "{resource_id: charger_main, holder_id: charging_request_charge_alpha}" \
  | rg "success=True"

echo "facility resource reservation and charging request workflow passed"
