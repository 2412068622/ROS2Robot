#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/request_opportunity_charging "robot_interfaces_mission/srv/RequestOpportunityCharging" 10 /tmp/robot_opportunity_charge_service.err
wait_service_type /v2/list_facility_resources "robot_interfaces_facility/srv/ListFacilityResources" 10 /tmp/robot_opportunity_charge_list.err
wait_service_type /v2/release_facility_resource "robot_interfaces_facility/srv/ReleaseFacilityResource" 10 /tmp/robot_opportunity_charge_release.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_opportunity_charge_state_topic.err

ros2 service call /v2/request_opportunity_charging robot_interfaces_mission/srv/RequestOpportunityCharging \
  "{request_id: high_battery_skip, charger_id: charger_main, battery_voltage: 24.5, opportunity_threshold_voltage: 23.2, critical_threshold_voltage: 22.0, priority: 0, start_if_idle: true, preempt_current: false, require_idle_queue: true}" \
  | rg "success=True|charging_queued=False|battery above threshold"

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: charger, include_disabled: false}" \
  | rg "available=\\[True"

ros2 service call /v2/request_opportunity_charging robot_interfaces_mission/srv/RequestOpportunityCharging \
  "{request_id: opp_alpha, charger_id: charger_main, battery_voltage: 22.8, opportunity_threshold_voltage: 23.2, critical_threshold_voltage: 22.0, priority: 0, start_if_idle: true, preempt_current: false, require_idle_queue: true}" \
  | rg "success=True|charging_queued=True|charging_request_opp_alpha|opportunity queued charging request"

seen_charging=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_opportunity_charge_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_opportunity_charge_state.err || true
  )"

  if grep -q "charging_request_opp_alpha" <<<"${active_goal}"; then
    seen_charging=true
  fi
  if ${seen_charging} && grep -q "FINISHED" <<<"${state}"; then
    break
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "opportunity charging failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "opportunity charging did not finish; seen_charging=${seen_charging}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources \
  "{resource_type: charger, include_disabled: false}" \
  | rg "occupied|charging_request_opp_alpha"

ros2 service call /v2/release_facility_resource robot_interfaces_facility/srv/ReleaseFacilityResource \
  "{resource_id: charger_main, holder_id: charging_request_opp_alpha}" \
  | rg "success=True"

echo "opportunity charging workflow passed"
