#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/preview_fleet_station_task "robot_interfaces_fleet/srv/PreviewFleetStationTask" 10 /tmp/robot_fleet_preview.err
wait_service_type /v2/update_fleet_robot_state "robot_interfaces_fleet/srv/UpdateFleetRobotState" 10 /tmp/robot_fleet_preview_update.err
wait_submit_order 10 /tmp/robot_fleet_preview_submit.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_fleet_preview_state_topic.err

ros2 service call /v2/preview_fleet_station_task robot_interfaces_fleet/srv/PreviewFleetStationTask \
  "{required_capability: station_transport, pickup_station: receiving, dropoff_station: storage_a}" \
  | rg "success=True|assigned_robot_id='robot_1'|local_robot=True|estimated_total_distance_m=5\\.1|approach_distance_m=3\\.9|task_distance_m=1\\.2"

ros2 service call /v2/update_fleet_robot_state robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_2, enabled: true, state: available, battery_voltage: 25.2, current_station_id: receiving, update_enabled: false, update_state: true, update_battery_voltage: true, update_current_station_id: true}" \
  | rg "success=True|robot_2"

ros2 service call /v2/preview_fleet_station_task robot_interfaces_fleet/srv/PreviewFleetStationTask \
  "{required_capability: station_transport, pickup_station: receiving, dropoff_station: storage_a}" \
  | rg "success=True|assigned_robot_id='robot_2'|local_robot=False|estimated_total_distance_m=1\\.2|approach_distance_m=0\\.0|task_distance_m=1\\.2"

ros2 service call /v2/update_fleet_robot_state robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_2, enabled: true, state: charging, battery_voltage: 23.0, current_station_id: dock, update_enabled: false, update_state: true, update_battery_voltage: true, update_current_station_id: true}" \
  | rg "success=True|robot_2"

call_submit_order preview_local fleet_station 45 \
  "task_id=preview_local;required_capability=station_transport;pickup_station=receiving;dropoff_station=storage_a;start_if_idle=true;preempt_current=false" \
  fleet_station_task \
  | rg "accepted=True|station_order_fleet_preview_local"

seen_task=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_fleet_preview_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_fleet_preview_state.err || true
  )"

  if grep -q "station_order_fleet_preview_local" <<<"${active_goal}"; then
    seen_task=true
  fi
  if ${seen_task} && grep -q "FINISHED" <<<"${state}"; then
    echo "fleet preview workflow passed"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "fleet preview workflow failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "fleet preview workflow did not finish; seen_task=${seen_task}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
