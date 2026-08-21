#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_fleet_robots "robot_interfaces_fleet/srv/ListFleetRobots" 10 /tmp/robot_fleet_list.err
wait_submit_order 10 /tmp/robot_fleet_task.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_fleet_state_topic.err

ros2 service call /v2/list_fleet_robots robot_interfaces_fleet/srv/ListFleetRobots \
  "{capability: station_transport, include_disabled: false}" \
  | rg "success=True|robot_1|station_transport"

call_submit_order alpha fleet_station 45 \
  "task_id=alpha;required_capability=station_transport;pickup_station=receiving;dropoff_station=storage_a;start_if_idle=true;preempt_current=false" \
  fleet_station_task \
  | rg "accepted=True|station_order_fleet_alpha"

seen_task=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_fleet_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_fleet_state.err || true
  )"

  if grep -q "station_order_fleet_alpha" <<<"${active_goal}"; then
    seen_task=true
  fi
  if ${seen_task} && grep -q "FINISHED" <<<"${state}"; then
    echo "fleet station task assigned to local robot and finished"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "fleet station task failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "fleet station task did not finish; seen_task=${seen_task}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
