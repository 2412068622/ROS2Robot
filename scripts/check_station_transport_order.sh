#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_stations "robot_interfaces_mission/srv/ListStations" 10 /tmp/robot_station_list_service.err
wait_submit_order 10 /tmp/robot_station_order_service.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_station_order_topic.err

ros2 service call /v2/list_stations robot_interfaces_mission/srv/ListStations "{}" \
  | rg "success=True" \
  | cat >/dev/null
ros2 service call /v2/list_stations robot_interfaces_mission/srv/ListStations "{}" \
  | rg "receiving|storage_a"

call_submit_order station_alpha station_transport 30 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=true;preempt_current=false" \
  station_transport_order \
  | rg "accepted=True|station_order_station_alpha"

seen_order=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_station_order_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_station_order_state.err || true
  )"

  if grep -q "station_order_station_alpha" <<<"${active_goal}"; then
    seen_order=true
  fi
  if ${seen_order} && grep -q "FINISHED" <<<"${state}"; then
    echo "station transport order mission finished"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "station transport order failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "station transport order did not finish; seen_order=${seen_order}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
