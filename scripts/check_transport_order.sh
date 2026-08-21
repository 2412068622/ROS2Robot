#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_submit_order 10 /tmp/robot_transport_order_service.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_transport_order_topic.err

call_submit_order order_alpha transport 20 \
  "frame_id=map;pickup_x=0.2;pickup_y=0.0;pickup_yaw=0.0;dropoff_x=0.8;dropoff_y=0.4;dropoff_yaw=1.57;start_if_idle=true" \
  transport_order \
  | rg "accepted=True|transport_order_order_alpha"

seen_order=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_transport_order_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_transport_order_state.err || true
  )"

  if grep -q "transport_order_order_alpha" <<<"${active_goal}"; then
    seen_order=true
  fi
  if ${seen_order} && grep -q "FINISHED" <<<"${state}"; then
    echo "transport order mission finished"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "transport order failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "transport order did not finish; seen_order=${seen_order}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
