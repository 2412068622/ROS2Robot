#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/submit_order "robot_interfaces_business/srv/SubmitOrder" 10 /tmp/robot_vda5050_order.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_vda5050_state_topic.err

ros2 service call /v2/submit_order robot_interfaces_business/srv/SubmitOrder \
  "{order_id: vda_alpha, order_type: vda5050, priority: 55, payload_json: 'serial_number=robot_1;pickup_node_id=receiving;dropoff_node_id=storage_a;start_if_idle=true;preempt_current=false', tags: [vda5050_adapter]}" \
  | rg "accepted=True|station_order_vda5050_vda_alpha"

seen_order=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_vda5050_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_vda5050_state.err || true
  )"

  if grep -q "station_order_vda5050_vda_alpha" <<<"${active_goal}"; then
    seen_order=true
  fi
  if ${seen_order} && grep -q "FINISHED" <<<"${state}"; then
    echo "vda5050 station order adapter workflow passed"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "vda5050 order failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "vda5050 order did not finish; seen_order=${seen_order}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
