#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /return_to_dock "std_srvs/srv/Trigger" 10 /tmp/robot_return_to_dock_service.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_return_to_dock_topic.err

ros2 service call /return_to_dock std_srvs/srv/Trigger "{}" | rg "success=True"

seen_dock=false
deadline=$((SECONDS + 20))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_return_to_dock_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_return_to_dock_state.err || true
  )"

  if grep -q "dock_return" <<<"${active_goal}"; then
    seen_dock=true
  fi
  if ${seen_dock} && grep -q "FINISHED" <<<"${state}"; then
    echo "return-to-dock mission finished"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "return-to-dock failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "return-to-dock did not finish; seen_dock=${seen_dock}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
