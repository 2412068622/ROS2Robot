#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_mission_events_service.err
wait_service_type /start_mission_profile "std_srvs/srv/Trigger" 10 /tmp/robot_mission_events_start.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_mission_events_state_topic.err

ros2 service call /start_mission_profile std_srvs/srv/Trigger "{}" | rg "success=True"

seen_demo=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_mission_events_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_mission_events_state.err || true
  )"

  if grep -q "demo_patrol" <<<"${active_goal}"; then
    seen_demo=true
  fi
  if ${seen_demo} && grep -q "FINISHED" <<<"${state}"; then
    break
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "mission event source mission failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "mission event source mission did not finish; seen_demo=${seen_demo}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: '', mission_id_filter: demo_patrol}" \
  | rg "success=True|demo_patrol|FINISHED|STARTING"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 5, state_filter: FINISHED, mission_id_filter: demo_patrol}" \
  | rg "success=True|states=\\['FINISHED'\\]|demo_patrol"

echo "mission event audit workflow passed"
