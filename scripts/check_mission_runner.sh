#!/usr/bin/env bash
set -euo pipefail

ros2 service type /start_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /reload_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /pause_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /resume_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /cancel_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /v2/enqueue_mission_profile | rg "robot_interfaces_mission/srv/EnqueueMission"
ros2 topic type /mission_runner/state | rg "robot_interfaces/msg/RobotState"

deadline=$((SECONDS + 20))
while true; do
  state="$(ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState --field state 2>/tmp/robot_mission_runner_state.err || true)"
  if grep -q "FINISHED" <<<"${state}"; then
    echo "${state}"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "mission runner failed with state: ${state}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "mission runner did not finish before timeout; last state: ${state}" >&2
    exit 1
  fi
  sleep 1
done
