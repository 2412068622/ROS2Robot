#!/usr/bin/env bash
set -euo pipefail

ros2 service type /start_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /pause_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /resume_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /cancel_mission_profile | rg "std_srvs/srv/Trigger"

ros2 service call /start_mission_profile std_srvs/srv/Trigger {} | rg "success=True"
sleep 1
ros2 service call /pause_mission_profile std_srvs/srv/Trigger {} | rg "success=True"

deadline=$((SECONDS + 8))
while true; do
  state="$(ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState --field state 2>/tmp/robot_mission_control_state.err || true)"
  if grep -q "PAUSED" <<<"${state}"; then
    echo "${state}"
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "mission runner did not enter PAUSED before timeout; last state: ${state}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /resume_mission_profile std_srvs/srv/Trigger {} | rg "success=True"
sleep 1
ros2 service call /cancel_mission_profile std_srvs/srv/Trigger {} | rg "success=True"

deadline=$((SECONDS + 8))
while true; do
  state="$(ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState --field state 2>/tmp/robot_mission_control_state.err || true)"
  if grep -q "CANCELED\\|CANCELING" <<<"${state}"; then
    echo "${state}"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "mission runner failed with state: ${state}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "mission runner did not cancel before timeout; last state: ${state}" >&2
    exit 1
  fi
  sleep 1
done
