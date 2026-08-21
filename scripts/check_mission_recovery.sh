#!/usr/bin/env bash
set -euo pipefail

deadline=$((SECONDS + 15))
while ! ros2 service type /start_mission_profile >/tmp/robot_recovery_service_type.txt 2>/dev/null; do
  if (( SECONDS >= deadline )); then
    echo "mission recovery services did not become ready" >&2
    exit 1
  fi
  sleep 1
done

ros2 service type /start_mission_profile | rg "std_srvs/srv/Trigger"
ros2 service type /v2/get_mission_recovery_state | rg "robot_interfaces_mission/srv/GetMissionRecoveryState"
ros2 service type /reset_mission_recovery_state | rg "std_srvs/srv/Trigger"
ros2 topic type /mission_runner/state | rg "robot_interfaces/msg/RobotState"

ros2 service call /reset_mission_recovery_state std_srvs/srv/Trigger "{}" | rg "success=True|success: true"
ros2 service call /start_mission_profile std_srvs/srv/Trigger "{}" | rg "success=True|success: true"

deadline=$((SECONDS + 30))
last_state=""
while true; do
  last_state="$(ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState --field state 2>/tmp/robot_mission_recovery_state.err || true)"
  if grep -q "NEEDS_OPERATOR" <<<"${last_state}"; then
    echo "${last_state}"
    break
  fi
  if grep -q "FINISHED" <<<"${last_state}"; then
    echo "mission unexpectedly finished during recovery test" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "mission recovery did not reach NEEDS_OPERATOR before timeout; last state: ${last_state}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/get_mission_recovery_state robot_interfaces_mission/srv/GetMissionRecoveryState "{}" | tee /tmp/robot_mission_recovery_response.txt
rg "retry_count=1|retry_count: 1" /tmp/robot_mission_recovery_response.txt
rg "last_recovery_action='manual'|last_recovery_action: manual" /tmp/robot_mission_recovery_response.txt
