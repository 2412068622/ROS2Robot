#!/usr/bin/env bash
set -euo pipefail

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT

low="${tmp_dir}/low_priority.yaml"
high="${tmp_dir}/high_priority.yaml"

cat >"${low}" <<'YAML'
mission_id: low_priority
frame_id: map
loop: false
goals:
  - x: 0.0
    y: 0.0
    yaw: 0.0
  - x: 0.1
    y: 0.0
    yaw: 0.0
YAML

cat >"${high}" <<'YAML'
mission_id: high_priority
frame_id: map
loop: false
goals:
  - x: 1.0
    y: 0.0
    yaw: 0.0
  - x: 1.1
    y: 0.0
    yaw: 0.0
  - x: 1.2
    y: 0.0
    yaw: 0.0
YAML

ros2 service type /v2/enqueue_mission_profile | rg "robot_interfaces_mission/srv/EnqueueMission"
ros2 topic type /mission_runner/state | rg "robot_interfaces/msg/RobotState"

ros2 service call /v2/enqueue_mission_profile robot_interfaces_mission/srv/EnqueueMission \
  "{mission_file: \"${low}\", priority: 1, start_if_idle: false}" | rg "success=True"
ros2 service call /v2/enqueue_mission_profile robot_interfaces_mission/srv/EnqueueMission \
  "{mission_file: \"${high}\", priority: 10, start_if_idle: true}" | rg "success=True"

seen_high=false
seen_low=false
deadline=$((SECONDS + 25))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_mission_queue_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_mission_queue_state.err || true
  )"

  if grep -q "high_priority" <<<"${active_goal}"; then
    seen_high=true
  fi
  if ${seen_high} && grep -q "low_priority" <<<"${active_goal}"; then
    seen_low=true
  fi
  if ${seen_high} && ${seen_low} && grep -q "FINISHED" <<<"${state}"; then
    echo "high_priority ran before low_priority and queue finished"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "mission queue failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "mission queue did not finish; seen_high=${seen_high}, seen_low=${seen_low}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done
