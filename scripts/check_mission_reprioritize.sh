#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT

first="${tmp_dir}/reprio_first.yaml"
second="${tmp_dir}/reprio_second.yaml"

cat >"${first}" <<'YAML'
mission_id: reprio_first
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

cat >"${second}" <<'YAML'
mission_id: reprio_second
frame_id: map
loop: false
goals:
  - x: 0.2
    y: 0.0
    yaw: 0.0
  - x: 0.3
    y: 0.0
    yaw: 0.0
YAML

wait_service_type /v2/enqueue_mission_profile "robot_interfaces_mission/srv/EnqueueMission" 10 /tmp/robot_reprio_enqueue.err
wait_service_type /v2/reprioritize_queued_mission "robot_interfaces_mission/srv/ReprioritizeQueuedMission" 10 /tmp/robot_reprio_single.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_reprio_events.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_reprio_state_topic.err

ros2 service call /v2/enqueue_mission_profile robot_interfaces_mission/srv/EnqueueMission \
  "{mission_file: \"${first}\", priority: 5, start_if_idle: false}" | rg "success=True"
ros2 service call /v2/enqueue_mission_profile robot_interfaces_mission/srv/EnqueueMission \
  "{mission_file: \"${second}\", priority: 1, start_if_idle: false}" | rg "success=True"

ros2 service call /v2/reprioritize_queued_mission robot_interfaces_mission/srv/ReprioritizeQueuedMission \
  "{mission_id: reprio_second, priority: 20}" \
  | rg "success=True|reprio_second|20|queue_size=2"

ros2 service call /start_mission_profile std_srvs/srv/Trigger "{}" | rg "success=True"

seen_second=false
seen_first=false
deadline=$((SECONDS + 30))
while true; do
  active_goal="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id 2>/tmp/robot_reprio_state.err || true
  )"
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_reprio_state.err || true
  )"
  if grep -q "reprio_second" <<<"${active_goal}"; then
    seen_second=true
  fi
  if ${seen_second} && grep -q "reprio_first" <<<"${active_goal}"; then
    seen_first=true
  fi
  if ${seen_second} && ${seen_first} && grep -q "FINISHED" <<<"${state}"; then
    break
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "reprioritize queue failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "reprioritize queue did not finish; seen_second=${seen_second}, seen_first=${seen_first}, state=${state}, active_goal_id=${active_goal}" >&2
    exit 1
  fi
  sleep 1
done

echo "mission reprioritize passed"
