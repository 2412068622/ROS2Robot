#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/estimate_station_sequence "robot_interfaces_mission/srv/EstimateStationSequence" 10 /tmp/robot_station_sequence_estimate.err
wait_submit_order 10 /tmp/robot_station_sequence_submit.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_station_sequence_events.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_station_sequence_state_topic.err

ros2 service call /v2/estimate_station_sequence robot_interfaces_mission/srv/EstimateStationSequence \
  "{station_ids: [receiving, storage_a, packing, dock], nominal_speed_mps: 0.5, battery_voltage: 24.5}" \
  | rg "success=True|estimate_station_sequence|waypoint_count=4|distance_m=3.9|battery_sufficient=True"

call_submit_order sequence_alpha station_sequence 35 \
  "station_ids=receiving,storage_a,packing,dock;start_if_idle=true;preempt_current=false" \
  station_sequence \
  | rg "accepted=True|station_sequence_sequence_alpha_leg_1|station_sequence_sequence_alpha_leg_2|station_sequence_sequence_alpha_leg_3"

seen_first=false
seen_second=false
seen_third=false
deadline=$((SECONDS + 45))
while true; do
  state="$(
    ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state 2>/tmp/robot_station_sequence_state.err || true
  )"

  if ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 50, state_filter: FINISHED, mission_id_filter: station_sequence_sequence_alpha_leg_1}" \
    2>/tmp/robot_station_sequence_events.err | rg "loaded 1 mission event|states=\\['FINISHED'\\]" >/dev/null; then
    seen_first=true
  fi
  if ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 50, state_filter: FINISHED, mission_id_filter: station_sequence_sequence_alpha_leg_2}" \
    2>/tmp/robot_station_sequence_events.err | rg "loaded 1 mission event|states=\\['FINISHED'\\]" >/dev/null; then
    seen_second=true
  fi
  if ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 50, state_filter: FINISHED, mission_id_filter: station_sequence_sequence_alpha_leg_3}" \
    2>/tmp/robot_station_sequence_events.err | rg "loaded 1 mission event|states=\\['FINISHED'\\]" >/dev/null; then
    seen_third=true
  fi
  if ${seen_first} && ${seen_second} && ${seen_third} && grep -q "FINISHED" <<<"${state}"; then
    echo "station sequence task passed"
    exit 0
  fi
  if grep -q "FAILED\\|ERROR" <<<"${state}"; then
    echo "station sequence task failed with state: ${state}, active_goal_id: ${active_goal}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "station sequence task did not finish; seen_first=${seen_first}, seen_second=${seen_second}, seen_third=${seen_third}, state=${state}" >&2
    exit 1
  fi
  sleep 1
done
