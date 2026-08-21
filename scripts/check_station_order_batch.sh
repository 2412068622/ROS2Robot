#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_submit_order 10 /tmp/robot_station_batch_submit.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_station_batch_events.err
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 10 /tmp/robot_station_batch_state_topic.err

call_submit_order wms_alpha station_order_batch 50 \
  "batch_id=wms_alpha;order_ids=one,two,three;pickup_station_ids=receiving,storage_a,packing;dropoff_station_ids=storage_a,packing,dock;start_if_idle=true;preempt_current=false;continue_on_error=false" \
  station_order_batch \
  | rg "accepted=True|station_order_batch_wms_alpha_order_one|station_order_batch_wms_alpha_order_two|station_order_batch_wms_alpha_order_three"

for mission in \
  station_order_batch_wms_alpha_order_one \
  station_order_batch_wms_alpha_order_two \
  station_order_batch_wms_alpha_order_three; do
  deadline=$((SECONDS + 45))
  while true; do
    if ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 80, state_filter: FINISHED, mission_id_filter: ${mission}}" \
      2>/tmp/robot_station_batch_events.err | rg "loaded 1 mission event|states=\\['FINISHED'\\]" >/dev/null; then
      break
    fi
    if (( SECONDS >= deadline )); then
      echo "station order batch mission did not finish: ${mission}" >&2
      exit 1
    fi
    sleep 1
  done
done

echo "station order batch passed"
