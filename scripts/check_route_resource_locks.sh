#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_submit_order 10 /tmp/robot_route_lock_submit.err
wait_service_type /v2/cancel_queued_mission "robot_interfaces_mission/srv/CancelQueuedMission" 10 /tmp/robot_route_lock_cancel.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_route_lock_events.err

call_submit_order route_lock_a station_transport 10 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  | rg "accepted=True|station_order_route_lock_a|queue_size=1"

call_submit_order route_lock_b station_transport 10 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  | rg "accepted=False|route resource locked|route_edge:receiving__storage_a|station_order_route_lock_a"

ros2 service call /v2/cancel_queued_mission robot_interfaces_mission/srv/CancelQueuedMission \
  "{mission_id: station_order_route_lock_a, cancel_active: false}" \
  | rg "success=True|removed queued mission station_order_route_lock_a|queue_size=0"

call_submit_order route_lock_b station_transport 10 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=true;preempt_current=false" \
  station_transport_order \
  | rg "accepted=True|station_order_route_lock_b"

deadline=$((SECONDS + 35))
while true; do
  if ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 80, state_filter: FINISHED, mission_id_filter: station_order_route_lock_b}" \
    2>/tmp/robot_route_lock_events.err | rg "loaded 1 mission event|states=\\['FINISHED'\\]" >/dev/null; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "route lock mission did not finish after release" >&2
    exit 1
  fi
  sleep 1
done

echo "route resource locks passed"
