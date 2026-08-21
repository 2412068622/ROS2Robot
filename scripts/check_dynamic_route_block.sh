#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

LOG_DIR="$(mktemp -d /tmp/robot_dynamic_route_block.XXXXXX)"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  rm -rf "${LOG_DIR}"
}
trap cleanup EXIT

start_bg() {
  local name="$1"
  shift
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  return_to_dock_on_low_battery:=false

wait_service_type /v2/block_station_route "robot_interfaces_mission/srv/BlockStationRoute" 20 \
  "${LOG_DIR}/wait_block.err"
wait_service_type /v2/unblock_station_route "robot_interfaces_mission/srv/UnblockStationRoute" 20 \
  "${LOG_DIR}/wait_unblock.err"
wait_service_type /v2/list_traffic_reservations "robot_interfaces_mission/srv/ListTrafficReservations" 20 \
  "${LOG_DIR}/wait_list_traffic.err"
wait_service_type /v2/clear_traffic_reservation "robot_interfaces_mission/srv/ClearTrafficReservation" 20 \
  "${LOG_DIR}/wait_clear_traffic.err"
wait_submit_order 20 "${LOG_DIR}/wait_submit.err"
wait_service_type /v2/cancel_queued_mission "robot_interfaces_mission/srv/CancelQueuedMission" 20 \
  "${LOG_DIR}/wait_cancel.err"

ros2 service call /v2/block_station_route robot_interfaces_mission/srv/BlockStationRoute \
  "{from_station: receiving, to_station: storage_a, reason: aisle maintenance}" \
  | rg "success=True|resource_id='route_edge:receiving__storage_a'"

ros2 service call /v2/list_traffic_reservations robot_interfaces_mission/srv/ListTrafficReservations "{}" \
  | rg "success=True|route_edge:receiving__storage_a|traffic_block:aisle_maintenance|blocked"

call_submit_order blocked_route_alpha station_transport 10 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  | rg "accepted=False|route resource locked|traffic_block:aisle_maintenance"

ros2 service call /v2/unblock_station_route robot_interfaces_mission/srv/UnblockStationRoute \
  "{from_station: receiving, to_station: storage_a}" \
  | rg "success=True|resource_id='route_edge:receiving__storage_a'"

call_submit_order clear_route_alpha station_transport 10 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  | rg "accepted=True|station_order_clear_route_alpha|queue_size=1"

ros2 service call /v2/list_traffic_reservations robot_interfaces_mission/srv/ListTrafficReservations "{}" \
  | rg "success=True|route_edge:receiving__storage_a|station_order_clear_route_alpha|mission"

ros2 service call /v2/clear_traffic_reservation robot_interfaces_mission/srv/ClearTrafficReservation \
  "{resource_id: 'route_edge:receiving__storage_a'}" \
  | rg "success=True|owner_id='station_order_clear_route_alpha'"

ros2 service call /v2/cancel_queued_mission robot_interfaces_mission/srv/CancelQueuedMission \
  "{mission_id: station_order_clear_route_alpha, cancel_active: false}" \
  | rg "success=True|queue_size=0"

echo "dynamic route block passed"
