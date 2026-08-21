#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_intersection_lock.XXXXXX)"
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
  return_to_dock_on_low_battery:=false \
  traffic_intersection_locks_enabled:=true

wait_submit_order 20 "${LOG_DIR}/wait_submit.err"
wait_service_type /v2/list_traffic_reservations "robot_interfaces_mission/srv/ListTrafficReservations" 20 \
  "${LOG_DIR}/wait_list_traffic.err"
wait_service_type /v2/cancel_queued_mission "robot_interfaces_mission/srv/CancelQueuedMission" 20 \
  "${LOG_DIR}/wait_cancel.err"

call_submit_order intersection_alpha station_transport 10 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  >"${LOG_DIR}/intersection_alpha.out"
rg "accepted=True" "${LOG_DIR}/intersection_alpha.out"
rg "station_order_intersection_alpha" "${LOG_DIR}/intersection_alpha.out"
rg "queue_size=1" "${LOG_DIR}/intersection_alpha.out"

ros2 service call /v2/list_traffic_reservations robot_interfaces_mission/srv/ListTrafficReservations "{}" \
  >"${LOG_DIR}/traffic.out"
rg "success=True" "${LOG_DIR}/traffic.out"
rg "route_node:storage_a" "${LOG_DIR}/traffic.out"
rg "station_order_intersection_alpha" "${LOG_DIR}/traffic.out"
rg "mission" "${LOG_DIR}/traffic.out"

call_submit_order intersection_beta station_transport 10 \
  "pickup_station=storage_a;dropoff_station=packing;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  >"${LOG_DIR}/intersection_beta.out"
rg "accepted=False" "${LOG_DIR}/intersection_beta.out"
rg "route resource locked: route_node:storage_a by station_order_intersection_alpha" \
  "${LOG_DIR}/intersection_beta.out"

ros2 service call /v2/cancel_queued_mission robot_interfaces_mission/srv/CancelQueuedMission \
  "{mission_id: station_order_intersection_alpha, cancel_active: false}" \
  >"${LOG_DIR}/cancel.out"
rg "success=True" "${LOG_DIR}/cancel.out"
rg "queue_size=0" "${LOG_DIR}/cancel.out"

call_submit_order intersection_beta_after_clear station_transport 10 \
  "pickup_station=storage_a;dropoff_station=packing;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  >"${LOG_DIR}/intersection_beta_after_clear.out"
rg "accepted=True" "${LOG_DIR}/intersection_beta_after_clear.out"
rg "station_order_intersection_beta_after_clear" "${LOG_DIR}/intersection_beta_after_clear.out"
rg "queue_size=1" "${LOG_DIR}/intersection_beta_after_clear.out"

echo "intersection lock passed"
