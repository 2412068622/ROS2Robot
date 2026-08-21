#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_traffic_deadlock.XXXXXX)"
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
  traffic_intersection_locks_enabled:=false

wait_submit_order 20 "${LOG_DIR}/wait_submit.err"
wait_service_type /v2/detect_traffic_deadlock "robot_interfaces_mission/srv/DetectTrafficDeadlock" 20 \
  "${LOG_DIR}/wait_detect.err"

call_submit_order deadlock_alpha station_transport 10 \
  "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  >"${LOG_DIR}/deadlock_alpha.out"
rg "accepted=True" "${LOG_DIR}/deadlock_alpha.out"
rg "station_order_deadlock_alpha" "${LOG_DIR}/deadlock_alpha.out"
rg "queue_size=1" "${LOG_DIR}/deadlock_alpha.out"

call_submit_order deadlock_beta station_transport 10 \
  "pickup_station=storage_a;dropoff_station=packing;start_if_idle=false;preempt_current=false" \
  station_transport_order \
  >"${LOG_DIR}/deadlock_beta.out"
rg "accepted=True" "${LOG_DIR}/deadlock_beta.out"
rg "station_order_deadlock_beta" "${LOG_DIR}/deadlock_beta.out"
rg "queue_size=2" "${LOG_DIR}/deadlock_beta.out"

ros2 service call /v2/detect_traffic_deadlock robot_interfaces_mission/srv/DetectTrafficDeadlock "{}" \
  >"${LOG_DIR}/deadlock_detect.out"
rg "success=True" "${LOG_DIR}/deadlock_detect.out"
rg "deadlocked=True" "${LOG_DIR}/deadlock_detect.out"
rg "station_order_deadlock_alpha" "${LOG_DIR}/deadlock_detect.out"
rg "station_order_deadlock_beta" "${LOG_DIR}/deadlock_detect.out"
rg "storage_a" "${LOG_DIR}/deadlock_detect.out"

echo "traffic deadlock detection passed"
