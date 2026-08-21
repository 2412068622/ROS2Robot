#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_replenishment_order.XXXXXX)"
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

assert_contains() {
  local file="$1"
  local pattern="$2"
  rg "${pattern}" "${file}"
}

FULFILLMENT_ID="replenishment_repl_alpha"

start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  auto_start_queue:=false \
  return_to_dock_on_low_battery:=false

wait_submit_order 20 "${LOG_DIR}/wait_replenishment.err"
wait_service_type /v2/confirm_pick "robot_interfaces_business/srv/ConfirmPick" 20 \
  "${LOG_DIR}/wait_confirm_pick.err"
wait_service_type /v2/confirm_dropoff "robot_interfaces_business/srv/ConfirmDropoff" 20 \
  "${LOG_DIR}/wait_confirm_dropoff.err"
wait_service_type /v2/get_order_fulfillment_status "robot_interfaces_business/srv/GetOrderFulfillmentStatus" 20 \
  "${LOG_DIR}/wait_status.err"
wait_service_type /v2/get_pending_confirmations "robot_interfaces_facility/srv/GetPendingConfirmations" 20 \
  "${LOG_DIR}/wait_pending.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

call_submit_order repl_alpha replenishment 35 \
  "order_id=repl_alpha;sku_id=bolts;tote_id=tote_line_1;source_station=storage_a;line_station=packing;quantity=4;start_if_idle=false" \
  replenishment_order \
  >"${LOG_DIR}/submit.out"
assert_contains "${LOG_DIR}/submit.out" "accepted=True"
assert_contains "${LOG_DIR}/submit.out" "${FULFILLMENT_ID}"
assert_contains "${LOG_DIR}/submit.out" "replenishment queued"

ros2 service call /v2/get_order_fulfillment_status robot_interfaces_business/srv/GetOrderFulfillmentStatus \
  "{fulfillment_id: ${FULFILLMENT_ID}}" >"${LOG_DIR}/queued_status.out"
assert_contains "${LOG_DIR}/queued_status.out" "success=True"
assert_contains "${LOG_DIR}/queued_status.out" "order_type='replenishment'"
assert_contains "${LOG_DIR}/queued_status.out" "sku_id='bolts'"
assert_contains "${LOG_DIR}/queued_status.out" "tote_id='tote_line_1'"
assert_contains "${LOG_DIR}/queued_status.out" "state='AWAITING_PICK'"
assert_contains "${LOG_DIR}/queued_status.out" "pick_station='storage_a'"
assert_contains "${LOG_DIR}/queued_status.out" "dropoff_station='packing'"
assert_contains "${LOG_DIR}/queued_status.out" "requested_quantity=4"
assert_contains "${LOG_DIR}/queued_status.out" "${FULFILLMENT_ID}_pick"
assert_contains "${LOG_DIR}/queued_status.out" "${FULFILLMENT_ID}_dropoff"

ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: storage_a}" >"${LOG_DIR}/pending_source.out"
assert_contains "${LOG_DIR}/pending_source.out" "${FULFILLMENT_ID}_pick"
assert_contains "${LOG_DIR}/pending_source.out" "pick_bolts"
assert_contains "${LOG_DIR}/pending_source.out" "pick 4 bolts into tote_line_1"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: FULFILLMENT_ROUTE_QUEUED, mission_id_filter: ${FULFILLMENT_ID}}" \
  >"${LOG_DIR}/queued_event.out"
assert_contains "${LOG_DIR}/queued_event.out" "success=True"
assert_contains "${LOG_DIR}/queued_event.out" "${FULFILLMENT_ID}"
assert_contains "${LOG_DIR}/queued_event.out" "replenishment queued from storage_a to packing"

ros2 service call /v2/confirm_pick robot_interfaces_business/srv/ConfirmPick \
  "{fulfillment_id: ${FULFILLMENT_ID}, operator_id: chen, picked_quantity: 4, note: tote loaded}" \
  >"${LOG_DIR}/confirm_pick.out"
assert_contains "${LOG_DIR}/confirm_pick.out" "success=True"
assert_contains "${LOG_DIR}/confirm_pick.out" "state='PICK_CONFIRMED'"
assert_contains "${LOG_DIR}/confirm_pick.out" "picked_quantity=4"
assert_contains "${LOG_DIR}/confirm_pick.out" "short_quantity=0"

ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: packing}" >"${LOG_DIR}/pending_line.out"
assert_contains "${LOG_DIR}/pending_line.out" "${FULFILLMENT_ID}_dropoff"
assert_contains "${LOG_DIR}/pending_line.out" "dropoff_bolts"
assert_contains "${LOG_DIR}/pending_line.out" "drop off tote_line_1 at packing"

ros2 service call /v2/confirm_dropoff robot_interfaces_business/srv/ConfirmDropoff \
  "{fulfillment_id: ${FULFILLMENT_ID}, operator_id: lin, note: line replenished}" \
  >"${LOG_DIR}/dropoff.out"
assert_contains "${LOG_DIR}/dropoff.out" "success=True"
assert_contains "${LOG_DIR}/dropoff.out" "state='FULFILLED'"

ros2 service call /v2/get_order_fulfillment_status robot_interfaces_business/srv/GetOrderFulfillmentStatus \
  "{fulfillment_id: ${FULFILLMENT_ID}}" >"${LOG_DIR}/final_status.out"
assert_contains "${LOG_DIR}/final_status.out" "success=True"
assert_contains "${LOG_DIR}/final_status.out" "order_type='replenishment'"
assert_contains "${LOG_DIR}/final_status.out" "state='FULFILLED'"
assert_contains "${LOG_DIR}/final_status.out" "requested_quantity=4"
assert_contains "${LOG_DIR}/final_status.out" "picked_quantity=4"
assert_contains "${LOG_DIR}/final_status.out" "short_quantity=0"
assert_contains "${LOG_DIR}/final_status.out" "${FULFILLMENT_ID}_pick"
assert_contains "${LOG_DIR}/final_status.out" "${FULFILLMENT_ID}_dropoff"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: FULFILLMENT_PICK_CONFIRMED, mission_id_filter: ${FULFILLMENT_ID}}" \
  >"${LOG_DIR}/pick_event.out"
assert_contains "${LOG_DIR}/pick_event.out" "success=True"
assert_contains "${LOG_DIR}/pick_event.out" "FULFILLMENT_PICK_CONFIRMED"
assert_contains "${LOG_DIR}/pick_event.out" "picked 4 of 4"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: FULFILLMENT_DROPOFF_CONFIRMED, mission_id_filter: ${FULFILLMENT_ID}}" \
  >"${LOG_DIR}/dropoff_event.out"
assert_contains "${LOG_DIR}/dropoff_event.out" "success=True"
assert_contains "${LOG_DIR}/dropoff_event.out" "FULFILLMENT_DROPOFF_CONFIRMED"
assert_contains "${LOG_DIR}/dropoff_event.out" "line replenished"

echo "replenishment order passed"
