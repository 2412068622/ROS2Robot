#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_picking_wave.XXXXXX)"
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

FULFILLMENT_ID="picking_wave_pick_alpha"

start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  auto_start_queue:=false \
  return_to_dock_on_low_battery:=false

wait_submit_order 20 "${LOG_DIR}/wait_picking.err"
wait_service_type /v2/confirm_pick "robot_interfaces_business/srv/ConfirmPick" 20 \
  "${LOG_DIR}/wait_confirm_pick.err"
wait_service_type /v2/confirm_dropoff "robot_interfaces_business/srv/ConfirmDropoff" 20 \
  "${LOG_DIR}/wait_confirm_dropoff.err"
wait_service_type /v2/report_short_pick "robot_interfaces_business/srv/ReportShortPick" 20 \
  "${LOG_DIR}/wait_short_pick.err"
wait_service_type /v2/get_order_fulfillment_status "robot_interfaces_business/srv/GetOrderFulfillmentStatus" 20 \
  "${LOG_DIR}/wait_status.err"
wait_service_type /v2/get_pending_confirmations "robot_interfaces_facility/srv/GetPendingConfirmations" 20 \
  "${LOG_DIR}/wait_pending.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

call_submit_order pick_alpha picking_wave 40 \
  "wave_id=pick_alpha;sku_id=sku_42;tote_id=tote_7;pick_station=receiving;dropoff_station=packing;quantity=10;start_if_idle=false" \
  picking_wave \
  >"${LOG_DIR}/submit.out"
assert_contains "${LOG_DIR}/submit.out" "accepted=True"
assert_contains "${LOG_DIR}/submit.out" "${FULFILLMENT_ID}"
assert_contains "${LOG_DIR}/submit.out" "picking_wave queued"

ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: receiving}" >"${LOG_DIR}/pending_pick.out"
assert_contains "${LOG_DIR}/pending_pick.out" "${FULFILLMENT_ID}_pick"
assert_contains "${LOG_DIR}/pending_pick.out" "pick_sku_42"
assert_contains "${LOG_DIR}/pending_pick.out" "pick 10 sku_42 into tote_7"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: FULFILLMENT_ROUTE_QUEUED, mission_id_filter: ${FULFILLMENT_ID}}" \
  >"${LOG_DIR}/queued_event.out"
assert_contains "${LOG_DIR}/queued_event.out" "success=True"
assert_contains "${LOG_DIR}/queued_event.out" "${FULFILLMENT_ID}"
assert_contains "${LOG_DIR}/queued_event.out" "picking_wave queued from receiving to packing"

ros2 service call /v2/confirm_pick robot_interfaces_business/srv/ConfirmPick \
  "{fulfillment_id: ${FULFILLMENT_ID}, operator_id: alice, picked_quantity: 8, note: two missing}" \
  >"${LOG_DIR}/confirm_pick.out"
assert_contains "${LOG_DIR}/confirm_pick.out" "success=True"
assert_contains "${LOG_DIR}/confirm_pick.out" "state='SHORT_PICK'"
assert_contains "${LOG_DIR}/confirm_pick.out" "picked_quantity=8"
assert_contains "${LOG_DIR}/confirm_pick.out" "short_quantity=2"

ros2 service call /v2/report_short_pick robot_interfaces_business/srv/ReportShortPick \
  "{fulfillment_id: ${FULFILLMENT_ID}, operator_id: alice, short_quantity: 2, reason: bin short}" \
  >"${LOG_DIR}/short_pick.out"
assert_contains "${LOG_DIR}/short_pick.out" "success=True"
assert_contains "${LOG_DIR}/short_pick.out" "state='SHORT_PICK'"
assert_contains "${LOG_DIR}/short_pick.out" "short_quantity=2"

ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: packing}" >"${LOG_DIR}/pending_dropoff.out"
assert_contains "${LOG_DIR}/pending_dropoff.out" "${FULFILLMENT_ID}_dropoff"
assert_contains "${LOG_DIR}/pending_dropoff.out" "dropoff_sku_42"
assert_contains "${LOG_DIR}/pending_dropoff.out" "drop off tote_7 at packing"

ros2 service call /v2/confirm_dropoff robot_interfaces_business/srv/ConfirmDropoff \
  "{fulfillment_id: ${FULFILLMENT_ID}, operator_id: bob, note: tote received}" \
  >"${LOG_DIR}/dropoff.out"
assert_contains "${LOG_DIR}/dropoff.out" "success=True"
assert_contains "${LOG_DIR}/dropoff.out" "state='FULFILLED_WITH_SHORT_PICK'"

ros2 service call /v2/get_order_fulfillment_status robot_interfaces_business/srv/GetOrderFulfillmentStatus \
  "{fulfillment_id: ${FULFILLMENT_ID}}" >"${LOG_DIR}/status.out"
assert_contains "${LOG_DIR}/status.out" "success=True"
assert_contains "${LOG_DIR}/status.out" "order_type='picking_wave'"
assert_contains "${LOG_DIR}/status.out" "sku_id='sku_42'"
assert_contains "${LOG_DIR}/status.out" "tote_id='tote_7'"
assert_contains "${LOG_DIR}/status.out" "state='FULFILLED_WITH_SHORT_PICK'"
assert_contains "${LOG_DIR}/status.out" "pick_station='receiving'"
assert_contains "${LOG_DIR}/status.out" "dropoff_station='packing'"
assert_contains "${LOG_DIR}/status.out" "requested_quantity=10"
assert_contains "${LOG_DIR}/status.out" "picked_quantity=8"
assert_contains "${LOG_DIR}/status.out" "short_quantity=2"
assert_contains "${LOG_DIR}/status.out" "${FULFILLMENT_ID}_pick"
assert_contains "${LOG_DIR}/status.out" "${FULFILLMENT_ID}_dropoff"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: FULFILLMENT_SHORT_PICK, mission_id_filter: ${FULFILLMENT_ID}}" \
  >"${LOG_DIR}/short_event.out"
assert_contains "${LOG_DIR}/short_event.out" "success=True"
assert_contains "${LOG_DIR}/short_event.out" "FULFILLMENT_SHORT_PICK"
assert_contains "${LOG_DIR}/short_event.out" "short pick 2 reported by alice"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: FULFILLMENT_DROPOFF_CONFIRMED, mission_id_filter: ${FULFILLMENT_ID}}" \
  >"${LOG_DIR}/dropoff_event.out"
assert_contains "${LOG_DIR}/dropoff_event.out" "success=True"
assert_contains "${LOG_DIR}/dropoff_event.out" "FULFILLMENT_DROPOFF_CONFIRMED"
assert_contains "${LOG_DIR}/dropoff_event.out" "tote received"

echo "picking wave passed"
