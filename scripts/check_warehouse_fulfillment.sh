#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_warehouse_fulfillment.XXXXXX)"
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

start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  auto_start_queue:=false \
  return_to_dock_on_low_battery:=false

wait_submit_order 20 "${LOG_DIR}/wait_submit_order.err"
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

PICK_OUT="${LOG_DIR}/picking.out"
call_submit_order pick_alpha picking_wave 40 \
  "wave_id=pick_alpha;sku_id=sku_42;tote_id=tote_7;pick_station=receiving;dropoff_station=packing;quantity=10;start_if_idle=false" \
  picking_wave \
  >"${PICK_OUT}"
assert_contains "${PICK_OUT}" "accepted=True"
assert_contains "${PICK_OUT}" "picking_wave_pick_alpha"
assert_contains "${PICK_OUT}" "picking_wave queued"

PENDING_OUT="${LOG_DIR}/pending.out"
ros2 service call /v2/get_pending_confirmations robot_interfaces_facility/srv/GetPendingConfirmations \
  "{station_id_filter: receiving}" >"${PENDING_OUT}"
assert_contains "${PENDING_OUT}" "picking_wave_pick_alpha_pick"
assert_contains "${PENDING_OUT}" "pick_sku_42"

EARLY_DROPOFF_OUT="${LOG_DIR}/early_dropoff.out"
ros2 service call /v2/confirm_dropoff robot_interfaces_business/srv/ConfirmDropoff \
  "{fulfillment_id: picking_wave_pick_alpha, operator_id: bob, note: too early}" \
  >"${EARLY_DROPOFF_OUT}"
assert_contains "${EARLY_DROPOFF_OUT}" "success=False"
assert_contains "${EARLY_DROPOFF_OUT}" "state='AWAITING_PICK'"
assert_contains "${EARLY_DROPOFF_OUT}" "dropoff is not ready"

CONFIRM_PICK_OUT="${LOG_DIR}/confirm_pick.out"
ros2 service call /v2/confirm_pick robot_interfaces_business/srv/ConfirmPick \
  "{fulfillment_id: picking_wave_pick_alpha, operator_id: alice, picked_quantity: 8, note: two missing}" \
  >"${CONFIRM_PICK_OUT}"
assert_contains "${CONFIRM_PICK_OUT}" "success=True"
assert_contains "${CONFIRM_PICK_OUT}" "state='SHORT_PICK'"
assert_contains "${CONFIRM_PICK_OUT}" "short_quantity=2"

SHORT_OUT="${LOG_DIR}/short_pick.out"
ros2 service call /v2/report_short_pick robot_interfaces_business/srv/ReportShortPick \
  "{fulfillment_id: picking_wave_pick_alpha, operator_id: alice, short_quantity: 2, reason: bin short}" \
  >"${SHORT_OUT}"
assert_contains "${SHORT_OUT}" "success=False"
assert_contains "${SHORT_OUT}" "state='SHORT_PICK'"
assert_contains "${SHORT_OUT}" "short pick is not pending"

DROPOFF_OUT="${LOG_DIR}/dropoff.out"
ros2 service call /v2/confirm_dropoff robot_interfaces_business/srv/ConfirmDropoff \
  "{fulfillment_id: picking_wave_pick_alpha, operator_id: bob, note: tote received}" \
  >"${DROPOFF_OUT}"
assert_contains "${DROPOFF_OUT}" "success=True"
assert_contains "${DROPOFF_OUT}" "state='FULFILLED_WITH_SHORT_PICK'"

STATUS_OUT="${LOG_DIR}/status.out"
ros2 service call /v2/get_order_fulfillment_status robot_interfaces_business/srv/GetOrderFulfillmentStatus \
  "{fulfillment_id: picking_wave_pick_alpha}" >"${STATUS_OUT}"
assert_contains "${STATUS_OUT}" "success=True"
assert_contains "${STATUS_OUT}" "order_type='picking_wave'"
assert_contains "${STATUS_OUT}" "picked_quantity=8"
assert_contains "${STATUS_OUT}" "short_quantity=2"
assert_contains "${STATUS_OUT}" "picking_wave_pick_alpha_pick"
assert_contains "${STATUS_OUT}" "picking_wave_pick_alpha_dropoff"

EVENT_OUT="${LOG_DIR}/events.out"
ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: FULFILLMENT_DROPOFF_CONFIRMED, mission_id_filter: picking_wave_pick_alpha}" \
  >"${EVENT_OUT}"
assert_contains "${EVENT_OUT}" "success=True"
assert_contains "${EVENT_OUT}" "FULFILLMENT_DROPOFF_CONFIRMED"

REPLENISH_OUT="${LOG_DIR}/replenishment.out"
call_submit_order repl_alpha replenishment 35 \
  "order_id=repl_alpha;sku_id=bolts;tote_id=tote_line_1;source_station=storage_a;line_station=packing;quantity=4;start_if_idle=false" \
  replenishment_order \
  >"${REPLENISH_OUT}"
assert_contains "${REPLENISH_OUT}" "accepted=True"
assert_contains "${REPLENISH_OUT}" "replenishment_repl_alpha"

REPLENISH_SHORT_OUT="${LOG_DIR}/replenishment_short_pick.out"
ros2 service call /v2/report_short_pick robot_interfaces_business/srv/ReportShortPick \
  "{fulfillment_id: replenishment_repl_alpha, operator_id: carol, short_quantity: 1, reason: one tote missing}" \
  >"${REPLENISH_SHORT_OUT}"
assert_contains "${REPLENISH_SHORT_OUT}" "success=True"
assert_contains "${REPLENISH_SHORT_OUT}" "state='SHORT_PICK'"
assert_contains "${REPLENISH_SHORT_OUT}" "short_quantity=1"

REPLENISH_DROPOFF_OUT="${LOG_DIR}/replenishment_dropoff.out"
ros2 service call /v2/confirm_dropoff robot_interfaces_business/srv/ConfirmDropoff \
  "{fulfillment_id: replenishment_repl_alpha, operator_id: dave, note: line received short}" \
  >"${REPLENISH_DROPOFF_OUT}"
assert_contains "${REPLENISH_DROPOFF_OUT}" "success=True"
assert_contains "${REPLENISH_DROPOFF_OUT}" "state='FULFILLED_WITH_SHORT_PICK'"

MILK_OUT="${LOG_DIR}/milk_run.out"
call_submit_order milk_alpha milk_run 30 \
  "run_id=milk_alpha;tote_id=cart_1;station_ids=receiving,storage_a,packing;start_if_idle=false" \
  milk_run \
  >"${MILK_OUT}"
assert_contains "${MILK_OUT}" "accepted=True"
assert_contains "${MILK_OUT}" "milk_run_milk_alpha"
assert_contains "${MILK_OUT}" "milk run queued"

MILK_STATUS_OUT="${LOG_DIR}/milk_status.out"
ros2 service call /v2/get_order_fulfillment_status robot_interfaces_business/srv/GetOrderFulfillmentStatus \
  "{fulfillment_id: milk_run_milk_alpha}" >"${MILK_STATUS_OUT}"
assert_contains "${MILK_STATUS_OUT}" "success=True"
assert_contains "${MILK_STATUS_OUT}" "mission_ids=\['station_sequence_milk_run_milk_alpha_leg_1', 'station_sequence_milk_run_milk_alpha_leg_2'\]"
assert_contains "${MILK_STATUS_OUT}" "confirmation_ids=\['milk_run_milk_alpha_receiving_stop', 'milk_run_milk_alpha_storage_a_stop', 'milk_run_milk_alpha_packing_stop'\]"

DUP_MILK_OUT="${LOG_DIR}/milk_run_duplicate.out"
call_submit_order milk_alpha_retry milk_run 30 \
  "run_id=milk_alpha;tote_id=cart_1;station_ids=receiving,storage_a,packing;start_if_idle=false" \
  milk_run \
  >"${DUP_MILK_OUT}"
assert_contains "${DUP_MILK_OUT}" "accepted=False"
assert_contains "${DUP_MILK_OUT}" "fulfillment already exists: milk_run_milk_alpha"

MILK_STATUS_AFTER_DUP_OUT="${LOG_DIR}/milk_status_after_duplicate.out"
ros2 service call /v2/get_order_fulfillment_status robot_interfaces_business/srv/GetOrderFulfillmentStatus \
  "{fulfillment_id: milk_run_milk_alpha}" >"${MILK_STATUS_AFTER_DUP_OUT}"
assert_contains "${MILK_STATUS_AFTER_DUP_OUT}" "success=True"
assert_contains "${MILK_STATUS_AFTER_DUP_OUT}" "mission_ids=\['station_sequence_milk_run_milk_alpha_leg_1', 'station_sequence_milk_run_milk_alpha_leg_2'\]"
assert_contains "${MILK_STATUS_AFTER_DUP_OUT}" "confirmation_ids=\['milk_run_milk_alpha_receiving_stop', 'milk_run_milk_alpha_storage_a_stop', 'milk_run_milk_alpha_packing_stop'\]"

MILK_PICK_OUT="${LOG_DIR}/milk_pick_rejected.out"
ros2 service call /v2/confirm_pick robot_interfaces_business/srv/ConfirmPick \
  "{fulfillment_id: milk_run_milk_alpha, operator_id: alice, picked_quantity: 1, note: not a pick order}" \
  >"${MILK_PICK_OUT}"
assert_contains "${MILK_PICK_OUT}" "success=False"
assert_contains "${MILK_PICK_OUT}" "state='REJECTED'"
assert_contains "${MILK_PICK_OUT}" "does not support pick confirmation"

echo "warehouse fulfillment passed"
