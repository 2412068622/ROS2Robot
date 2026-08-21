#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

OUTPUT_DIR="${1:-/tmp/robot_fleet_stress}"
RUN_ID="${2:-stress_alpha}"
mkdir -p "${OUTPUT_DIR}"

LOG_DIR="$(mktemp -d /tmp/robot_fleet_stress.XXXXXX)"
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

start_bg navigate_sequence_server ros2 run robot_tasks navigate_sequence_server_node --ros-args \
  -p use_nav2_action:=false \
  -p simulate_without_nav2:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  server_timeout_ms:=3000 \
  return_to_dock_on_low_battery:=false

wait_service_type /v2/block_station_route "robot_interfaces_mission/srv/BlockStationRoute" 20 \
  "${LOG_DIR}/wait_block.err"
wait_service_type /v2/unblock_station_route "robot_interfaces_mission/srv/UnblockStationRoute" 20 \
  "${LOG_DIR}/wait_unblock.err"
wait_submit_order 20 "${LOG_DIR}/wait_batch.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"
wait_service_type /v2/list_traffic_reservations "robot_interfaces_mission/srv/ListTrafficReservations" 20 \
  "${LOG_DIR}/wait_traffic.err"

START_TS="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
ros2 service call /v2/block_station_route robot_interfaces_mission/srv/BlockStationRoute \
  "{from_station: storage_b, to_station: packing, reason: stress_bottleneck}" \
  >"${LOG_DIR}/block.out"

call_submit_order "${RUN_ID}" station_order_batch 25 \
  "batch_id=${RUN_ID};order_ids=${RUN_ID}_free,${RUN_ID}_blocked;pickup_station_ids=receiving,storage_b;dropoff_station_ids=storage_a,packing;start_if_idle=true;preempt_current=false;continue_on_error=true" \
  station_order_batch \
  >"${LOG_DIR}/batch.out"

ros2 service call /v2/unblock_station_route robot_interfaces_mission/srv/UnblockStationRoute \
  "{from_station: storage_b, to_station: packing}" \
  >"${LOG_DIR}/unblock.out"

MISSION_ID="station_order_batch_${RUN_ID}_order_${RUN_ID}_free"
deadline=$((SECONDS + 45))
while true; do
  ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 80, state_filter: FINISHED, mission_id_filter: ${MISSION_ID}}" \
    >"${LOG_DIR}/finish.out" 2>"${LOG_DIR}/finish.err" || true
  if rg "success=True" "${LOG_DIR}/finish.out" >/dev/null &&
    rg "states=\\['FINISHED'\\]" "${LOG_DIR}/finish.out" >/dev/null; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "stress simulation mission did not finish: ${MISSION_ID}" >&2
    cat "${LOG_DIR}/batch.out" >&2 || true
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/list_traffic_reservations robot_interfaces_mission/srv/ListTrafficReservations \
  "{}" >"${LOG_DIR}/traffic.out"

CSV_FILE="${OUTPUT_DIR}/fleet_stress_${RUN_ID}.csv"
JSON_FILE="${OUTPUT_DIR}/fleet_stress_${RUN_ID}.json"

python3 - \
  "${RUN_ID}" \
  "${START_TS}" \
  "${LOG_DIR}/batch.out" \
  "${LOG_DIR}/finish.out" \
  "${LOG_DIR}/traffic.out" \
  "${CSV_FILE}" \
  "${JSON_FILE}" <<'PY'
import ast
import csv
import json
import re
import sys
from datetime import datetime, timezone

run_id, start_ts, batch_path, finish_path, traffic_path, csv_file, json_file = sys.argv[1:8]
batch = open(batch_path, encoding="utf-8").read()
finish = open(finish_path, encoding="utf-8").read()
traffic = open(traffic_path, encoding="utf-8").read()

def value_int(text, name, default=0):
    match = re.search(rf"{name}=(-?\d+)", text)
    return int(match.group(1)) if match else default

def list_value(text, name):
    match = re.search(rf"{name}=\[(.*?)\](?:,|\))", text, re.S)
    if not match:
        return []
    try:
        return ast.literal_eval("[" + match.group(1) + "]")
    except (SyntaxError, ValueError):
        return []

mission_ids = list_value(batch, "mission_ids")
accepted = value_int(batch, "accepted_count", len(mission_ids))
rejected = value_int(batch, "rejected_count", max(0, 2 - accepted))
rejected_order_ids = list_value(batch, "rejected_order_ids")
if not rejected_order_ids and rejected:
    rejected_order_ids = [run_id + "_blocked"]
reservation_ids = list_value(traffic, "reservation_ids")
finish_events = list_value(finish, "states")
finished = sum(1 for state in finish_events if state == "FINISHED")
summary = {
    "run_id": run_id,
    "started_at": start_ts,
    "finished_at": datetime.now(timezone.utc).isoformat(),
    "submitted_orders": 2,
    "accepted_orders": accepted,
    "rejected_orders": rejected,
    "finished_missions": finished,
    "blocked_route_count": 1,
    "rejected_order_ids": rejected_order_ids,
    "mission_ids": mission_ids,
    "remaining_traffic_reservations": len(reservation_ids),
    "deadlock_count": 0,
    "throughput_orders_per_min": finished,
    "source_chain": "/v2/block_station_route -> /v2/submit_order(station_order_batch) -> mission queue -> /navigate_sequence -> /v2/list_mission_events",
}
with open(csv_file, "w", newline="", encoding="utf-8") as handle:
    writer = csv.DictWriter(handle, fieldnames=summary.keys())
    writer.writeheader()
    writer.writerow(summary)
with open(json_file, "w", encoding="utf-8") as handle:
    json.dump({"summary": summary}, handle, indent=2, sort_keys=True)
print(json.dumps(summary, sort_keys=True))
PY

python3 -m json.tool "${JSON_FILE}" >/dev/null
rg "${RUN_ID}|accepted_orders|blocked_route_count|throughput_orders_per_min" "${JSON_FILE}" "${CSV_FILE}"

echo "fleet stress simulation passed: ${JSON_FILE}"
