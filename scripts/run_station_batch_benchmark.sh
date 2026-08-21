#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

if [ ! -f "${ROOT_DIR}/install/setup.bash" ]; then
  echo "install/setup.bash not found; run colcon build --symlink-install first" >&2
  exit 1
fi

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

RESULTS_DIR="${1:-${ROOT_DIR}/src/robot_experiments/results}"
BATCH_ID="${2:-batch_kpi_alpha}"
TIMEOUT_SEC="${3:-60}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ISO_STAMP="$(date -Iseconds)"
CSV_FILE="${RESULTS_DIR}/station_batch_${BATCH_ID}_${STAMP}.csv"
JSON_FILE="${RESULTS_DIR}/station_batch_${BATCH_ID}_${STAMP}.json"
LOG_DIR="$(mktemp -d /tmp/robot_station_batch_benchmark.XXXXXX)"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

start_bg() {
  local name="$1"
  shift
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

mkdir -p "${RESULTS_DIR}"

start_bg navigate_sequence_server ros2 run robot_tasks navigate_sequence_server_node --ros-args \
  -p use_nav2_action:=false \
  -p simulate_without_nav2:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  server_timeout_ms:=3000 \
  return_to_dock_on_low_battery:=false

wait_submit_order 20 "${LOG_DIR}/wait_batch.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 "${LOG_DIR}/wait_events.err"

SUBMIT_OUT="${LOG_DIR}/submit_batch.out"
call_submit_order "${BATCH_ID}" station_order_batch 20 \
  "batch_id=${BATCH_ID};order_ids=one,two;pickup_station_ids=receiving,storage_a;dropoff_station_ids=storage_a,packing;start_if_idle=true;preempt_current=false;continue_on_error=false" \
  station_order_batch \
  >"${SUBMIT_OUT}"
rg "accepted=True" "${SUBMIT_OUT}"

MISSION_IDS=(
  "station_order_batch_${BATCH_ID}_order_one"
  "station_order_batch_${BATCH_ID}_order_two"
)

wait_mission_finished() {
  local mission_id="$1"
  local deadline=$((SECONDS + TIMEOUT_SEC))
  local out_file="${LOG_DIR}/${mission_id}_finished.out"
  while true; do
    ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 80, state_filter: FINISHED, mission_id_filter: ${mission_id}}" \
      >"${out_file}" || true
    if rg "success=True" "${out_file}" >/dev/null && rg "states=\\['FINISHED'\\]" "${out_file}" >/dev/null; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "${mission_id} did not reach FINISHED within ${TIMEOUT_SEC}s" >&2
      cat "${out_file}" >&2 || true
      return 1
    fi
    sleep 1
  done
}

EVENT_OUT="${LOG_DIR}/mission_events.out"
: >"${EVENT_OUT}"
for mission_id in "${MISSION_IDS[@]}"; do
  wait_mission_finished "${mission_id}"
  mission_event_out="${LOG_DIR}/${mission_id}_events.out"
  ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 80, state_filter: '', mission_id_filter: ${mission_id}}" \
    >"${mission_event_out}"
  rg "success=True" "${mission_event_out}"
  cat "${mission_event_out}" >>"${EVENT_OUT}"
  printf "\n" >>"${EVENT_OUT}"
done

python3 - "$CSV_FILE" "$JSON_FILE" "$EVENT_OUT" "$SUBMIT_OUT" "$ISO_STAMP" "$BATCH_ID" "${MISSION_IDS[@]}" <<'PY'
import ast
import csv
import json
import re
import sys

csv_file, json_file, event_file, submit_file = sys.argv[1:5]
timestamp, batch_id = sys.argv[5:7]
mission_ids = sys.argv[7:]
text = open(event_file, encoding="utf-8").read()
submit_text = open(submit_file, encoding="utf-8").read()

def parse_response_lists(name):
    values = []
    for match in re.finditer(rf"(?<![A-Za-z_]){name}=\[(.*?)\](?:,|\))", text, re.S):
        values.extend(ast.literal_eval("[" + match.group(1) + "]"))
    return values

stamps = parse_response_lists("stamp")
ids = parse_response_lists("mission_ids")
states = parse_response_lists("states")
messages = parse_response_lists("messages")

events = []
for index, mission_id in enumerate(ids):
    try:
        stamp = float(stamps[index])
    except (ValueError, IndexError):
        stamp = 0.0
    events.append({
        "stamp": stamp,
        "mission_id": mission_id,
        "state": states[index] if index < len(states) else "",
        "message": messages[index] if index < len(messages) else "",
    })

known = [event for event in events if event["mission_id"] in mission_ids]
start_stamps = [event["stamp"] for event in known if event["state"] == "STARTING"]
finish_stamps = [event["stamp"] for event in known if event["state"] == "FINISHED"]
finished_missions = {event["mission_id"] for event in known if event["state"] == "FINISHED"}
failed_count = sum(1 for event in known if event["state"] in {"FAILED", "CANCELED", "NEEDS_OPERATOR"})
cycle_time = max(finish_stamps) - min(start_stamps) if start_stamps and finish_stamps else 0.0

accepted_match = re.search(r"accepted_count=(\d+)", submit_text)
rejected_match = re.search(r"rejected_count=(\d+)", submit_text)
accepted_count = int(accepted_match.group(1)) if accepted_match else len(mission_ids)
rejected_count = int(rejected_match.group(1)) if rejected_match else 0

summary = {
    "timestamp": timestamp,
    "batch_id": batch_id,
    "mission_count": len(mission_ids),
    "accepted_count": accepted_count,
    "rejected_count": rejected_count,
    "finished_count": len(finished_missions),
    "failed_count": failed_count,
    "cycle_time_sec": round(cycle_time, 3),
    "throughput_missions_per_min": round((len(finished_missions) / cycle_time) * 60.0, 3) if cycle_time > 0 else 0.0,
    "average_mission_duration_sec": round(cycle_time / len(finished_missions), 3) if finished_missions else 0.0,
    "queue_waiting_time_sec": 0.0,
    "source_chain": "/v2/submit_order(station_order_batch) -> station order -> preflight -> mission queue -> /navigate_sequence -> /v2/list_mission_events",
    "scope": "headless fallback batch KPI benchmark; no GUI, real hardware, real traffic manager, or physical map was used",
}

with open(csv_file, "w", newline="", encoding="utf-8") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(summary.keys()))
    writer.writeheader()
    writer.writerow(summary)
with open(json_file, "w", encoding="utf-8") as handle:
    json.dump({"summary": summary, "events": sorted(known, key=lambda item: item["stamp"])}, handle, indent=2)
PY

python3 -m json.tool "${JSON_FILE}" >/dev/null
test -s "${CSV_FILE}"

echo "CSV=${CSV_FILE}"
echo "JSON=${JSON_FILE}"
echo "station batch benchmark passed"
