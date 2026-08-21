#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

REST_PORT="${1:-18083}"
WEBHOOK_PORT="${2:-18084}"
LOG_DIR="$(mktemp -d /tmp/robot_external_callbacks.XXXXXX)"
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
  auto_start_queue:=false \
  return_to_dock_on_low_battery:=false
start_bg rest_gateway python3 "${SCRIPT_DIR}/rest_api_gateway.py" \
  --host 127.0.0.1 \
  --port "${REST_PORT}" \
  --idempotency-store "${LOG_DIR}/idempotency_store.json"

wait_service_type /v2/submit_order "robot_interfaces_business/srv/SubmitOrder" 20 \
  "${LOG_DIR}/wait_business.err"
wait_service_type /v2/list_business_order_types "robot_interfaces_business/srv/ListBusinessOrderTypes" 20 \
  "${LOG_DIR}/wait_business_types.err"
wait_service_type /v2/request_station_confirmation "robot_interfaces_facility/srv/RequestStationConfirmation" 20 \
  "${LOG_DIR}/wait_confirmation.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

python3 - "http://127.0.0.1:${REST_PORT}" <<'PY'
import json
import sys
import time
import urllib.request

base = sys.argv[1]

deadline = time.time() + 20
while time.time() < deadline:
    try:
        with urllib.request.urlopen(base + "/health", timeout=2) as response:
            if json.loads(response.read().decode("utf-8")).get("success"):
                break
    except Exception:
        time.sleep(0.5)
else:
    raise SystemExit("REST gateway did not become healthy")

def post_once():
    body = json.dumps({
        "business_order_id": "idem_alpha",
        "business_type": "campus_mail",
        "priority": 10,
        "start_if_idle": False,
        "preempt_current": False,
        "external_correlation_id": "wms-42",
    }).encode("utf-8")
    request = urllib.request.Request(
        base + "/api/business_orders",
        data=body,
        headers={
            "Content-Type": "application/json",
            "Idempotency-Key": "idem-key-alpha",
        },
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.loads(response.read().decode("utf-8"))

first = post_once()
second = post_once()
assert first["success"], first
assert second["success"], second
assert second.get("idempotent_replay") is True, second
assert first["mission_ids"] == second["mission_ids"], (first, second)
assert second["external_correlation_id"] == "wms-42", second
print("IDEMPOTENT_FIRST=" + json.dumps(first, sort_keys=True))
print("IDEMPOTENT_SECOND=" + json.dumps(second, sort_keys=True))
PY

ros2 service call /v2/request_station_confirmation robot_interfaces_facility/srv/RequestStationConfirmation \
  "{confirmation_id: webhook_alpha, station_id: receiving, action: external_callback_probe, mission_id: webhook_probe, operator_hint: callback test, timeout_sec: 0, timeout_policy: manual}" \
  >"${LOG_DIR}/confirmation.out"
rg "success=True" "${LOG_DIR}/confirmation.out"

cat >"${LOG_DIR}/mock_receiver.py" <<'PY'
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

port, received_path, fail_once_path = int(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        with received_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, sort_keys=True) + "\n")
        if fail_once_path.exists():
            fail_once_path.unlink()
            self.send_response(503)
            self.end_headers()
            self.wfile.write(b"temporary failure")
            return
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"ok")

ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
PY

touch "${LOG_DIR}/fail_once"
start_bg webhook_receiver python3 "${LOG_DIR}/mock_receiver.py" \
  "${WEBHOOK_PORT}" "${LOG_DIR}/received.jsonl" "${LOG_DIR}/fail_once"
sleep 1

python3 "${SCRIPT_DIR}/external_callback_outbox.py" \
  --outbox "${LOG_DIR}/outbox.json" \
  --endpoint "http://127.0.0.1:${WEBHOOK_PORT}/callbacks" \
  --state-filter CONFIRMATION_PENDING \
  --mission-id-filter station_confirmation_webhook_alpha \
  >"${LOG_DIR}/outbox_failed.json"
python3 -m json.tool "${LOG_DIR}/outbox_failed.json" >/dev/null
rg '"status": "FAILED"' "${LOG_DIR}/outbox_failed.json"

python3 "${SCRIPT_DIR}/external_callback_outbox.py" \
  --outbox "${LOG_DIR}/outbox.json" \
  --endpoint "http://127.0.0.1:${WEBHOOK_PORT}/callbacks" \
  --state-filter CONFIRMATION_PENDING \
  --mission-id-filter station_confirmation_webhook_alpha \
  --replay-failed \
  >"${LOG_DIR}/outbox_sent.json"
python3 -m json.tool "${LOG_DIR}/outbox_sent.json" >/dev/null
rg '"status": "SENT"' "${LOG_DIR}/outbox_sent.json"
rg "station_confirmation_webhook_alpha" "${LOG_DIR}/received.jsonl"

echo "external callbacks and idempotency passed"
