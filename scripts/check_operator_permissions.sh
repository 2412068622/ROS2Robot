#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

PORT="${1:-18081}"
POLICY="${ROOT_DIR}/src/robot_tasks/config/operators/rest_role_policy.json"
AUDIT_LOG="$(mktemp /tmp/robot_rest_audit.XXXXXX.jsonl)"
LOG_DIR="$(mktemp -d /tmp/robot_operator_permissions.XXXXXX)"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  rm -rf "${LOG_DIR}" "${AUDIT_LOG}"
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
start_bg rest_gateway python3 "${SCRIPT_DIR}/rest_api_gateway.py" \
  --host 127.0.0.1 \
  --port "${PORT}" \
  --role-policy "${POLICY}" \
  --enforce-roles \
  --audit-log "${AUDIT_LOG}"

wait_submit_order 20 "${LOG_DIR}/wait_business.err"
wait_service_type /v2/cancel_workflow "robot_interfaces_business/srv/CancelWorkflow" 20 "${LOG_DIR}/wait_cancel.err"

python3 - "http://127.0.0.1:${PORT}" "${AUDIT_LOG}" <<'PY'
import json
import sys
import time
import urllib.error
import urllib.request

base, audit_log = sys.argv[1:3]

def wait_health():
    deadline = time.time() + 20
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(base + "/health", timeout=2) as response:
                if json.loads(response.read().decode("utf-8")).get("success"):
                    return
        except Exception:
            time.sleep(0.5)
    raise SystemExit("REST gateway did not become healthy")

def post(path, payload, operator_id):
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        base + path,
        data=body,
        headers={"Content-Type": "application/json", "X-Operator-Id": operator_id},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))

wait_health()

status, submit = post("/api/business_orders", {
    "business_order_id": "perm_cancel",
    "business_type": "campus_mail",
    "priority": 0,
    "start_if_idle": False,
    "preempt_current": False,
}, "alice")
assert status == 200 and submit["success"], submit

status, denied = post("/api/workflows/cancel", {
    "order_id": "business_perm_cancel",
    "cancel_active": False,
}, "alice")
assert status == 403 and not denied["success"], denied

status, canceled = post("/api/workflows/cancel", {
    "order_id": "business_perm_cancel",
    "cancel_active": False,
}, "bob")
assert status == 200 and canceled["success"], canceled
assert "station_sequence_business_perm_cancel_step_1_leg_1" in str(canceled), canceled

records = [json.loads(line) for line in open(audit_log, encoding="utf-8") if line.strip()]
assert any(record["operator_id"] == "alice" and record["path"] == "/api/business_orders" and record["allowed"] for record in records), records
assert any(record["operator_id"] == "alice" and record["path"] == "/api/workflows/cancel" and not record["allowed"] for record in records), records
assert any(record["operator_id"] == "bob" and record["path"] == "/api/workflows/cancel" and record["allowed"] for record in records), records

print("PERMISSION_DENIED=" + json.dumps(denied, sort_keys=True))
print("PERMISSION_CANCELED=" + json.dumps(canceled, sort_keys=True))
print("AUDIT_RECORDS=" + str(len(records)))
PY

echo "operator permissions passed"
