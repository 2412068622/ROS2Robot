#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

PORT="${1:-18082}"
POLICY="${ROOT_DIR}/src/robot_tasks/config/operators/rest_role_policy.json"
LOG_DIR="$(mktemp -d /tmp/robot_rest_workflow_controls.XXXXXX)"
AUDIT_LOG="$(mktemp /tmp/robot_rest_workflow_controls.XXXXXX.jsonl)"
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
wait_service_type /v2/list_business_order_types "robot_interfaces_business/srv/ListBusinessOrderTypes" 20 "${LOG_DIR}/wait_business_types.err"
wait_service_type /v2/pause_workflow "robot_interfaces_business/srv/PauseWorkflow" 20 "${LOG_DIR}/wait_pause.err"
wait_service_type /v2/resume_workflow "robot_interfaces_business/srv/ResumeWorkflow" 20 "${LOG_DIR}/wait_resume.err"
wait_service_type /v2/get_workflow_status "robot_interfaces_business/srv/GetWorkflowStatus" 20 "${LOG_DIR}/wait_status.err"
wait_service_type /v2/reprioritize_queued_mission "robot_interfaces_mission/srv/ReprioritizeQueuedMission" 20 "${LOG_DIR}/wait_reprio.err"
wait_service_type /v2/reprioritize_batch "robot_interfaces_mission/srv/ReprioritizeBatch" 20 "${LOG_DIR}/wait_reprio_batch.err"
wait_service_type /v2/update_fleet_robot_state "robot_interfaces_fleet/srv/UpdateFleetRobotState" 20 "${LOG_DIR}/wait_update_robot.err"
wait_service_type /v2/update_remote_task_state "robot_interfaces_fleet/srv/UpdateRemoteTaskState" 20 "${LOG_DIR}/wait_remote_task.err"
wait_service_type /v2/get_operator_snapshot "robot_interfaces_business/srv/GetOperatorSnapshot" 20 "${LOG_DIR}/wait_snapshot.err"

ros2 service call /v2/update_fleet_robot_state robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_1, enabled: true, state: busy, battery_voltage: 24.0, current_station_id: dock, update_enabled: true, update_state: true, update_battery_voltage: true, update_current_station_id: true}" \
  | rg "success=True"
ros2 service call /v2/update_fleet_robot_state robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_2, enabled: true, state: available, battery_voltage: 24.2, current_station_id: receiving, update_enabled: true, update_state: true, update_battery_voltage: true, update_current_station_id: true}" \
  | rg "success=True"
call_submit_order remote_rest_control fleet_station 20 \
  "task_id=remote_rest_control;required_capability=station_transport;pickup_station=receiving;dropoff_station=storage_a;start_if_idle=true;preempt_current=false" \
  fleet_station_task \
  | rg "accepted=True|remote_fleet_task_remote_rest_control"

python3 - "http://127.0.0.1:${PORT}" "${AUDIT_LOG}" <<'PY'
import json
import sys
import time
import urllib.parse
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

def request(method, path, payload=None, query=None, operator_id="bob"):
    url = base + path
    if query:
        url += "?" + urllib.parse.urlencode(query)
    body = json.dumps(payload or {}).encode("utf-8") if method == "POST" else None
    req = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json", "X-Operator-Id": operator_id},
        method=method,
    )
    with urllib.request.urlopen(req, timeout=20) as response:
        return json.loads(response.read().decode("utf-8"))

wait_health()

submit = request("POST", "/api/business_orders", {
    "business_order_id": "rest_control",
    "business_type": "campus_mail",
    "priority": 10,
    "start_if_idle": False,
    "preempt_current": False,
})
assert submit["success"] and submit["mission_ids"], submit
order_id = "business_rest_control"

status = request(
    "GET", "/api/workflows/status", query={"order_id": order_id}, operator_id="alice")
assert status["success"] and status["order_id"] == order_id, status
assert status["queued_steps"] >= 2 and not status["paused"], status
queued_mission_ids = status["queued_mission_ids"]
assert len(queued_mission_ids) >= 2, status

paused = request("POST", "/api/workflows/pause", {
    "order_id": order_id,
    "pause_active": False,
})
assert paused["success"] and paused["affected_mission_ids"], paused

status = request(
    "GET", "/api/workflows/status", query={"order_id": order_id}, operator_id="alice")
assert status["success"] and status["paused"], status
queued_mission_ids = status["queued_mission_ids"]
assert len(queued_mission_ids) >= 2, status

single = request("POST", "/api/missions/reprioritize", {
    "mission_id": queued_mission_ids[-1],
    "priority": 99,
})
assert single["success"] and single["queue_size"] >= 2, single

batch = request("POST", "/api/missions/reprioritize_batch", {
    "mission_ids": queued_mission_ids[:2],
    "priorities": [77, 76],
})
assert batch["success"], batch
assert set(queued_mission_ids[:2]).issubset(set(batch["updated_mission_ids"])), batch

resumed = request("POST", "/api/workflows/resume", {
    "order_id": order_id,
    "resume_active": False,
})
assert resumed["success"] and resumed["affected_mission_ids"], resumed

remote = request("POST", "/api/remote_tasks/state", {
    "task_id": "remote_rest_control",
    "state": "FINISHED",
    "message": "remote robot completed through REST",
})
assert remote["success"] and remote["state"] == "FINISHED", remote

snapshot = request(
    "GET", "/api/operator/snapshot", query={"event_limit": 20}, operator_id="alice")
assert snapshot["success"], snapshot
assert "campus_mail" in snapshot["business_types"], snapshot
assert any(task["task_id"] == "remote_rest_control" and task["state"] == "FINISHED"
           for task in snapshot["remote_tasks"]), snapshot

records = [json.loads(line) for line in open(audit_log, encoding="utf-8") if line.strip()]
assert any(record["path"] == "/api/workflows/pause" and record["allowed"]
           for record in records), records
assert any(record["path"] == "/api/operator/snapshot" and record["operator_id"] == "alice"
           for record in records), records

print("REST_WORKFLOW_STATUS=" + json.dumps(status, sort_keys=True))
print("REST_REPRIORITIZE_BATCH=" + json.dumps(batch, sort_keys=True))
print("REST_REMOTE_STATE=" + json.dumps(remote, sort_keys=True))
print("REST_SNAPSHOT_REMOTE_TASKS=" + json.dumps(snapshot["remote_tasks"], sort_keys=True))
PY

echo "rest workflow controls passed"
