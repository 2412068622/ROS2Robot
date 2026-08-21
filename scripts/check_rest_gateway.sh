#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

PORT="${1:-18080}"
LOG_DIR="$(mktemp -d /tmp/robot_rest_gateway.XXXXXX)"
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
start_bg rest_gateway python3 "${SCRIPT_DIR}/rest_api_gateway.py" --host 127.0.0.1 --port "${PORT}"

wait_service_type /v2/submit_order "robot_interfaces_business/srv/SubmitOrder" 20 "${LOG_DIR}/wait_business.err"
wait_service_type /v2/list_business_order_types "robot_interfaces_business/srv/ListBusinessOrderTypes" 20 "${LOG_DIR}/wait_business_types.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 "${LOG_DIR}/wait_events.err"

python3 - "http://127.0.0.1:${PORT}/health" <<'PY'
import json
import sys
import time
import urllib.request

url = sys.argv[1]
deadline = time.time() + 20
while time.time() < deadline:
    try:
        with urllib.request.urlopen(url, timeout=2) as response:
            data = json.loads(response.read().decode("utf-8"))
        if data.get("success"):
            raise SystemExit(0)
    except Exception:
        time.sleep(0.5)
raise SystemExit("REST gateway did not become healthy")
PY

python3 - "http://127.0.0.1:${PORT}" <<'PY'
import json
import sys
import time
import urllib.parse
import urllib.request

base = sys.argv[1]

def post(path, payload):
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        base + path,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.loads(response.read().decode("utf-8"))

def get(path, query=None):
    url = base + path
    if query:
        url += "?" + urllib.parse.urlencode(query)
    with urllib.request.urlopen(url, timeout=20) as response:
        return json.loads(response.read().decode("utf-8"))

submit = post("/api/business_orders", {
    "business_order_id": "rest_alpha",
    "business_type": "campus_mail",
    "priority": 0,
    "start_if_idle": True,
    "preempt_current": False,
})
assert submit["success"], submit
assert submit["routed_scenario_id"] == "campus", submit
assert submit["routed_workflow_id"] == "full_mail_route", submit

mission_id = "station_sequence_business_rest_alpha_step_1_leg_4"
deadline = time.time() + 80
events = []
while time.time() < deadline:
    payload = get("/api/missions/events", {
        "limit": 80,
        "state_filter": "FINISHED",
        "mission_id_filter": mission_id,
    })
    events = payload.get("events", [])
    if payload.get("success") and any(event.get("mission_id") == mission_id for event in events):
        break
    time.sleep(1.0)
else:
    raise SystemExit(f"mission did not finish via REST events: {mission_id}, events={events}")

robots = get("/api/fleet/robots", {"include_disabled": "true"})
assert robots["success"] and robots["robots"], robots
resources = get("/api/facilities/resources", {"include_disabled": "true"})
assert resources["success"] and resources["resources"], resources

print("REST_SUBMIT=" + json.dumps(submit, sort_keys=True))
print("REST_EVENTS=" + json.dumps(events, sort_keys=True))
print("REST_ROBOTS=" + str(len(robots["robots"])))
print("REST_RESOURCES=" + str(len(resources["resources"])))
PY

echo "rest gateway passed"
