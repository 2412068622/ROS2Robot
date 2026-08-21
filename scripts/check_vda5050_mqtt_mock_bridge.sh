#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_vda5050_mqtt_mock.XXXXXX)"
BROKER_DIR="${LOG_DIR}/broker"
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

mkdir -p "${BROKER_DIR}/orders"
cat >"${BROKER_DIR}/orders/mock_order_alpha.json" <<'JSON'
{
  "order_id": "mock_mqtt_alpha",
  "serial_number": "robot_1",
  "pickup_node_id": "receiving",
  "dropoff_node_id": "storage_a",
  "priority": 65,
  "start_if_idle": true,
  "preempt_current": false
}
JSON

start_bg navigate_sequence_server ros2 run robot_tasks navigate_sequence_server_node --ros-args \
  -p use_nav2_action:=false \
  -p simulate_without_nav2:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  server_timeout_ms:=3000 \
  return_to_dock_on_low_battery:=false

wait_service_type /v2/submit_order "robot_interfaces_business/srv/SubmitOrder" 20 \
  "${LOG_DIR}/wait_vda.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

python3 "${SCRIPT_DIR}/vda5050_mqtt_mock_bridge.py" \
  --mock-broker-dir "${BROKER_DIR}" \
  --timeout-sec 45

STATE_FILE="${BROKER_DIR}/states/mock_mqtt_alpha.state.json"
CONNECTION_FILE="${BROKER_DIR}/connections/robot_1.connection.json"
python3 -m json.tool "${STATE_FILE}" >/dev/null
python3 -m json.tool "${CONNECTION_FILE}" >/dev/null
rg '"orderId": "mock_mqtt_alpha"' "${STATE_FILE}"
rg '"orderState": "FINISHED"' "${STATE_FILE}"
rg '"nodeStates"' "${STATE_FILE}"
rg '"edgeStates"' "${STATE_FILE}"
rg '"batteryState"' "${STATE_FILE}"
rg '"connectionState": "ONLINE"' "${CONNECTION_FILE}"

echo "vda5050 mqtt mock bridge passed"
