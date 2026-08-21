#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

tmp_dir="$(mktemp -d)"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

start_bg() {
  local name="$1"
  shift
  "$@" >"${tmp_dir}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

order_json="${tmp_dir}/vda_order.json"
state_json="${tmp_dir}/vda_state.json"

cat >"${order_json}" <<'JSON'
{
  "order_id": "bridge_alpha",
  "serial_number": "robot_1",
  "pickup_node_id": "receiving",
  "dropoff_node_id": "storage_a",
  "priority": 60,
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

wait_service_type /v2/submit_order "robot_interfaces_business/srv/SubmitOrder" 10 /tmp/robot_vda_bridge_submit.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_vda_bridge_events.err

python3 "${SCRIPT_DIR}/vda5050_file_bridge.py" "${order_json}" "${state_json}" 45
python3 -m json.tool "${state_json}" >/dev/null
rg '"order_id": "bridge_alpha"|"mission_id": "station_order_vda5050_bridge_alpha"|"state": "FINISHED"' "${state_json}"

echo "vda5050 state bridge passed"
