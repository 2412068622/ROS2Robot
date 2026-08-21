#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_vda5050_instant.XXXXXX)"
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

write_action() {
  local path="$1"
  local action_id="$2"
  local action_type="$3"
  local order_id="$4"
  cat >"${path}" <<JSON
{
  "action_id": "${action_id}",
  "action_type": "${action_type}",
  "order_id": "${order_id}",
  "serial_number": "robot_1"
}
JSON
}

start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  auto_start_queue:=false \
  return_to_dock_on_low_battery:=false

wait_submit_order 20 "${LOG_DIR}/wait_submit.err"
wait_service_type /v2/pause_workflow "robot_interfaces_business/srv/PauseWorkflow" 20 \
  "${LOG_DIR}/wait_pause.err"
wait_service_type /v2/resume_workflow "robot_interfaces_business/srv/ResumeWorkflow" 20 \
  "${LOG_DIR}/wait_resume.err"
wait_service_type /v2/cancel_workflow "robot_interfaces_business/srv/CancelWorkflow" 20 \
  "${LOG_DIR}/wait_cancel.err"
wait_service_type /v2/get_workflow_status "robot_interfaces_business/srv/GetWorkflowStatus" 20 \
  "${LOG_DIR}/wait_status.err"

call_submit_order vda_instant_alpha scenario_workflow 30 \
  "scenario_id=campus;workflow_id=full_mail_route;order_id=vda_instant_alpha;start_if_idle=false;preempt_current=false" \
  scenario_workflow \
  >"${LOG_DIR}/submit.out"
rg "accepted=True" "${LOG_DIR}/submit.out"
rg "station_sequence_vda_instant_alpha_step_1_leg_1" "${LOG_DIR}/submit.out"

write_action "${LOG_DIR}/pause.json" "act_pause" "pause" "vda_instant_alpha"
python3 "${SCRIPT_DIR}/vda5050_instant_action_bridge.py" \
  "${LOG_DIR}/pause.json" "${LOG_DIR}/pause_state.json"
python3 -m json.tool "${LOG_DIR}/pause_state.json" >/dev/null
rg '"action_type": "pause"' "${LOG_DIR}/pause_state.json"
rg '"accepted": true' "${LOG_DIR}/pause_state.json"
rg '"action_state": "FINISHED"' "${LOG_DIR}/pause_state.json"
ros2 service call /v2/get_workflow_status robot_interfaces_business/srv/GetWorkflowStatus \
  "{order_id: vda_instant_alpha}" >"${LOG_DIR}/paused_status.out"
rg "state='PAUSED'" "${LOG_DIR}/paused_status.out"
rg "paused=True" "${LOG_DIR}/paused_status.out"

write_action "${LOG_DIR}/resume.json" "act_resume" "resume" "vda_instant_alpha"
python3 "${SCRIPT_DIR}/vda5050_instant_action_bridge.py" \
  "${LOG_DIR}/resume.json" "${LOG_DIR}/resume_state.json"
python3 -m json.tool "${LOG_DIR}/resume_state.json" >/dev/null
rg '"action_type": "resume"' "${LOG_DIR}/resume_state.json"
rg '"accepted": true' "${LOG_DIR}/resume_state.json"
rg '"action_state": "FINISHED"' "${LOG_DIR}/resume_state.json"

write_action "${LOG_DIR}/factsheet.json" "act_factsheet" "factsheet" ""
python3 "${SCRIPT_DIR}/vda5050_instant_action_bridge.py" \
  "${LOG_DIR}/factsheet.json" "${LOG_DIR}/factsheet_state.json"
python3 -m json.tool "${LOG_DIR}/factsheet_state.json" >/dev/null
rg '"action_type": "factsheet"' "${LOG_DIR}/factsheet_state.json"
rg '"supported_instant_actions"' "${LOG_DIR}/factsheet_state.json"
rg '"cancelOrder"' "${LOG_DIR}/factsheet_state.json"

write_action "${LOG_DIR}/cancel.json" "act_cancel" "cancelOrder" "vda_instant_alpha"
python3 "${SCRIPT_DIR}/vda5050_instant_action_bridge.py" \
  "${LOG_DIR}/cancel.json" "${LOG_DIR}/cancel_state.json"
python3 -m json.tool "${LOG_DIR}/cancel_state.json" >/dev/null
rg '"action_type": "cancelOrder"' "${LOG_DIR}/cancel_state.json"
rg '"accepted": true' "${LOG_DIR}/cancel_state.json"
rg '"action_state": "FINISHED"' "${LOG_DIR}/cancel_state.json"
ros2 service call /v2/get_workflow_status robot_interfaces_business/srv/GetWorkflowStatus \
  "{order_id: vda_instant_alpha}" >"${LOG_DIR}/canceled_status.out"
rg "success=False" "${LOG_DIR}/canceled_status.out"
rg "total_steps=0" "${LOG_DIR}/canceled_status.out"
rg "queued_steps=0" "${LOG_DIR}/canceled_status.out"

echo "vda5050 instant actions passed"
