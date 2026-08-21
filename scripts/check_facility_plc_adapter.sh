#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT
plc_json="${tmp_dir}/plc_state.json"
request_json="${tmp_dir}/plc_request.json"
result_json="${tmp_dir}/plc_result.json"

cat >"${plc_json}" <<'JSON'
{
  "receiving_door": {
    "state": "closed"
  }
}
JSON

cat >"${request_json}" <<'JSON'
{
  "request_id": "plc_door_alpha",
  "resource_id": "receiving_door",
  "resource_type": "door",
  "action": "open",
  "priority": 70,
  "start_if_idle": true,
  "preempt_current": false,
  "hold_after_action": true
}
JSON

wait_service_type /v2/execute_facility_action "robot_interfaces_facility/srv/ExecuteFacilityAction" 10 /tmp/robot_plc_facility_action.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_plc_events.err

python3 "${SCRIPT_DIR}/facility_plc_file_adapter.py" "${plc_json}" "${request_json}" "${result_json}" 45
python3 -m json.tool "${plc_json}" >/dev/null
python3 -m json.tool "${result_json}" >/dev/null
rg '"state": "open"|"last_action": "open"' "${plc_json}"
rg '"state_before": "closed"|"state_after": "open"|"finished": true' "${result_json}"

echo "facility plc adapter passed"
