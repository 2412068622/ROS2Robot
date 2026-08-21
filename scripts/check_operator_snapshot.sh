#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/get_operator_snapshot "robot_interfaces_business/srv/GetOperatorSnapshot" 10 /tmp/robot_operator_snapshot.err
wait_submit_order 10 /tmp/robot_operator_snapshot_submit.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_operator_snapshot_events.err

ros2 service call /v2/get_operator_snapshot robot_interfaces_business/srv/GetOperatorSnapshot "{event_limit: 5}" \
  | rg "success=True|operator snapshot|fleet_robot_ids=\\[|facility_resource_ids=\\[|business_types=\\["

call_submit_order operator_alpha business 0 \
  "business_order_id=operator_alpha;business_type=warehouse_inbound;start_if_idle=true;preempt_current=false" \
  business_order \
  | rg "accepted=True|warehouse|door_assisted_inbound"

for mission in facility_action_business_operator_alpha_step_1 station_order_fleet_business_operator_alpha_step_2; do
  deadline=$((SECONDS + 45))
  while true; do
    if ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 80, state_filter: FINISHED, mission_id_filter: ${mission}}" \
      2>/tmp/robot_operator_snapshot_events.err | rg "loaded 1 mission event|states=\\['FINISHED'\\]" >/dev/null; then
      break
    fi
    if (( SECONDS >= deadline )); then
      echo "operator snapshot mission did not finish: ${mission}" >&2
      exit 1
    fi
    sleep 1
  done
done

ros2 service call /v2/get_operator_snapshot robot_interfaces_business/srv/GetOperatorSnapshot "{event_limit: 20}" \
  | rg "success=True|FINISHED|facility_action_business_operator_alpha_step_1|station_order_fleet_business_operator_alpha_step_2|business_types=\\["

echo "operator snapshot passed"
