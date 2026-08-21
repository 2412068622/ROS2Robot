#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_submit_order 10 /tmp/robot_workflow_cancel_submit.err
wait_service_type /v2/cancel_workflow "robot_interfaces_business/srv/CancelWorkflow" 10 /tmp/robot_workflow_cancel.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_workflow_cancel_events.err

call_submit_order cancel_alpha scenario_workflow 0 \
  "scenario_id=warehouse;workflow_id=inbound_crossdock;order_id=cancel_alpha;start_if_idle=false;preempt_current=false" \
  scenario_workflow \
  | rg "accepted=True|station_order_fleet_cancel_alpha_step_1|station_order_fleet_cancel_alpha_step_2|queue_size=2"

ros2 service call /v2/cancel_workflow robot_interfaces_business/srv/CancelWorkflow \
  "{order_id: cancel_alpha, cancel_active: false}" \
  | rg "success=True|station_order_fleet_cancel_alpha_step_1|station_order_fleet_cancel_alpha_step_2|active_cancel_requested=False|queue_size=0"

ros2 service call /v2/cancel_workflow robot_interfaces_business/srv/CancelWorkflow \
  "{order_id: cancel_alpha, cancel_active: false}" \
  | rg "success=False|workflow has no queued mission: cancel_alpha|queue_size=0"

echo "workflow cancel passed"
