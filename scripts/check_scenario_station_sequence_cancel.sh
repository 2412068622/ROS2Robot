#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_submit_order 10 /tmp/robot_scenario_sequence_cancel_submit.err
wait_service_type /v2/cancel_workflow "robot_interfaces_business/srv/CancelWorkflow" 10 /tmp/robot_scenario_sequence_cancel.err

call_submit_order campus_cancel_alpha scenario_workflow 0 \
  "scenario_id=campus;workflow_id=full_mail_route;order_id=campus_cancel_alpha;start_if_idle=false;preempt_current=false" \
  scenario_workflow \
  | rg "accepted=True|station_sequence_campus_cancel_alpha_step_1_leg_1|queue_size=4"

ros2 service call /v2/cancel_workflow robot_interfaces_business/srv/CancelWorkflow \
  "{order_id: campus_cancel_alpha, cancel_active: false}" \
  | rg "success=True|station_sequence_campus_cancel_alpha_step_1_leg_1|station_sequence_campus_cancel_alpha_step_1_leg_4|queue_size=0"

echo "scenario station sequence cancel passed"
