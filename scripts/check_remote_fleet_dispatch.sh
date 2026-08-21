#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/update_fleet_robot_state "robot_interfaces_fleet/srv/UpdateFleetRobotState" 10 /tmp/robot_remote_fleet_update_robot.err
wait_submit_order 10 /tmp/robot_remote_fleet_submit.err
wait_service_type /v2/update_remote_task_state "robot_interfaces_fleet/srv/UpdateRemoteTaskState" 10 /tmp/robot_remote_fleet_update_task.err
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 10 /tmp/robot_remote_fleet_events.err
wait_service_type /v2/get_operator_snapshot "robot_interfaces_business/srv/GetOperatorSnapshot" 10 /tmp/robot_remote_fleet_snapshot.err

ros2 service call /v2/update_fleet_robot_state robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_1, enabled: true, state: busy, battery_voltage: 24.0, current_station_id: dock, update_enabled: true, update_state: true, update_battery_voltage: true, update_current_station_id: true}" \
  | rg "success=True|robot_1|busy"

ros2 service call /v2/update_fleet_robot_state robot_interfaces_fleet/srv/UpdateFleetRobotState \
  "{robot_id: robot_2, enabled: true, state: available, battery_voltage: 24.2, current_station_id: receiving, update_enabled: true, update_state: true, update_battery_voltage: true, update_current_station_id: true}" \
  | rg "success=True|robot_2|available"

call_submit_order remote_alpha fleet_station 20 \
  "task_id=remote_alpha;required_capability=station_transport;pickup_station=receiving;dropoff_station=storage_a;start_if_idle=true;preempt_current=false" \
  fleet_station_task \
  | rg "accepted=True|remote_fleet_task_remote_alpha|dispatched remote fleet task"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: DISPATCHED_REMOTE, mission_id_filter: remote_fleet_task_remote_alpha}" \
  | rg "success=True|DISPATCHED_REMOTE|remote_fleet_task_remote_alpha"

ros2 service call /v2/update_remote_task_state robot_interfaces_fleet/srv/UpdateRemoteTaskState \
  "{task_id: remote_alpha, state: FINISHED, message: remote robot completed task}" \
  | rg "success=True|remote_alpha|robot_2|FINISHED"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: FINISHED, mission_id_filter: remote_fleet_task_remote_alpha}" \
  | rg "success=True|FINISHED|remote robot completed task"

ros2 service call /v2/get_operator_snapshot robot_interfaces_business/srv/GetOperatorSnapshot "{event_limit: 10}" \
  | rg "success=True|remote_task_ids=\\['remote_alpha'\\]|remote_task_robot_ids=\\['robot_2'\\]|remote_task_states=\\['FINISHED'\\]"

echo "remote fleet dispatch passed"
