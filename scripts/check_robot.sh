#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Usage:
  scripts/check_robot.sh --list
  scripts/check_robot.sh <suite> <case> [case args...]
  scripts/check_robot.sh <suite> all

Suites and cases:
  mission:
    runner controls queue queue_control reprioritize recovery resource_cleanup action_result events estimates preflight
    transport_order station_transport station_sequence station_order_batch
    station_order_batch_cancel workflow_cancel workflow_pause_resume station_confirmation
    station_confirmation_timeout return_to_dock docking_state docking_retry payload
    top_module all
  facility:
    actions charging door_passage lift_state lift_session plc multi_floor showcase all
  amr_sim:
    default gazebo_entities station_motion robot2 traffic_wait visualization rviz_topics assets
    task_dashboard gazebo_sensor gazebo_slam gazebo_nav2 full all
  business:
    order catalog idempotent picking_wave replenishment warehouse_fulfillment callbacks
    webhook fulfillment_benchmark vda5050_adapter vda5050_state vda5050_instant
    vda5050_mqtt business_benchmark fleet_preview fleet_runtime fleet_task_pool
    remote_fleet scenario_tasks scenario_workflows scenario_facility_workflow
    scenario_station_sequence scenario_station_sequence_cancel industry_workflows
    more_industry_workflows all
  safety:
    dynamic_route dynamic_speed obstacle_stop obstacle_recovery localization localization_auto_recovery
    relocalization low_battery_dock opportunity_charging route_locks intersection_lock traffic_deadlock
    fault_supervisor system_health all
  ops:
    rest_gateway rest_workflow operator_snapshot operator_permissions operator_static udp_chassis
    interfaces_split legacy_usage site_config_validation site_config_activation map_manager map_zones tf_tree topics all

Examples:
  scripts/check_robot.sh mission queue
  scripts/check_robot.sh amr_sim full /tmp/robot_full_amr_simulation_latest
  scripts/check_robot.sh business all
EOF
}

run_script() {
  local script_name="$1"
  shift || true
  local script_path="${SCRIPT_DIR}/${script_name}"
  if [[ ! -x "${script_path}" ]]; then
    echo "check script is missing or not executable: ${script_path}" >&2
    return 1
  fi
  echo "==> ${script_name} $*"
  "${script_path}" "$@"
}

case_script() {
  local suite="$1"
  local check_case="$2"
  case "${suite}:${check_case}" in
    mission:runner) echo "check_mission_runner.sh" ;;
    mission:controls) echo "check_mission_controls.sh" ;;
    mission:queue) echo "check_mission_queue.sh" ;;
    mission:queue_control) echo "check_mission_queue_control.sh" ;;
    mission:reprioritize) echo "check_mission_reprioritize.sh" ;;
    mission:recovery) echo "check_mission_recovery.sh" ;;
    mission:resource_cleanup) echo "check_mission_resource_cleanup_runtime.sh" ;;
    mission:action_result) echo "check_mission_action_result_runtime.sh" ;;
    mission:events) echo "check_mission_events.sh" ;;
    mission:estimates) echo "check_mission_estimates.sh" ;;
    mission:preflight) echo "check_mission_preflight.sh" ;;
    mission:transport_order) echo "check_transport_order.sh" ;;
    mission:station_transport) echo "check_station_transport_order.sh" ;;
    mission:station_sequence) echo "check_station_sequence_task.sh" ;;
    mission:station_order_batch) echo "check_station_order_batch.sh" ;;
    mission:station_order_batch_cancel) echo "check_station_order_batch_cancel.sh" ;;
    mission:workflow_cancel) echo "check_workflow_cancel.sh" ;;
    mission:workflow_pause_resume) echo "check_workflow_pause_resume.sh" ;;
    mission:station_confirmation) echo "check_station_confirmation.sh" ;;
    mission:station_confirmation_timeout) echo "check_station_confirmation_timeout.sh" ;;
    mission:return_to_dock) echo "check_return_to_dock.sh" ;;
    mission:docking_state) echo "check_docking_state_machine.sh" ;;
    mission:docking_retry) echo "check_docking_retry.sh" ;;
    mission:payload) echo "check_payload_load_unload.sh" ;;
    mission:top_module) echo "check_top_module_action.sh" ;;

    facility:actions) echo "check_facility_actions.sh" ;;
    facility:charging) echo "check_facility_charging.sh" ;;
    facility:door_passage) echo "check_door_passage_workflow.sh" ;;
    facility:lift_state) echo "check_lift_door_state.sh" ;;
    facility:lift_session) echo "check_lift_session.sh" ;;
    facility:plc) echo "check_facility_plc_adapter.sh" ;;
    facility:multi_floor) echo "check_multi_floor_sequence.sh" ;;
    facility:showcase) echo "check_amr_facility_sequence_showcase.sh" ;;

    amr_sim:default) echo "check_amr_sim_default_demo.sh" ;;
    amr_sim:gazebo_entities) echo "check_amr_sim_gazebo_entities.sh" ;;
    amr_sim:station_motion) echo "check_amr_sim_station_motion.sh" ;;
    amr_sim:robot2) echo "check_amr_sim_robot2_namespace.sh" ;;
    amr_sim:traffic_wait) echo "check_amr_sim_two_robot_traffic_wait.sh" ;;
    amr_sim:visualization) echo "check_amr_sim_visualization.sh" ;;
    amr_sim:rviz_topics) echo "check_amr_rviz_showcase_topics.sh" ;;
    amr_sim:assets) echo "check_amr_showcase_demo_assets.sh" ;;
    amr_sim:task_dashboard) echo "check_amr_task_dashboard_showcase.sh" ;;
    amr_sim:gazebo_sensor) echo "check_gazebo_sensor_stack.sh" ;;
    amr_sim:gazebo_slam) echo "check_gazebo_slam_runtime.sh" ;;
    amr_sim:gazebo_nav2) echo "check_gazebo_nav2_runtime.sh" ;;
    amr_sim:full) echo "check_full_amr_simulation_stack.sh" ;;

    business:order) echo "check_business_order.sh" ;;
    business:catalog) echo "check_business_order_catalog.sh" ;;
    business:idempotent) echo "check_idempotent_order_submit.sh" ;;
    business:picking_wave) echo "check_picking_wave.sh" ;;
    business:replenishment) echo "check_replenishment_order.sh" ;;
    business:warehouse_fulfillment) echo "check_warehouse_fulfillment.sh" ;;
    business:callbacks) echo "check_external_callbacks_and_idempotency.sh" ;;
    business:webhook) echo "check_external_webhook_callbacks.sh" ;;
    business:fulfillment_benchmark) echo "run_fulfillment_benchmark.sh" ;;
    business:vda5050_adapter) echo "check_vda5050_adapter.sh" ;;
    business:vda5050_state) echo "check_vda5050_state_bridge.sh" ;;
    business:vda5050_instant) echo "check_vda5050_instant_actions.sh" ;;
    business:vda5050_mqtt) echo "check_vda5050_mqtt_mock_bridge.sh" ;;
    business:business_benchmark) echo "run_business_order_benchmark.sh" ;;
    business:fleet_preview) echo "check_fleet_preview.sh" ;;
    business:fleet_runtime) echo "check_fleet_runtime_state.sh" ;;
    business:fleet_task_pool) echo "check_fleet_task_pool.sh" ;;
    business:remote_fleet) echo "check_remote_fleet_dispatch.sh" ;;
    business:scenario_tasks) echo "check_scenario_tasks.sh" ;;
    business:scenario_workflows) echo "check_scenario_workflows.sh" ;;
    business:scenario_facility_workflow) echo "check_scenario_facility_workflow.sh" ;;
    business:scenario_station_sequence) echo "check_scenario_station_sequence.sh" ;;
    business:scenario_station_sequence_cancel) echo "check_scenario_station_sequence_cancel.sh" ;;
    business:industry_workflows) echo "check_industry_workflows.sh" ;;
    business:more_industry_workflows) echo "check_more_industry_workflows.sh" ;;

    safety:dynamic_route) echo "check_dynamic_route_block.sh" ;;
    safety:dynamic_speed) echo "check_dynamic_speed_limit.sh" ;;
    safety:obstacle_stop) echo "check_amr_obstacle_safety_stop.sh" ;;
    safety:obstacle_recovery) echo "check_obstacle_blockage_recovery.sh" ;;
    safety:localization) echo "check_localization_health.sh" ;;
    safety:localization_auto_recovery) echo "check_localization_auto_recovery.sh" ;;
    safety:relocalization) echo "check_relocalization_resume.sh" ;;
    safety:low_battery_dock) echo "check_low_battery_dock.sh" ;;
    safety:opportunity_charging) echo "check_opportunity_charging.sh" ;;
    safety:route_locks) echo "check_route_resource_locks.sh" ;;
    safety:intersection_lock) echo "check_intersection_lock.sh" ;;
    safety:traffic_deadlock) echo "check_traffic_deadlock_detection.sh" ;;
    safety:fault_supervisor) echo "check_fault_supervisor.sh" ;;
    safety:system_health) echo "check_system_health.sh" ;;

    ops:rest_gateway) echo "check_rest_gateway.sh" ;;
    ops:rest_workflow) echo "check_rest_workflow_controls.sh" ;;
    ops:operator_snapshot) echo "check_operator_snapshot.sh" ;;
    ops:operator_permissions) echo "check_operator_permissions.sh" ;;
    ops:operator_static) echo "check_operator_console_static.sh" ;;
    ops:udp_chassis) echo "udp_sim_acceptance.sh" ;;
    ops:interfaces_split) echo "check_robot_interfaces_split.sh" ;;
    ops:legacy_usage) echo "check_robot_interfaces_legacy_usage.sh" ;;
    ops:site_config_validation) echo "check_site_config_validation.sh" ;;
    ops:site_config_activation) echo "check_site_config_activation.sh" ;;
    ops:map_manager) echo "check_map_manager.sh" ;;
    ops:map_zones) echo "check_map_zones.sh" ;;
    ops:tf_tree) echo "check_tf_tree.sh" ;;
    ops:topics) echo "check_topics.sh" ;;

    *) return 1 ;;
  esac
}

suite_cases() {
  case "$1" in
    mission)
      echo "runner controls queue queue_control reprioritize recovery resource_cleanup action_result events estimates preflight transport_order station_transport station_sequence station_order_batch station_order_batch_cancel workflow_cancel workflow_pause_resume station_confirmation station_confirmation_timeout return_to_dock docking_state docking_retry payload top_module"
      ;;
    facility)
      echo "actions charging door_passage lift_state lift_session plc multi_floor showcase"
      ;;
    amr_sim)
      echo "default gazebo_entities station_motion robot2 traffic_wait visualization rviz_topics assets task_dashboard gazebo_sensor gazebo_slam gazebo_nav2 full"
      ;;
    business)
      echo "order catalog idempotent picking_wave replenishment warehouse_fulfillment callbacks webhook fulfillment_benchmark vda5050_adapter vda5050_state vda5050_instant vda5050_mqtt business_benchmark fleet_preview fleet_runtime fleet_task_pool remote_fleet scenario_tasks scenario_workflows scenario_facility_workflow scenario_station_sequence scenario_station_sequence_cancel industry_workflows more_industry_workflows"
      ;;
    safety)
      echo "dynamic_route dynamic_speed obstacle_stop obstacle_recovery localization localization_auto_recovery relocalization low_battery_dock opportunity_charging route_locks intersection_lock traffic_deadlock fault_supervisor system_health"
      ;;
    ops)
      echo "rest_gateway rest_workflow operator_snapshot operator_permissions operator_static udp_chassis interfaces_split legacy_usage site_config_validation site_config_activation map_manager map_zones tf_tree topics"
      ;;
    *) return 1 ;;
  esac
}

run_case() {
  local suite="$1"
  local check_case="$2"
  shift 2
  local script_name
  if ! script_name="$(case_script "${suite}" "${check_case}")"; then
    echo "unknown check case: ${suite} ${check_case}" >&2
    usage >&2
    return 1
  fi
  run_script "${script_name}" "$@"
}

main() {
  if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    usage
    return 0
  fi
  if [[ "${1:-}" == "--list" ]]; then
    usage
    return 0
  fi
  if [[ $# -lt 2 ]]; then
    usage >&2
    return 1
  fi

  local suite="$1"
  local check_case="$2"
  shift 2

  if [[ "${check_case}" == "all" ]]; then
    local cases
    if ! cases="$(suite_cases "${suite}")"; then
      echo "unknown check suite: ${suite}" >&2
      usage >&2
      return 1
    fi
    local current_case
    for current_case in ${cases}; do
      run_case "${suite}" "${current_case}" "$@"
    done
    return 0
  fi

  run_case "${suite}" "${check_case}" "$@"
}

main "$@"
