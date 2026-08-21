#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

USER_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-}"
BASE_ROS_DOMAIN_ID="${USER_ROS_DOMAIN_ID:-$((RANDOM % 120 + 40))}"
LOG_DIR="$(mktemp -d /tmp/robot_mission_action_result.XXXXXX)"
MISSION_DIR="${LOG_DIR}/missions"
mkdir -p "${MISSION_DIR}"
PIDS=()

cleanup_phase() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -TERM "-${pid}" >/dev/null 2>&1 || kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  PIDS=()
  sleep 1
}

cleanup() {
  local status=$?
  cleanup_phase
  if [[ ${status} -eq 0 ]]; then
    rm -rf "${LOG_DIR}"
  else
    echo "preserving logs in ${LOG_DIR}" >&2
  fi
  return "${status}"
}
trap cleanup EXIT

set_phase_domain() {
  local offset="$1"
  if [[ -n "${USER_ROS_DOMAIN_ID}" ]]; then
    export ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID}"
  else
    export ROS_DOMAIN_ID="$((BASE_ROS_DOMAIN_ID + offset))"
  fi
}

start_bg() {
  local name="$1"
  shift
  setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

write_missions() {
  cat >"${MISSION_DIR}/autostart_success.yaml" <<'YAML'
mission_id: action_autostart_success
frame_id: map
loop: false
goals:
  - x: 0.1
    y: 0.0
    yaw: 0.0
YAML

  cat >"${MISSION_DIR}/queued_cancel.yaml" <<'YAML'
mission_id: action_queued_cancel
frame_id: map
loop: false
goals:
  - x: 0.0
    y: 0.0
    yaw: 0.0
  - x: 0.1
    y: 0.0
    yaw: 0.0
  - x: 0.2
    y: 0.0
    yaw: 0.0
  - x: 0.3
    y: 0.0
    yaw: 0.0
  - x: 0.4
    y: 0.0
    yaw: 0.0
  - x: 0.5
    y: 0.0
    yaw: 0.0
  - x: 0.6
    y: 0.0
    yaw: 0.0
  - x: 0.7
    y: 0.0
    yaw: 0.0
  - x: 0.8
    y: 0.0
    yaw: 0.0
  - x: 0.9
    y: 0.0
    yaw: 0.0
  - x: 1.0
    y: 0.0
    yaw: 0.0
  - x: 1.1
    y: 0.0
    yaw: 0.0
YAML

  cat >"${MISSION_DIR}/action_failure.yaml" <<'YAML'
mission_id: action_failure_runtime
frame_id: map
loop: false
goals:
  - x: 0.2
    y: 0.0
    yaw: 0.0
YAML

  cat >"${MISSION_DIR}/action_failure_should_wait.yaml" <<'YAML'
mission_id: action_failure_should_wait
frame_id: map
loop: false
goals:
  - x: 0.4
    y: 0.0
    yaw: 0.0
YAML
}

wait_mission_event() {
  local mission_id="$1"
  local state="$2"
  local timeout_sec="${3:-45}"
  local out_file="${LOG_DIR}/${mission_id}_${state}.out"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 80, state_filter: ${state}, mission_id_filter: ${mission_id}}" \
      >"${out_file}" 2>"${out_file}.err" || true
    if rg "success=True" "${out_file}" >/dev/null &&
        rg "loaded [1-9][0-9]* mission event" "${out_file}" >/dev/null &&
        rg "mission_ids=.*${mission_id}" "${out_file}" >/dev/null &&
        rg "states=.*${state}" "${out_file}" >/dev/null; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "mission ${mission_id} did not publish ${state} event" >&2
      cat "${out_file}" >&2 || true
      return 1
    fi
    sleep 1
  done
}

assert_no_mission_event() {
  local mission_id="$1"
  local state="$2"
  local out_file="${LOG_DIR}/${mission_id}_${state}_absent.out"
  ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
    "{limit: 80, state_filter: ${state}, mission_id_filter: ${mission_id}}" \
    >"${out_file}" 2>"${out_file}.err" || true
  rg "success=True" "${out_file}"
  if rg "loaded [1-9][0-9]* mission event" "${out_file}" >/dev/null ||
      rg "mission_ids=.*${mission_id}" "${out_file}" >/dev/null ||
      rg "states=.*${state}" "${out_file}" >/dev/null; then
    echo "unexpected ${state} event for ${mission_id}" >&2
    cat "${out_file}" >&2
    return 1
  fi
}

wait_active_mission() {
  local mission_id="$1"
  local timeout_sec="${2:-20}"
  local state_file="${LOG_DIR}/${mission_id}_active_state.out"
  local active_file="${LOG_DIR}/${mission_id}_active_goal.out"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    timeout 4s ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field active_goal_id >"${active_file}" 2>"${active_file}.err" || true
    timeout 4s ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state >"${state_file}" 2>"${state_file}.err" || true
    if rg "${mission_id}" "${active_file}" >/dev/null &&
        rg "STARTING|RUNNING|PAUSED" "${state_file}" >/dev/null; then
      return 0
    fi
    if rg "FAILED|ERROR|CANCELED|FINISHED" "${state_file}" >/dev/null; then
      echo "mission ${mission_id} left active state too early" >&2
      cat "${state_file}" >&2 || true
      cat "${active_file}" >&2 || true
      return 1
    fi
    if (( SECONDS >= deadline )); then
      echo "mission ${mission_id} did not become active" >&2
      cat "${state_file}" >&2 || true
      cat "${active_file}" >&2 || true
      return 1
    fi
    sleep 1
  done
}

wait_common_runtime() {
  local prefix="$1"
  wait_action_type /navigate_sequence "robot_interfaces/action/NavigateSequence" 20 \
    "${LOG_DIR}/${prefix}_wait_action.err"
  wait_service_type /start_mission_profile "std_srvs/srv/Trigger" 20 \
    "${LOG_DIR}/${prefix}_wait_start.err"
  wait_service_type /v2/enqueue_mission_profile "robot_interfaces_mission/srv/EnqueueMission" 20 \
    "${LOG_DIR}/${prefix}_wait_enqueue.err"
  wait_service_type /v2/cancel_queued_mission "robot_interfaces_mission/srv/CancelQueuedMission" 20 \
    "${LOG_DIR}/${prefix}_wait_cancel_queued.err"
  wait_service_type /v2/get_mission_recovery_state \
    "robot_interfaces_mission/srv/GetMissionRecoveryState" 20 \
    "${LOG_DIR}/${prefix}_wait_recovery.err"
  wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
    "${LOG_DIR}/${prefix}_wait_events.err"
  wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 20 \
    "${LOG_DIR}/${prefix}_wait_state.err"
}

start_success_stack() {
  local prefix="$1"
  local mission_file="$2"
  shift 2
  start_bg "${prefix}_navigate_sequence_server" \
    ros2 run robot_tasks navigate_sequence_server_node --ros-args \
      -p use_nav2_action:=false \
      -p simulate_without_nav2:=true
  start_bg "${prefix}_mission_runner" \
    ros2 launch robot_tasks mission_runner.launch.py \
      "mission_file:=${mission_file}" \
      server_timeout_ms:=3000 \
      return_to_dock_on_low_battery:=false \
      failure_recovery_enabled:=false \
      preflight_enabled:=false \
      estimate_use_nav2_path:=false \
      "$@"
  wait_common_runtime "${prefix}"
}

run_autostart_success_phase() {
  set_phase_domain 0
  start_success_stack autostart "${MISSION_DIR}/autostart_success.yaml" autostart:=true
  wait_mission_event action_autostart_success FINISHED 35
  cleanup_phase
}

run_queued_cancel_phase() {
  set_phase_domain 1
  start_success_stack queued_cancel "${MISSION_DIR}/queued_cancel.yaml" autostart:=false
  ros2 service call /v2/enqueue_mission_profile robot_interfaces_mission/srv/EnqueueMission \
    "{mission_file: \"${MISSION_DIR}/queued_cancel.yaml\", priority: 42, start_if_idle: true}" \
    >"${LOG_DIR}/queued_cancel_enqueue.out" 2>"${LOG_DIR}/queued_cancel_enqueue.err"
  rg "success=True" "${LOG_DIR}/queued_cancel_enqueue.out"
  wait_active_mission action_queued_cancel 20
  ros2 service call /v2/cancel_queued_mission robot_interfaces_mission/srv/CancelQueuedMission \
    "{mission_id: action_queued_cancel, cancel_active: true}" \
    >"${LOG_DIR}/queued_cancel_cancel.out" 2>"${LOG_DIR}/queued_cancel_cancel.err"
  rg "success=True" "${LOG_DIR}/queued_cancel_cancel.out"
  rg "active mission cancel requested" "${LOG_DIR}/queued_cancel_cancel.out"
  wait_mission_event action_queued_cancel CANCELED 35
  cleanup_phase
}

run_action_failure_phase() {
  set_phase_domain 2
  start_bg failure_navigate_sequence_server \
    ros2 run robot_tasks navigate_sequence_server_node --ros-args \
      -p use_waypoint_follower:=false \
      -p use_nav2_action:=true \
      -p simulate_without_nav2:=false \
      -p nav2_server_timeout_ms:=100
  start_bg failure_mission_runner \
    ros2 launch robot_tasks mission_runner.launch.py \
      "mission_file:=${MISSION_DIR}/action_failure.yaml" \
      autostart:=false \
      server_timeout_ms:=3000 \
      return_to_dock_on_low_battery:=false \
      failure_recovery_enabled:=false \
      preflight_enabled:=false \
      estimate_use_nav2_path:=false
  wait_common_runtime failure
  ros2 service call /v2/enqueue_mission_profile robot_interfaces_mission/srv/EnqueueMission \
    "{mission_file: \"${MISSION_DIR}/action_failure_should_wait.yaml\", priority: 1, start_if_idle: false}" \
    >"${LOG_DIR}/failure_enqueue_waiting.out" 2>"${LOG_DIR}/failure_enqueue_waiting.err"
  rg "success=True" "${LOG_DIR}/failure_enqueue_waiting.out"
  ros2 service call /v2/enqueue_mission_profile robot_interfaces_mission/srv/EnqueueMission \
    "{mission_file: \"${MISSION_DIR}/action_failure.yaml\", priority: 10, start_if_idle: true}" \
    >"${LOG_DIR}/failure_enqueue_start.out" 2>"${LOG_DIR}/failure_enqueue_start.err"
  rg "success=True" "${LOG_DIR}/failure_enqueue_start.out"
  wait_mission_event action_failure_runtime FAILED 35
  sleep 3
  assert_no_mission_event action_failure_should_wait FAILED
  ros2 service call /v2/get_mission_recovery_state \
    robot_interfaces_mission/srv/GetMissionRecoveryState "{}" \
    >"${LOG_DIR}/failure_recovery.out" 2>"${LOG_DIR}/failure_recovery.err"
  rg "success=True" "${LOG_DIR}/failure_recovery.out"
  rg "enabled=False" "${LOG_DIR}/failure_recovery.out"
  rg "last_failed_mission_id='action_failure_runtime'|last_failed_mission_id: action_failure_runtime" \
    "${LOG_DIR}/failure_recovery.out"
  rg "last_recovery_action='none'|last_recovery_action: none" \
    "${LOG_DIR}/failure_recovery.out"
  cleanup_phase
}

write_missions
run_autostart_success_phase
run_queued_cancel_phase
run_action_failure_phase

echo "mission action result runtime passed"
