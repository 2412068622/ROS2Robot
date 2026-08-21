#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

LOG_DIR="$(mktemp -d /tmp/robot_amr_obstacle_safety_stop.XXXXXX)"
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

publish_probe_cmd() {
  ros2 topic pub --once /virtual_rc/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.35, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >"${LOG_DIR}/publish_probe_cmd.out" 2>"${LOG_DIR}/publish_probe_cmd.err"
}

sample_cmd_vel() {
  local output="$1"
  timeout 4s ros2 topic echo --once /cmd_vel geometry_msgs/msg/Twist \
    >"${output}" 2>"${output}.err"
}

linear_x_from_sample() {
  awk '/linear:/ {section="linear"} section=="linear" && /x:/ {print $2; exit}' "$1"
}

wait_cmd_linear_condition() {
  local condition="$1"
  local timeout_sec="${2:-20}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/cmd_vel_sample.txt"
  local linear_x
  while true; do
    if sample_cmd_vel "${sample}"; then
      linear_x="$(linear_x_from_sample "${sample}")"
      if awk -v x="${linear_x:-999}" "BEGIN { exit !(${condition}) }"; then
        return 0
      fi
    fi
    if ((SECONDS >= deadline)); then
      echo "cmd_vel linear.x did not satisfy '${condition}'" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${sample}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

wait_safety_state() {
  local expected_state="$1"
  local timeout_sec="${2:-20}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/safety_state_sample.txt"
  while true; do
    if timeout 4s ros2 topic echo --once /safety_state robot_interfaces/msg/SafetyState \
        >"${sample}" 2>"${sample}.err" &&
      rg "state: ${expected_state}" "${sample}" >/dev/null; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "safety_state did not reach ${expected_state}" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${sample}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

start_bg cmd_vel_mux ros2 run robot_teleop cmd_vel_mux_node --ros-args -p default_source:=teleop
start_bg virtual_rc ros2 run robot_teleop virtual_rc_node --ros-args -p manual_takeover:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  return_to_dock_on_low_battery:=false

wait_service_type /v2/report_obstacle_blockage "robot_interfaces_navigation/srv/ReportObstacleBlockage" 30 \
  "${LOG_DIR}/wait_report_obstacle.err"
wait_service_type /v2/clear_obstacle_blockage "robot_interfaces_navigation/srv/ClearObstacleBlockage" 30 \
  "${LOG_DIR}/wait_clear_obstacle.err"
wait_topic_type /cmd_vel "geometry_msgs/msg/Twist" 20 "${LOG_DIR}/wait_cmd_vel.err"
wait_topic_type /safety_state "robot_interfaces/msg/SafetyState" 20 "${LOG_DIR}/wait_safety_state.err"

publish_probe_cmd
wait_cmd_linear_condition "x > 0.10" 20

ros2 service call /v2/report_obstacle_blockage robot_interfaces_navigation/srv/ReportObstacleBlockage \
  "{blockage_id: sim_safety_stop_probe, reason: safety stop probe, pause_active: false}" \
  >"${LOG_DIR}/report_obstacle.out"
rg "success=True" "${LOG_DIR}/report_obstacle.out"
rg "state='OBSTACLE_BLOCKED'" "${LOG_DIR}/report_obstacle.out"

wait_safety_state "OBSTACLE_BLOCKED" 20
wait_cmd_linear_condition "x < 0.02 && x > -0.02" 20

ros2 service call /v2/clear_obstacle_blockage robot_interfaces_navigation/srv/ClearObstacleBlockage \
  "{blockage_id: sim_safety_stop_probe, resolution: safety stop probe cleared, resume_active: false}" \
  >"${LOG_DIR}/clear_obstacle.out"
rg "success=True" "${LOG_DIR}/clear_obstacle.out"
rg "state='OK'" "${LOG_DIR}/clear_obstacle.out"

publish_probe_cmd
wait_safety_state "OK" 20
wait_cmd_linear_condition "x > 0.10" 20

echo "AMR obstacle safety stop passed"
