#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 160 + 40))}"

LOG_DIR="$(mktemp -d /tmp/robot_localization_auto_recovery.XXXXXX)"
MISSION_FILE="${LOG_DIR}/localization_auto_recovery.yaml"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -TERM "-${pid}" >/dev/null 2>&1 || kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  rm -rf "${LOG_DIR}"
}
trap cleanup EXIT

start_bg() {
  local name="$1"
  shift
  setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

cat >"${MISSION_FILE}" <<'YAML'
mission_id: localization_auto_recovery_long
frame_id: map
loop: false
goals:
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
  - x: 1.2
    y: 0.0
    yaw: 0.0
YAML

publish_amcl_covariance() {
  local covariance="$1"
  COVARIANCE="${covariance}" python3 - <<'PY'
import os
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

covariance = float(os.environ["COVARIANCE"])
rclpy.init()
node = rclpy.create_node("mock_amcl_covariance_update")
pub = node.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.pose.pose.orientation.w = 1.0
msg.pose.covariance[0] = covariance
msg.pose.covariance[7] = covariance
for _ in range(8):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
PY
}

wait_runner_state() {
  local pattern="$1"
  local timeout_sec="${2:-20}"
  local out_file="${LOG_DIR}/mission_state_${pattern//[^A-Za-z0-9]/_}.out"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    timeout 4s ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState \
      --field state >"${out_file}" 2>"${out_file}.err" || true
    if rg "${pattern}" "${out_file}" >/dev/null; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "mission runner did not reach state matching ${pattern}" >&2
      cat "${out_file}" >&2 || true
      return 1
    fi
    sleep 1
  done
}

wait_health_state() {
  local state_pattern="$1"
  local localized="$2"
  local timeout_sec="${3:-20}"
  local out_file="${LOG_DIR}/localization_health_${state_pattern//[^A-Za-z0-9]/_}.out"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    timeout 4s ros2 topic echo --once /localization_health \
      robot_interfaces/msg/LocalizationHealth >"${out_file}" 2>"${out_file}.err" || true
    if rg "state: ${state_pattern}" "${out_file}" >/dev/null &&
        rg "localized: ${localized}" "${out_file}" >/dev/null; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "localization health did not reach ${state_pattern}/${localized}" >&2
      cat "${out_file}" >&2 || true
      return 1
    fi
    sleep 1
  done
}

wait_mission_event() {
  local state="$1"
  local message_pattern="$2"
  local timeout_sec="${3:-20}"
  local out_file="${LOG_DIR}/event_${state}.out"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
      "{limit: 40, state_filter: ${state}, mission_id_filter: localization}" \
      >"${out_file}" 2>"${out_file}.err" || true
    if rg "success=True" "${out_file}" >/dev/null &&
        rg "loaded [1-9][0-9]* mission event" "${out_file}" >/dev/null &&
        rg "${state}" "${out_file}" >/dev/null &&
        rg "${message_pattern}" "${out_file}" >/dev/null; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "missing localization event ${state}" >&2
      cat "${out_file}" >&2 || true
      return 1
    fi
    sleep 1
  done
}

start_bg navigate_sequence_server ros2 run robot_tasks navigate_sequence_server_node --ros-args \
  -p use_nav2_action:=false \
  -p simulate_without_nav2:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  mission_file:="${MISSION_FILE}" \
  autostart:=false \
  server_timeout_ms:=3000 \
  return_to_dock_on_low_battery:=false \
  localization_covariance_threshold:=0.5 \
  localization_pause_on_lost:=true

wait_action_type /navigate_sequence "robot_interfaces/action/NavigateSequence" 20 \
  "${LOG_DIR}/wait_action.err"
wait_service_type /start_mission_profile "std_srvs/srv/Trigger" 20 \
  "${LOG_DIR}/wait_start.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"
wait_topic_type /localization_health "robot_interfaces/msg/LocalizationHealth" 20 \
  "${LOG_DIR}/wait_localization_health.err"
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 20 \
  "${LOG_DIR}/wait_mission_state.err"

ros2 service call /start_mission_profile std_srvs/srv/Trigger "{}" \
  >"${LOG_DIR}/start.out" 2>"${LOG_DIR}/start.err"
rg "success=True" "${LOG_DIR}/start.out"
wait_runner_state "RUNNING" 10

publish_amcl_covariance 1.2
wait_health_state LOST false 20
wait_runner_state "LOCALIZATION_LOST|PAUSED" 20
wait_mission_event LOCALIZATION_LOST "covariance"

publish_amcl_covariance 0.05
wait_health_state "OK|RECOVERED" true 20
wait_runner_state "RUNNING|FINISHED" 20
wait_mission_event LOCALIZATION_RECOVERED "covariance recovered"

echo "localization auto recovery passed"
