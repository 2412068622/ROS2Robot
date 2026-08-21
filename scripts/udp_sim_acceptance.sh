#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
duration="${1:-5}"
port="${ROBOT_UDP_SIM_PORT:-19000}"
LOG_DIR="$(mktemp -d /tmp/robot_udp_sim_acceptance.XXXXXX)"
PIDS=()

set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

cleanup() {
  local status=$?
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  if [[ "${status}" -eq 0 && "${KEEP_ROBOT_LOGS:-0}" != "1" ]]; then
    rm -rf "${LOG_DIR}"
  else
    echo "UDP simulator acceptance logs: ${LOG_DIR}" >&2
  fi
}
trap cleanup EXIT

fail() {
  echo "UDP simulator acceptance failed: $*" >&2
  echo "logs: ${LOG_DIR}" >&2
  exit 1
}

start_bg() {
  local name="$1"
  shift
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

wait_topic() {
  local topic="$1"
  local timeout_sec="${2:-15}"
  local deadline=$((SECONDS + timeout_sec))
  while ! ros2 topic list 2>/dev/null | grep -qx "${topic}"; do
    if ((SECONDS >= deadline)); then
      fail "${topic} did not appear"
    fi
    sleep 1
  done
}

sample_topic_once() {
  local topic="$1"
  local type="$2"
  local output="$3"
  timeout 6s ros2 topic echo --once "${topic}" "${type}" >"${output}" 2>"${output}.err"
}

odom_x_from_sample() {
  awk '
    /position:/ {section="position"; next}
    section=="position" && /^[[:space:]]+x:/ {print $2; exit}
  ' "$1"
}

start_bg chassis_simulator ros2 launch robot_hardware chassis_simulator.launch.py \
  bind_host:=127.0.0.1 \
  port:="${port}"
start_bg chassis_driver ros2 run robot_hardware chassis_driver_node --ros-args \
  -p backend:=udp \
  -p udp_host:=127.0.0.1 \
  -p udp_port:="${port}" \
  -p io_timeout_ms:=1000 \
  -p protocol:=text \
  -p imu_source:=none

wait_topic "/chassis/state" 15
wait_topic "/odom" 15
sleep 1

timeout "${duration}"s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.1}}" \
  --rate 10 >"${LOG_DIR}/cmd_vel_pub.log" 2>&1 || true

odom_sample="${LOG_DIR}/odom_sample.yaml"
state_sample="${LOG_DIR}/chassis_state_sample.yaml"
sample_topic_once "/odom" "nav_msgs/msg/Odometry" "${odom_sample}" ||
  fail "failed to sample /odom"
sample_topic_once "/chassis/state" "robot_interfaces/msg/ChassisState" "${state_sample}" ||
  fail "failed to sample /chassis/state"

odom_x="$(odom_x_from_sample "${odom_sample}")"
if ! awk -v x="${odom_x:-0}" 'BEGIN { exit !(x > 0.01) }'; then
  cat "${odom_sample}" >&2
  fail "/odom pose.position.x did not move forward; x='${odom_x:-}'"
fi
if ! grep -q "backend: udp" "${state_sample}"; then
  cat "${state_sample}" >&2
  fail "/chassis/state backend was not udp"
fi
if ! grep -q "connected: true" "${state_sample}"; then
  cat "${state_sample}" >&2
  fail "/chassis/state.connected was not true"
fi
if ! grep -q "simulated" "${state_sample}"; then
  cat "${state_sample}" >&2
  fail "/chassis/state status did not include simulator state"
fi
if ! grep -q "wheels_rpm=" "${state_sample}"; then
  cat "${state_sample}" >&2
  fail "/chassis/state status did not include wheel rpm telemetry"
fi

echo "UDP simulator adapter loop OK on 127.0.0.1:${port}; odom_x=${odom_x}; ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
