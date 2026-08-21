#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

LOG_DIR="$(mktemp -d /tmp/robot_dynamic_speed.XXXXXX)"
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

start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  autostart:=false \
  return_to_dock_on_low_battery:=false \
  safety_zone_speed_limit_mps:=0.6
start_bg cmd_vel_mux ros2 run robot_teleop cmd_vel_mux_node

wait_topic_type /safety_state "robot_interfaces/msg/SafetyState" 20 "${LOG_DIR}/wait_safety.err"
wait_service_type /v2/set_dynamic_speed_limit "robot_interfaces_navigation/srv/SetDynamicSpeedLimit" 20 \
  "${LOG_DIR}/wait_speed_limit.err"
wait_topic_type /cmd_vel "geometry_msgs/msg/Twist" 20 "${LOG_DIR}/wait_cmd_vel.err"

ros2 service call /v2/set_dynamic_speed_limit robot_interfaces_navigation/srv/SetDynamicSpeedLimit \
  "{speed_limit_mps: 0.2, reason: narrow aisle}" \
  | rg "success=True|runtime_speed_limit_mps=0.2|effective_speed_limit_mps=0.2"

ros2 topic echo --once /safety_state robot_interfaces/msg/SafetyState \
  | rg "state: SPEED_LIMITED|runtime_speed_limit_mps: 0.2|effective_speed_limit_mps: 0.2"

python3 - <<'PY'
import math
import time

import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("dynamic_speed_limit_probe")
pub = node.create_publisher(Twist, "/teleop_cmd_vel", 10)
seen_limited = False

def on_cmd(msg):
    global seen_limited
    speed = math.hypot(msg.linear.x, msg.linear.y)
    if 0.15 <= speed <= 0.21:
        seen_limited = True

sub = node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
twist = Twist()
twist.linear.x = 1.0
deadline = time.time() + 5.0
while time.time() < deadline and not seen_limited:
    pub.publish(twist)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)

node.destroy_subscription(sub)
node.destroy_node()
rclpy.shutdown()
if not seen_limited:
    raise SystemExit("cmd_vel did not reflect dynamic speed limit")
PY

ros2 service call /v2/set_dynamic_speed_limit robot_interfaces_navigation/srv/SetDynamicSpeedLimit \
  "{speed_limit_mps: 0.0, reason: clear aisle}" \
  | rg "success=True|runtime_speed_limit_mps=0.0|effective_speed_limit_mps=0.6"

echo "dynamic speed limit passed"
