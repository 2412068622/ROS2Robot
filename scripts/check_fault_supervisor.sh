#!/usr/bin/env bash
set -eo pipefail

set +u
source install/setup.bash
set -u

mux_log="/tmp/robot_fault_supervisor_mux.log"
supervisor_log="/tmp/robot_fault_supervisor.log"
ros2 run robot_teleop cmd_vel_mux_node >"${mux_log}" 2>&1 &
mux_pid=$!
ros2 run robot_utils fault_supervisor_node --ros-args -p startup_grace_ms:=0 -p command_cooldown_ms:=50 >"${supervisor_log}" 2>&1 &
supervisor_pid=$!

cleanup() {
  kill "${supervisor_pid}" "${mux_pid}" >/dev/null 2>&1 || true
  wait "${supervisor_pid}" "${mux_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

wait_for_service() {
  local service_name="$1"
  local deadline=$((SECONDS + 10))
  while ! ros2 service list 2>/dev/null | grep -qx "${service_name}"; do
    if (( SECONDS >= deadline )); then
      echo "service did not appear: ${service_name}" >&2
      exit 1
    fi
    sleep 1
  done
}

wait_for_estop() {
  local expected="$1"
  local deadline=$((SECONDS + 10))
  while true; do
    local state
    state="$(ros2 topic echo --once /emergency_stop/state std_msgs/msg/Bool --field data 2>/tmp/robot_fault_estop.err || true)"
    if grep -qi "${expected}" <<<"${state}"; then
      echo "${state}"
      return
    fi
    if (( SECONDS >= deadline )); then
      echo "emergency stop did not become ${expected}; last state: ${state}" >&2
      exit 1
    fi
    sleep 1
  done
}

wait_for_service "/enable_emergency_stop"
wait_for_service "/clear_emergency_stop"
ros2 topic type /fault_supervisor/state | rg "robot_interfaces/msg/RobotState"

ros2 topic pub --once /system_health robot_interfaces/msg/RobotState "{state: ERROR, message: injected fault, recoverable: false}" >/tmp/robot_fault_inject.log 2>&1
wait_for_estop "true"

ros2 topic pub --once /system_health robot_interfaces/msg/RobotState "{state: OK, message: recovered, recoverable: true}" >/tmp/robot_fault_recover.log 2>&1
wait_for_estop "false"

echo "Fault supervisor acceptance OK; logs: ${supervisor_log}, ${mux_log}"

