#!/usr/bin/env bash
set -euo pipefail

backend="${1:-mock}"
duration="${2:-20}"
dev="${ROBOT_SERIAL_DEVICE:-/dev/ttyUSB0}"
baud="${ROBOT_SERIAL_BAUD:-115200}"
ip="${ROBOT_UDP_HOST:-192.168.1.10}"
port="${ROBOT_UDP_PORT:-9000}"

set +u
source install/setup.bash
set -u
source scripts/robot_wait.sh

case "${backend}" in
  mock)
    args=(backend:=mock)
    ;;
  serial)
    if [[ ! -e "${dev}" ]]; then
      echo "serial device not found: ${dev}" >&2
      exit 2
    fi
    args=(backend:=serial dev:="${dev}" baud:="${baud}")
    ;;
  udp)
    args=(backend:=udp ip:="${ip}" port:="${port}")
    ;;
  *)
    echo "usage: scripts/hardware_acceptance.sh [mock|serial|udp] [duration_seconds]" >&2
    exit 1
    ;;
esac

log="/tmp/robot_hardware_acceptance_${backend}.log"
echo "Running hardware acceptance: backend=${backend}, duration=${duration}s"
ros2 launch robot_bringup bringup.launch.py mode:=hardware "${args[@]}" >"${log}" 2>&1 &
launch_pid=$!
cleanup() {
  if kill -0 "${launch_pid}" >/dev/null 2>&1; then
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 2
if ! kill -0 "${launch_pid}" >/dev/null 2>&1; then
  tail -120 "${log}" >&2
  exit 1
fi

wait_topic_sample /chassis/state robot_interfaces/msg/ChassisState 15 /tmp/robot_hardware_acceptance_chassis_state.err
wait_topic_sample /odom nav_msgs/msg/Odometry 15 /tmp/robot_hardware_acceptance_odom.err
wait_service_type /v2/set_chassis_mode robot_interfaces_core/srv/SetChassisMode 15 /tmp/robot_hardware_acceptance_set_mode.err
sleep "${duration}"

echo "Hardware acceptance entrypoints OK for backend=${backend}; log=${log}"
