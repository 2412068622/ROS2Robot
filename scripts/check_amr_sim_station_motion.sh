#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"

LOG_DIR="$(mktemp -d /tmp/robot_amr_sim_station_motion.XXXXXX)"
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

read_odom_field() {
  local field="$1"
  timeout 5s ros2 topic echo --once /model/mobile_robot/odometry \
    nav_msgs/msg/Odometry --field "${field}" 2>"${LOG_DIR}/odom_${field//./_}.err" \
    | rg -m 1 '^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$'
}

wait_finished_event() {
  local mission_id="$1"
  local timeout_sec="${2:-90}"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    output="$(
      ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
        "{limit: 20, state_filter: FINISHED, mission_id_filter: ${mission_id}}" \
        2>"${LOG_DIR}/list_events.err" || true
    )"
    if grep -q "success=True" <<<"${output}" &&
      grep -q "mission_ids=\\['${mission_id}'\\]" <<<"${output}" &&
      grep -q "states=\\['FINISHED'\\]" <<<"${output}"; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "mission did not finish: ${mission_id}" >&2
      echo "${output}" >&2
      return 1
    fi
    sleep 1
  done
}

start_bg amr_demo ros2 launch robot_bringup amr_demo.launch.py gui:=false use_rviz:=false auto_demo:=false

wait_topic_type /model/mobile_robot/odometry "nav_msgs/msg/Odometry" 30 \
  "${LOG_DIR}/wait_odom.err"
wait_service_type /v2/submit_order \
  "robot_interfaces_business/srv/SubmitOrder" 30 \
  "${LOG_DIR}/wait_submit.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 30 \
  "${LOG_DIR}/wait_events.err"

start_x="$(read_odom_field pose.pose.position.x)"
start_y="$(read_odom_field pose.pose.position.y)"

ros2 service call /v2/submit_order robot_interfaces_business/srv/SubmitOrder \
  "{order_id: sim_station_motion, order_type: station_transport, priority: 20, payload_json: 'pickup_station=receiving;dropoff_station=storage_b;start_if_idle=true;preempt_current=false', tags: [amr_sim_station_motion]}" \
  | rg "accepted=True|station_order_sim_station_motion"

wait_finished_event "station_order_sim_station_motion" 90

end_x="$(read_odom_field pose.pose.position.x)"
end_y="$(read_odom_field pose.pose.position.y)"

awk -v sx="${start_x}" -v sy="${start_y}" -v ex="${end_x}" -v ey="${end_y}" \
  'BEGIN {
     dx = ex - sx;
     dy = ey - sy;
     moved = sqrt(dx * dx + dy * dy);
     exit !(moved > 1.5 && ex < -1.2 && ey > 1.2)
   }' || {
  echo "station order did not move Gazebo robot to storage_b: start=(${start_x}, ${start_y}) end=(${end_x}, ${end_y})" >&2
  exit 1
}

echo "amr simulation station motion passed"
