#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"
export GZ_PARTITION="${GZ_PARTITION:-robot_task_dashboard_${ROS_DOMAIN_ID}_$$}"

LOG_DIR="$(mktemp -d /tmp/robot_task_dashboard.XXXXXX)"
PIDS=()

cleanup() {
  ros2 service call /v2/unblock_station_route robot_interfaces_mission/srv/UnblockStationRoute \
    "{from_station: storage_a, to_station: packing}" >/dev/null 2>&1 || true
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

call_service() {
  local name="$1"
  local service="$2"
  local type="$3"
  local request="$4"
  if ! timeout -k 5s 35s ros2 service call "${service}" "${type}" "${request}" \
      >"${LOG_DIR}/${name}.out" 2>"${LOG_DIR}/${name}.err"; then
    echo "${service} call failed" >&2
    cat "${LOG_DIR}/${name}.out" >&2 2>/dev/null || true
    cat "${LOG_DIR}/${name}.err" >&2 2>/dev/null || true
    return 1
  fi
}

wait_dashboard_marker() {
  local timeout_sec="${1:-90}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/markers.txt"
  while true; do
    if timeout 8s ros2 topic echo --once --full-length \
        /amr_simulation/markers visualization_msgs/msg/MarkerArray \
        >"${sample}" 2>"${sample}.err" &&
      rg "AMR任务看板|task_board" "${sample}" >/dev/null &&
      rg "当前机器人 robot:" "${sample}" >/dev/null &&
      rg "当前任务 task:" "${sample}" >/dev/null &&
      rg "当前阶段 phase:" "${sample}" >/dev/null &&
      rg "目标站点 target:" "${sample}" >/dev/null &&
      rg "payload:" "${sample}" >/dev/null &&
      rg "dock:" "${sample}" >/dev/null &&
      rg "safety:" "${sample}" >/dev/null &&
      rg "traffic: [1-9]" "${sample}" >/dev/null &&
      rg "最近事件 events:" "${sample}" >/dev/null; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "task dashboard marker did not contain required AMR fields" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${sample}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

start_bg amr_demo ros2 launch robot_bringup amr_demo.launch.py gui:=false use_rviz:=false

wait_topic_type /amr_simulation/markers "visualization_msgs/msg/MarkerArray" 60 \
  "${LOG_DIR}/wait_markers.err"
wait_topic_type /navigate_sequence/current_goal "geometry_msgs/msg/PoseStamped" 90 \
  "${LOG_DIR}/wait_current_goal.err"
wait_topic_type /payload/state "robot_interfaces/msg/PayloadState" 60 \
  "${LOG_DIR}/wait_payload.err"
wait_topic_type /safety_state "robot_interfaces/msg/SafetyState" 60 \
  "${LOG_DIR}/wait_safety.err"
wait_service_type /v2/get_operator_snapshot "robot_interfaces_business/srv/GetOperatorSnapshot" 60 \
  "${LOG_DIR}/wait_snapshot.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 60 \
  "${LOG_DIR}/wait_events.err"
wait_service_type /v2/list_traffic_reservations "robot_interfaces_mission/srv/ListTrafficReservations" 60 \
  "${LOG_DIR}/wait_traffic.err"
wait_service_type /v2/list_docks "robot_interfaces_mission/srv/ListDocks" 60 \
  "${LOG_DIR}/wait_docks.err"
wait_service_type /v2/block_station_route "robot_interfaces_mission/srv/BlockStationRoute" 60 \
  "${LOG_DIR}/wait_block_route.err"

call_service snapshot /v2/get_operator_snapshot robot_interfaces_business/srv/GetOperatorSnapshot "{event_limit: 8}"
rg "success=True|operator snapshot|fleet_robot_ids=\\[|facility_resource_ids=\\[|recent_event_states=\\[" \
  "${LOG_DIR}/snapshot.out"

call_service block_route /v2/block_station_route robot_interfaces_mission/srv/BlockStationRoute \
  "{from_station: storage_a, to_station: packing, reason: task_dashboard_probe}"
rg "success=True|route_edge:storage_a__packing|route_edge:packing__storage_a" \
  "${LOG_DIR}/block_route.out"

wait_dashboard_marker 90

echo "AMR task dashboard showcase passed"
