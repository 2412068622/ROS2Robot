#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_obstacle_blockage.XXXXXX)"
MISSION_FILE="${LOG_DIR}/long_obstacle_mission.yaml"
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

cat >"${MISSION_FILE}" <<'YAML'
mission_id: obstacle_blockage_long
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
YAML

start_bg navigate_sequence ros2 run robot_tasks navigate_sequence_server_node --ros-args \
  -p use_nav2_action:=false \
  -p simulate_without_nav2:=true
start_bg mission_runner ros2 launch robot_tasks mission_runner.launch.py \
  mission_file:="${MISSION_FILE}" \
  autostart:=false \
  server_timeout_ms:=3000 \
  return_to_dock_on_low_battery:=false \
  safety_obstacle_pause_on_blockage:=true

wait_service_type /start_mission_profile "std_srvs/srv/Trigger" 20 "${LOG_DIR}/wait_start.err"
wait_service_type /v2/report_obstacle_blockage "robot_interfaces_navigation/srv/ReportObstacleBlockage" 20 \
  "${LOG_DIR}/wait_report.err"
wait_service_type /v2/clear_obstacle_blockage "robot_interfaces_navigation/srv/ClearObstacleBlockage" 20 \
  "${LOG_DIR}/wait_clear.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"
wait_topic_type /safety_state "robot_interfaces/msg/SafetyState" 20 "${LOG_DIR}/wait_safety.err"

ros2 service call /start_mission_profile std_srvs/srv/Trigger "{}" | rg "success=True"

ros2 service call /v2/report_obstacle_blockage robot_interfaces_navigation/srv/ReportObstacleBlockage \
  "{blockage_id: pallet_aisle, reason: pallet blocking aisle, pause_active: true}" \
  | rg "success=True|state='OBSTACLE_BLOCKED'|blockage_id='pallet_aisle'"

ros2 topic echo --once /safety_state robot_interfaces/msg/SafetyState \
  | rg "obstacle_blocked: true|safety_stop: true|state: OBSTACLE_BLOCKED|blockage_id: pallet_aisle"

ros2 service call /v2/report_obstacle_blockage robot_interfaces_navigation/srv/ReportObstacleBlockage \
  "{blockage_id: fallen_box, reason: second obstacle should not overwrite active pause, pause_active: false}" \
  | rg "success=False|obstacle blockage already active: pallet_aisle|blockage_id='pallet_aisle'"

ros2 topic echo --once /safety_state robot_interfaces/msg/SafetyState \
  | rg "obstacle_blocked: true|safety_stop: true|state: OBSTACLE_BLOCKED|blockage_id: pallet_aisle"

deadline=$((SECONDS + 10))
while true; do
  state="$(ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState --field state \
    2>"${LOG_DIR}/mission_state.err" || true)"
  if grep -q "SAFETY_BLOCKED\\|PAUSED" <<<"${state}"; then
    echo "${state}"
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "mission runner did not enter safety blocked state: ${state}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/clear_obstacle_blockage robot_interfaces_navigation/srv/ClearObstacleBlockage \
  "{blockage_id: pallet_aisle, resolution: operator removed pallet, resume_active: true}" \
  | rg "success=True|state='OK'"

ros2 topic echo --once /safety_state robot_interfaces/msg/SafetyState \
  | rg "obstacle_blocked: false|safety_stop: false|state: OK"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: OBSTACLE_BLOCKED, mission_id_filter: safety_pallet_aisle}" \
  | rg "success=True|OBSTACLE_BLOCKED|pallet blocking aisle"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: OBSTACLE_CLEARED, mission_id_filter: safety_pallet_aisle}" \
  | rg "success=True|OBSTACLE_CLEARED|operator removed pallet"

echo "obstacle blockage recovery passed"
