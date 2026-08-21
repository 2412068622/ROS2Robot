#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

if [ ! -f "${ROOT_DIR}/install/setup.bash" ]; then
  echo "install/setup.bash not found; run colcon build --symlink-install first" >&2
  exit 1
fi

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

RESULTS_DIR="${1:-${ROOT_DIR}/src/robot_experiments/results}"
SAMPLE_SECONDS="${2:-8}"
NAV_PARAMS="${ROOT_DIR}/src/robot_navigation/config/nav2_advanced.yaml"
MAP_FILE="${ROOT_DIR}/src/robot_navigation/maps/indoor_room.yaml"
STAMP="$(date +%Y%m%d_%H%M%S)"
ISO_STAMP="$(date -Iseconds)"
CSV_FILE="${RESULTS_DIR}/nav2_advanced_${STAMP}.csv"
JSON_FILE="${RESULTS_DIR}/nav2_advanced_${STAMP}.json"
LOG_DIR="$(mktemp -d /tmp/robot_nav2_advanced_experiment.XXXXXX)"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

start_bg() {
  local name="$1"
  shift
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

mkdir -p "${RESULTS_DIR}"

start_bg static_lidar_tf ros2 run tf2_ros static_transform_publisher \
  0.18 0 0.27 0 0 0 base_footprint lidar_link
start_bg static_map_tf ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom
start_bg cmd_vel_mux ros2 run robot_teleop cmd_vel_mux_node --ros-args \
  -p default_source:=nav2 -p watchdog_timeout_ms:=500
start_bg chassis_driver ros2 run robot_hardware chassis_driver_node --ros-args \
  -p backend:=mock -p publish_tf:=true
start_bg fake_scan ros2 run robot_sensors fake_scan_node --ros-args \
  -p topic:=/scan
start_bg nav2_advanced ros2 launch robot_navigation nav.launch.py \
  "params_file:=${NAV_PARAMS}" "map:=${MAP_FILE}" use_sim_time:=false navigation_start_delay:=5.0

wait_topic_type /scan "sensor_msgs/msg/LaserScan" 20 "${LOG_DIR}/wait_scan.err"
wait_topic_type /odom "nav_msgs/msg/Odometry" 20 "${LOG_DIR}/wait_odom.err"
wait_topic_type /map "nav_msgs/msg/OccupancyGrid" 40 "${LOG_DIR}/wait_map.err"

for service in \
  /map_server/get_state \
  /amcl/get_state \
  /controller_server/get_state \
  /planner_server/get_state \
  /behavior_server/get_state \
  /bt_navigator/get_state; do
  node_name="$(basename "$(dirname "${service}")")"
  wait_service_type "${service}" "lifecycle_msgs/srv/GetState" 45 "${LOG_DIR}/${node_name}_service.err"
  wait_lifecycle_active "${service}" 45 "${LOG_DIR}/${node_name}_state.err"
done

wait_topic_type /nav2_cmd_vel "geometry_msgs/msg/Twist" 20 "${LOG_DIR}/wait_nav2_cmd_vel.err"
wait_action_type /navigate_to_pose "nav2_msgs/action/NavigateToPose" 30 "${LOG_DIR}/wait_nav_action.err"

sleep "${SAMPLE_SECONDS}"

cat >"${CSV_FILE}" <<CSV
timestamp,profile,map_file,params_file,scan_topic,odom_topic,map_topic,nav2_cmd_vel_topic,navigate_to_pose_action,map_server,amcl,controller_server,planner_server,behavior_server,bt_navigator,sample_seconds
${ISO_STAMP},advanced,${MAP_FILE},${NAV_PARAMS},available,available,available,available,available,active,active,active,active,active,active,${SAMPLE_SECONDS}
CSV

cat >"${JSON_FILE}" <<JSON
{
  "timestamp": "${ISO_STAMP}",
  "profile": "advanced",
  "map_file": "${MAP_FILE}",
  "params_file": "${NAV_PARAMS}",
  "sample_seconds": ${SAMPLE_SECONDS},
  "topics": {
    "/scan": "available",
    "/odom": "available",
    "/map": "available",
    "/nav2_cmd_vel": "available"
  },
  "actions": {
    "/navigate_to_pose": "available"
  },
  "lifecycle": {
    "map_server": "active",
    "amcl": "active",
    "controller_server": "active",
    "planner_server": "active",
    "behavior_server": "active",
    "bt_navigator": "active"
  },
  "scope": "headless mock advanced Nav2 profile readiness experiment; no GUI, real hardware, or real-world SLAM was used"
}
JSON

echo "CSV=${CSV_FILE}"
echo "JSON=${JSON_FILE}"
