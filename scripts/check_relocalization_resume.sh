#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 20))}"

LOG_DIR="$(mktemp -d /tmp/robot_relocalization_resume.XXXXXX)"
MISSION_FILE="${LOG_DIR}/long_relocalization_mission.yaml"
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
mission_id: relocalization_resume_long
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
YAML

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

wait_service_type /start_mission_profile "std_srvs/srv/Trigger" 20 \
  "${LOG_DIR}/wait_start.err"
wait_service_type /v2/request_relocalization "robot_interfaces_navigation/srv/RequestRelocalization" 20 \
  "${LOG_DIR}/wait_relocalization.err"
wait_service_type /v2/set_initial_pose_from_station "robot_interfaces_navigation/srv/SetInitialPoseFromStation" 20 \
  "${LOG_DIR}/wait_initial_pose.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"
wait_topic_type /localization_health "robot_interfaces/msg/LocalizationHealth" 20 \
  "${LOG_DIR}/wait_health.err"
wait_topic_type /mission_runner/state "robot_interfaces/msg/RobotState" 20 \
  "${LOG_DIR}/wait_mission_state.err"

ros2 service call /start_mission_profile std_srvs/srv/Trigger "{}" \
  >"${LOG_DIR}/start.out"
rg "success=True" "${LOG_DIR}/start.out"

deadline=$((SECONDS + 10))
while true; do
  state="$(ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState --field state \
    2>"${LOG_DIR}/mission_running.err" || true)"
  if grep -q "RUNNING" <<<"${state}"; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "mission did not enter RUNNING before localization test: ${state}" >&2
    exit 1
  fi
  sleep 1
done

python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

rclpy.init()
node = rclpy.create_node("mock_amcl_relocalization_loss")
pub = node.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.pose.pose.orientation.w = 1.0
msg.pose.covariance[0] = 1.2
msg.pose.covariance[7] = 1.1
for _ in range(6):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
PY

deadline=$((SECONDS + 15))
while true; do
  state="$(ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState --field state \
    2>"${LOG_DIR}/mission_lost.err" || true)"
  if grep -q "LOCALIZATION_LOST\\|PAUSED" <<<"${state}"; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "mission did not pause on localization loss: ${state}" >&2
    exit 1
  fi
  sleep 1
done

ros2 topic echo --once /localization_health robot_interfaces/msg/LocalizationHealth \
  >"${LOG_DIR}/health_lost.out"
rg "localized: false" "${LOG_DIR}/health_lost.out"
rg "state: LOST" "${LOG_DIR}/health_lost.out"

ros2 service call /v2/request_relocalization robot_interfaces_navigation/srv/RequestRelocalization \
  "{reason: localization lost during active mission}" \
  >"${LOG_DIR}/request_relocalization.out"
rg "success=True" "${LOG_DIR}/request_relocalization.out"
rg "state='RELOCALIZING'" "${LOG_DIR}/request_relocalization.out"

ros2 service call /v2/set_initial_pose_from_station robot_interfaces_navigation/srv/SetInitialPoseFromStation \
  "{station_id: dock}" \
  >"${LOG_DIR}/set_initial_pose.out"
rg "success=True" "${LOG_DIR}/set_initial_pose.out"
rg "station_id='dock'" "${LOG_DIR}/set_initial_pose.out"
rg "state='RECOVERED'" "${LOG_DIR}/set_initial_pose.out"

deadline=$((SECONDS + 15))
while true; do
  state="$(ros2 topic echo --once /mission_runner/state robot_interfaces/msg/RobotState --field state \
    2>"${LOG_DIR}/mission_resumed.err" || true)"
  if grep -q "RUNNING\\|FINISHED" <<<"${state}"; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "mission did not resume after relocalization: ${state}" >&2
    exit 1
  fi
  sleep 1
done

ros2 topic echo --once /localization_health robot_interfaces/msg/LocalizationHealth \
  >"${LOG_DIR}/health_recovered.out"
rg "localized: true" "${LOG_DIR}/health_recovered.out"
rg "state: RECOVERED" "${LOG_DIR}/health_recovered.out"
rg "station_id: dock" "${LOG_DIR}/health_recovered.out"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 30, state_filter: LOCALIZATION_LOST, mission_id_filter: localization}" \
  >"${LOG_DIR}/lost_events.out"
rg "success=True" "${LOG_DIR}/lost_events.out"
rg "LOCALIZATION_LOST" "${LOG_DIR}/lost_events.out"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 30, state_filter: LOCALIZATION_RECOVERED, mission_id_filter: localization}" \
  >"${LOG_DIR}/recovered_events.out"
rg "success=True" "${LOG_DIR}/recovered_events.out"
rg "LOCALIZATION_RECOVERED" "${LOG_DIR}/recovered_events.out"
rg "initial pose set from station dock" "${LOG_DIR}/recovered_events.out"

echo "relocalization resume passed"
