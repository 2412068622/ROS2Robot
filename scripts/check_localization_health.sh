#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_localization_health.XXXXXX)"
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
  localization_covariance_threshold:=0.5

wait_topic_type /localization_health "robot_interfaces/msg/LocalizationHealth" 20 \
  "${LOG_DIR}/wait_localization_health.err"
wait_topic_type /initialpose "geometry_msgs/msg/PoseWithCovarianceStamped" 20 \
  "${LOG_DIR}/wait_initialpose.err"
wait_service_type /v2/request_relocalization "robot_interfaces_navigation/srv/RequestRelocalization" 20 \
  "${LOG_DIR}/wait_relocalization.err"
wait_service_type /v2/set_initial_pose_from_station "robot_interfaces_navigation/srv/SetInitialPoseFromStation" 20 \
  "${LOG_DIR}/wait_initial_pose_station.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 20 \
  "${LOG_DIR}/wait_events.err"

python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

rclpy.init()
node = rclpy.create_node("mock_amcl_high_covariance")
pub = node.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.pose.pose.orientation.w = 1.0
msg.pose.covariance[0] = 1.2
msg.pose.covariance[7] = 1.1
for _ in range(5):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
PY

deadline=$((SECONDS + 20))
while true; do
  health="$(
    ros2 topic echo --once /localization_health robot_interfaces/msg/LocalizationHealth \
      2>"${LOG_DIR}/localization_health.err" || true
  )"
  if grep -q "localized: false" <<<"${health}" && grep -q "state: LOST" <<<"${health}"; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "localization health did not enter LOST: ${health}" >&2
    exit 1
  fi
  sleep 1
done

ros2 service call /v2/request_relocalization robot_interfaces_navigation/srv/RequestRelocalization \
  "{reason: operator requested station reset}" \
  | rg "success=True|state='RELOCALIZING'|localized=False"

ros2 service call /v2/set_initial_pose_from_station robot_interfaces_navigation/srv/SetInitialPoseFromStation \
  "{station_id: dock}" \
  | rg "success=True|station_id='dock'|state='RECOVERED'|localized=True"

ros2 topic echo --once /localization_health robot_interfaces/msg/LocalizationHealth \
  | rg "localized: true|state: RECOVERED|station_id: dock"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: LOCALIZATION_LOST, mission_id_filter: localization}" \
  | rg "success=True|LOCALIZATION_LOST|covariance"

ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
  "{limit: 20, state_filter: LOCALIZATION_RECOVERED, mission_id_filter: localization}" \
  | rg "success=True|LOCALIZATION_RECOVERED|initial pose set from station dock"

echo "localization health passed"
