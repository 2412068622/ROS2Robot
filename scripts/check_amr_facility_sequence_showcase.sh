#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"
export GZ_PARTITION="${GZ_PARTITION:-robot_facility_sequence_${ROS_DOMAIN_ID}_$$}"

LOG_DIR="$(mktemp -d /tmp/robot_facility_sequence.XXXXXX)"
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

call_service() {
  local name="$1"
  local service="$2"
  local type="$3"
  local request="$4"
  local attempt
  for attempt in 1 2 3; do
    if timeout -k 5s 35s ros2 service call "${service}" "${type}" "${request}" \
        >"${LOG_DIR}/${name}.out" 2>"${LOG_DIR}/${name}.err"; then
      return 0
    fi
    sleep 2
  done
  echo "${service} call failed after retries" >&2
  cat "${LOG_DIR}/${name}.out" >&2 2>/dev/null || true
  cat "${LOG_DIR}/${name}.err" >&2 2>/dev/null || true
  return 1
}

model_pose_xyz() {
  local model="$1"
  timeout 8s gz model -m "${model}" -p 2>"${LOG_DIR}/${model}.err" \
    | awk '/Pose \[ XYZ/ {getline; gsub(/[][]/, "", $0); print $1, $2, $3; exit}'
}

wait_model_pose() {
  local model="$1"
  local condition="$2"
  local timeout_sec="${3:-60}"
  local deadline=$((SECONDS + timeout_sec))
  local pose
  while true; do
    pose="$(model_pose_xyz "${model}" || true)"
    if [[ -n "${pose}" ]] &&
      awk -v x="$(awk '{print $1}' <<<"${pose}")" \
          -v y="$(awk '{print $2}' <<<"${pose}")" \
          -v z="$(awk '{print $3}' <<<"${pose}")" \
          "BEGIN { exit !(${condition}) }"; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "model ${model} did not satisfy '${condition}', last pose: ${pose}" >&2
      cat "${LOG_DIR}/${model}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

start_bg amr_demo ros2 launch robot_bringup amr_demo.launch.py \
  gui:=false \
  use_rviz:=false \
  auto_demo:=false

wait_service_type /world/indoor_room/set_pose "ros_gz_interfaces/srv/SetEntityPose" 40 \
  "${LOG_DIR}/wait_set_pose.err"
wait_service_type /v2/execute_facility_action "robot_interfaces_facility/srv/ExecuteFacilityAction" 40 \
  "${LOG_DIR}/wait_execute_facility.err"
wait_service_type /v2/release_facility_resource "robot_interfaces_facility/srv/ReleaseFacilityResource" 40 \
  "${LOG_DIR}/wait_release_facility.err"
wait_service_type /v2/request_lift_session "robot_interfaces_facility/srv/RequestLiftSession" 40 \
  "${LOG_DIR}/wait_request_lift.err"
wait_service_type /v2/release_lift_session "robot_interfaces_facility/srv/ReleaseLiftSession" 40 \
  "${LOG_DIR}/wait_release_lift.err"
wait_service_type /v2/get_door_state "robot_interfaces_facility/srv/GetDoorState" 40 \
  "${LOG_DIR}/wait_door_state.err"
wait_service_type /v2/get_lift_state "robot_interfaces_facility/srv/GetLiftState" 40 \
  "${LOG_DIR}/wait_lift_state.err"
wait_service_type /v2/cancel_queued_mission "robot_interfaces_mission/srv/CancelQueuedMission" 40 \
  "${LOG_DIR}/wait_cancel.err"

call_service door_wait_pose /world/indoor_room/set_pose \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: mobile_robot, type: 2}, pose: {position: {x: -4.15, y: -0.65, z: 0.24}, orientation: {w: 1.0}}}"
rg "success=True" "${LOG_DIR}/door_wait_pose.out"
wait_model_pose "mobile_robot" "x < -3.8 && y < -0.4 && y > -0.9 && z > 0.05" 40

call_service door_pass /v2/execute_facility_action \
  robot_interfaces_facility/srv/ExecuteFacilityAction \
  "{request_id: facility_door_pass_showcase, resource_id: receiving_door, resource_type: door, action: pass, priority: 31, start_if_idle: false, preempt_current: false, hold_after_action: true}"
rg "success=True" "${LOG_DIR}/door_pass.out"
rg "facility_action_facility_door_pass_showcase" "${LOG_DIR}/door_pass.out"

call_service door_state_reserved /v2/get_door_state \
  robot_interfaces_facility/srv/GetDoorState \
  "{door_id: receiving_door}"
rg "success=True" "${LOG_DIR}/door_state_reserved.out"
rg "holder_id='facility_action_facility_door_pass_showcase'" "${LOG_DIR}/door_state_reserved.out"
rg "status='reserved'" "${LOG_DIR}/door_state_reserved.out"
wait_model_pose "receiving_door_panel" "y > 0.35" 40

call_service door_pass_pose /world/indoor_room/set_pose \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: mobile_robot, type: 2}, pose: {position: {x: -3.45, y: 0.65, z: 0.24}, orientation: {w: 1.0}}}"
rg "success=True" "${LOG_DIR}/door_pass_pose.out"
wait_model_pose "mobile_robot" "x > -3.8 && x < -3.0 && y > 0.35 && y < 0.95 && z > 0.05" 40

call_service release_door /v2/release_facility_resource \
  robot_interfaces_facility/srv/ReleaseFacilityResource \
  "{resource_id: receiving_door, holder_id: facility_action_facility_door_pass_showcase}"
rg "success=True" "${LOG_DIR}/release_door.out"

call_service cancel_door_mission /v2/cancel_queued_mission \
  robot_interfaces_mission/srv/CancelQueuedMission \
  "{mission_id: facility_action_facility_door_pass_showcase, cancel_active: false}"
rg "success=True" "${LOG_DIR}/cancel_door_mission.out"

call_service door_state_released /v2/get_door_state \
  robot_interfaces_facility/srv/GetDoorState \
  "{door_id: receiving_door}"
rg "success=True" "${LOG_DIR}/door_state_released.out"
rg "available=True" "${LOG_DIR}/door_state_released.out"
wait_model_pose "receiving_door_panel" "y < 0.15" 40

call_service lift_wait_pose /world/indoor_room/set_pose \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: mobile_robot, type: 2}, pose: {position: {x: 3.65, y: 2.05, z: 0.24}, orientation: {w: 1.0}}}"
rg "success=True" "${LOG_DIR}/lift_wait_pose.out"
wait_model_pose "mobile_robot" "x > 3.2 && x < 4.1 && y > 1.75 && y < 2.35 && z > 0.05" 40

call_service request_lift /v2/request_lift_session \
  robot_interfaces_facility/srv/RequestLiftSession \
  "{session_id: facility_lift_showcase, lift_id: freight_elevator, requester_id: facility_showcase_robot, from_station: lift_lobby, to_station: floor_2_dropoff, release_existing_for_requester: true}"
rg "success=True" "${LOG_DIR}/request_lift.out"
rg "status='lift_session'" "${LOG_DIR}/request_lift.out"

call_service lift_state_reserved /v2/get_lift_state \
  robot_interfaces_facility/srv/GetLiftState \
  "{lift_id: freight_elevator}"
rg "success=True" "${LOG_DIR}/lift_state_reserved.out"
rg "holder_id='facility_showcase_robot'" "${LOG_DIR}/lift_state_reserved.out"
rg "status='lift_session'" "${LOG_DIR}/lift_state_reserved.out"
wait_model_pose "freight_elevator_cabin" "z > 0.55" 40

call_service lift_enter_pose /world/indoor_room/set_pose \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: mobile_robot, type: 2}, pose: {position: {x: 3.65, y: 2.85, z: 0.24}, orientation: {w: 1.0}}}"
rg "success=True" "${LOG_DIR}/lift_enter_pose.out"
wait_model_pose "mobile_robot" "x > 3.25 && x < 4.05 && y > 2.45 && y < 3.20 && z > 0.05" 40

call_service lift_exit_pose /world/indoor_room/set_pose \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: mobile_robot, type: 2}, pose: {position: {x: 3.65, y: 3.55, z: 0.24}, orientation: {w: 1.0}}}"
rg "success=True" "${LOG_DIR}/lift_exit_pose.out"
wait_model_pose "mobile_robot" "x > 3.25 && x < 4.05 && y > 3.25 && y < 3.85 && z > 0.05" 40

call_service release_lift /v2/release_lift_session \
  robot_interfaces_facility/srv/ReleaseLiftSession \
  "{session_id: facility_lift_showcase, lift_id: freight_elevator}"
rg "success=True" "${LOG_DIR}/release_lift.out"
rg "status='available'" "${LOG_DIR}/release_lift.out"

call_service lift_state_released /v2/get_lift_state \
  robot_interfaces_facility/srv/GetLiftState \
  "{lift_id: freight_elevator}"
rg "success=True" "${LOG_DIR}/lift_state_released.out"
rg "available=True" "${LOG_DIR}/lift_state_released.out"

echo "AMR facility sequence showcase passed"
