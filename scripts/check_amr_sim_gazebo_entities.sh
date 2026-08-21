#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 200 + 1))}"
export GZ_PARTITION="${GZ_PARTITION:-robot_amr_entities_${ROS_DOMAIN_ID}_$$}"

LOG_DIR="$(mktemp -d /tmp/robot_amr_sim_gazebo_entities.XXXXXX)"
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
      echo "model ${model} did not satisfy pose condition '${condition}', last pose: ${pose}" >&2
      cat "${LOG_DIR}/${model}.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

wait_models_exist() {
  local timeout_sec="$1"
  shift
  local models=("$@")
  local deadline=$((SECONDS + timeout_sec))
  local pose
  while true; do
    local visible_count=0
    for model in "${models[@]}"; do
      pose="$(model_pose_xyz "${model}" || true)"
      if [[ -n "${pose}" ]]; then
        visible_count=$((visible_count + 1))
      fi
    done
    if ((visible_count == ${#models[@]})); then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "Gazebo showcase models did not appear; expected ${#models[@]}, got ${visible_count}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_label_models() {
  local timeout_sec="${1:-60}"
  local deadline=$((SECONDS + timeout_sec))
  local labels=(
    label_robot_1
    label_robot_2
    label_dock
    label_receiving
    label_door
    label_elevator
    label_floor_2
    label_storage_a
    label_storage_b
    label_packing
    label_traffic
    label_slow_zone
    label_confirm
    label_payload
  )
  local pose
  while true; do
    local visible_count=0
    for label in "${labels[@]}"; do
      pose="$(model_pose_xyz "${label}" || true)"
      if [[ -n "${pose}" ]] &&
        awk -v z="$(awk '{print $3}' <<<"${pose}")" 'BEGIN { exit !(z > 0.5) }'; then
        visible_count=$((visible_count + 1))
      fi
    done
    if ((visible_count == ${#labels[@]})); then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "Gazebo label models did not appear; expected ${#labels[@]}, got ${visible_count}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_station_finished() {
  local timeout_sec="${1:-180}"
  local deadline=$((SECONDS + timeout_sec))
  local sample="${LOG_DIR}/station_events.txt"
  local pose
  while true; do
    pose="$(model_pose_xyz "payload_box_waiting" || true)"
    if [[ -n "${pose}" ]] &&
      awk -v x="$(awk '{print $1}' <<<"${pose}")" \
          -v y="$(awk '{print $2}' <<<"${pose}")" \
          -v z="$(awk '{print $3}' <<<"${pose}")" \
          'BEGIN { exit !(x < -1.0 && y > 1.0 && z > 0.1 && z < 0.4) }'; then
      return 0
    fi
    if timeout 6s ros2 service call /v2/list_mission_events robot_interfaces_mission/srv/ListMissionEvents \
        "{limit: 10, state_filter: FINISHED, mission_id_filter: station_order_sim_auto_transport}" \
        >"${sample}" 2>"${LOG_DIR}/station_events.err" &&
      rg "mission_ids=\\['station_order_sim_auto_transport'\\]" "${sample}" >/dev/null &&
      rg "states=\\['FINISHED'\\]" "${sample}" >/dev/null; then
      return 0
    fi
    if ((SECONDS >= deadline)); then
      echo "station transport did not finish in default demo" >&2
      cat "${sample}" >&2 2>/dev/null || true
      cat "${LOG_DIR}/station_events.err" >&2 2>/dev/null || true
      return 1
    fi
    sleep 2
  done
}

start_bg amr_demo ros2 launch robot_bringup amr_demo.launch.py gui:=false use_rviz:=false

wait_service_type /world/indoor_room/set_pose "ros_gz_interfaces/srv/SetEntityPose" 30 \
  "${LOG_DIR}/wait_set_pose.err"
wait_service_type /v2/list_mission_events "robot_interfaces_mission/srv/ListMissionEvents" 30 \
  "${LOG_DIR}/wait_events.err"

wait_label_models 60
wait_models_exist 60 \
  showcase_zone_floor \
  showcase_lane_markers \
  showcase_station_pads \
  showcase_pallets_and_totes \
  showcase_dock_guidance
wait_model_pose "manual_takeover_indicator" "z > 0.5" 60
wait_model_pose "label_manual_takeover_status" "z > 1.0" 60
wait_model_pose "payload_box_waiting" "(z > 0.55) || (x < -1.0 && y > 1.0 && z > 0.1 && z < 0.4)" 60
wait_model_pose "operator_confirmation_panel" "x < -3.0 && y > 0.8 && z > 0.35" 60
wait_model_pose "label_operator_confirmation_status" "x < -3.0 && y > 0.8 && z > 0.8" 60
wait_model_pose "top_module_status_indicator" "z > 0.7" 60
wait_model_pose "label_top_module_status" "z > 1.2" 60
wait_model_pose "payload_lock_indicator" "z > 0.9" 80
wait_model_pose "receiving_door_panel" "y > 0.35" 60
wait_model_pose "freight_elevator_cabin" "z > 0.55" 60
wait_model_pose "localization_status_beacon" "z > 1.0" 80
wait_model_pose "label_localization_status" "z > 1.3" 80
wait_model_pose "traffic_route_block_barrier" "((x > -2.0 && x < 0.0 && y > 0.5) || (x > 1.2 && x < 2.2 && y > -0.5 && y < 1.0)) && z > 0.15" 60
wait_model_pose "label_traffic_block_status" "((x > -2.0 && x < 0.0 && y > 0.5) || (x > 1.2 && x < 2.2 && y > -0.5 && y < 1.0)) && z > 0.7" 60
wait_model_pose "traffic_deadlock_warning_beacon" "((x > -2.0 && x < 0.0 && y > 0.5) || (x > 1.2 && x < 2.2 && y > -0.5 && y < 1.0)) && z > 0.8" 60
wait_model_pose "label_traffic_deadlock_status" "((x > -2.0 && x < 0.0 && y > 0.5) || (x > 1.2 && x < 2.2 && y > -0.5 && y < 1.0)) && z > 1.2" 60
wait_model_pose "speed_limit_zone_indicator" "z > 0.0" 60
wait_model_pose "label_speed_limit_status" "z > 0.5" 60
wait_model_pose "docking_retry_warning_beacon" "z > 0.8" 240
wait_model_pose "label_docking_retry_status" "z > 1.2" 240
wait_model_pose "charging_status_beacon" "z > 1.0" 240
wait_model_pose "label_charging_status" "z > 1.3" 240
wait_station_finished 180
wait_model_pose "payload_box_waiting" "x < -1.0 && y > 1.0 && z > 0.1 && z < 0.4" 90
wait_model_pose "traffic_yield_wait_marker" "x > -1.6 && x < -0.4 && y < -3.6 && z > 0.1" 120
wait_model_pose "traffic_release_go_marker" "x > 1.2 && x < 2.2 && y > -0.5 && y < 1.0 && z > 0.2" 120
wait_model_pose "mobile_robot_2" "x > 1.0 && y < -0.5" 90
wait_model_pose "opportunity_charge_beacon" "z > 0.8" 120
wait_model_pose "label_opportunity_charge_status" "z > 1.2" 120
wait_model_pose "safety_obstacle_block" "z > 0.2" 90
wait_model_pose "label_obstacle_status" "z > 0.8" 90
wait_model_pose "obstacle_clearance_indicator" "z > 0.7" 120
wait_model_pose "label_obstacle_clear_status" "z > 1.0" 120
wait_model_pose "traffic_route_reopen_beacon" "((x > -2.0 && x < 0.0 && y > 0.5) || (x > 1.2 && x < 2.2 && y > -0.5 && y < 1.0)) && z > 0.8" 120
wait_model_pose "label_traffic_reopen_status" "((x > -2.0 && x < 0.0 && y > 0.5) || (x > 1.2 && x < 2.2 && y > -0.5 && y < 1.0)) && z > 1.0" 120
wait_model_pose "speed_limit_restored_beacon" "z > 0.7" 120
wait_model_pose "label_speed_restored_status" "z > 1.0" 120

echo "amr simulation Gazebo dynamic entities passed"
