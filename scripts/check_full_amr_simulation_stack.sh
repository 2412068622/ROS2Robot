#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

BASE_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 120 + 20))}"
LOG_DIR="${1:-$(mktemp -d /tmp/robot_full_amr_simulation.XXXXXX)}"
COOLDOWN_S="${AMR_SIM_STACK_COOLDOWN_S:-4}"
CHECK_ATTEMPTS="${AMR_SIM_STACK_ATTEMPTS:-2}"

mkdir -p "${LOG_DIR}"

cleanup_known_processes() {
  local patterns=(
    "gz sim .*indoor_room_.*\\.sdf"
    "ros_gz_bridge parameter_bridge"
    "ros_gz_bridge/parameter_bridge"
    "amr_sim_visualizer_node"
    "amr_sim_gazebo_entity_bridge_node"
    "amr_sim_demo_director_node"
    "navigate_sequence_server_node"
    "mission_runner_node"
    "async_slam_toolbox_node"
    "nav2_controller/controller_server"
    "nav2_planner/planner_server"
    "nav2_behaviors/behavior_server"
    "nav2_bt_navigator/bt_navigator"
    "nav2_lifecycle_manager/lifecycle_manager"
    "nav2_map_server/map_server"
    "nav2_amcl/amcl"
  )
  local pattern
  local pid
  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      if [[ -n "${pid}" && "${pid}" != "$$" ]]; then
        kill "${pid}" >/dev/null 2>&1 || true
      fi
    done < <(pgrep -f "${pattern}" || true)
  done
  sleep "${COOLDOWN_S}"
}

run_check() {
  local index="$1"
  local name="$2"
  local script="$3"
  local domain_id=$((BASE_DOMAIN_ID + index))
  local attempt
  local log_file

  for attempt in $(seq 1 "${CHECK_ATTEMPTS}"); do
    cleanup_known_processes
    log_file="${LOG_DIR}/${index}_${name}_attempt${attempt}.log"
    echo "[${index}] ${name}: start attempt ${attempt}/${CHECK_ATTEMPTS}, ROS_DOMAIN_ID=${domain_id}"
    if ROS_DOMAIN_ID="${domain_id}" bash "${SCRIPT_DIR}/${script}" >"${log_file}" 2>&1; then
      echo "[${index}] ${name}: passed attempt ${attempt}/${CHECK_ATTEMPTS}"
      cleanup_known_processes
      return 0
    fi

    echo "[${index}] ${name}: failed attempt ${attempt}/${CHECK_ATTEMPTS}; log=${log_file}" >&2
    tail -n 120 "${log_file}" >&2 || true
    cleanup_known_processes
  done
  return 1
}

run_check 1 gazebo_sensor_stack check_gazebo_sensor_stack.sh
run_check 2 gazebo_slam_runtime check_gazebo_slam_runtime.sh
run_check 3 gazebo_nav2_runtime check_gazebo_nav2_runtime.sh
run_check 4 amr_default_demo check_amr_sim_default_demo.sh
run_check 5 amr_gazebo_entities check_amr_sim_gazebo_entities.sh
run_check 6 amr_station_motion check_amr_sim_station_motion.sh

echo "Full AMR simulation stack passed"
echo "Logs: ${LOG_DIR}"
