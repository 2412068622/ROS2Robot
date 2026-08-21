#!/usr/bin/env bash
set -euo pipefail

name="${1:-indoor_room}"
output_dir="${2:-src/robot_navigation/maps}"
map_topic="${3:-/map}"
save_timeout_sec="${4:-10.0}"
base="${output_dir}/${name}"

mkdir -p "${output_dir}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not available in PATH" >&2
  exit 1
fi

echo "Saving map from ${map_topic} to ${base}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -t "${map_topic}" -f "${base}" --ros-args -p save_map_timeout:="${save_timeout_sec}"
"$(dirname "$0")/validate_map.sh" "${base}.yaml"
