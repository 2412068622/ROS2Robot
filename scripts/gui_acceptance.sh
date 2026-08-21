#!/usr/bin/env bash
set -euo pipefail

duration="${1:-30}"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "No DISPLAY or WAYLAND_DISPLAY found; GUI acceptance requires a desktop session." >&2
  echo "Headless alternatives: display.launch.py use_rviz:=false and sim.launch.py gui:=false" >&2
  exit 2
fi

set +u
source install/setup.bash
set -u

echo "Running RViz2 model acceptance for ${duration}s"
timeout "${duration}s" ros2 launch robot_bringup display.launch.py use_rviz:=true || {
  status=$?
  if [[ "${status}" != "124" ]]; then
    exit "${status}"
  fi
}

echo "Running Gazebo GUI acceptance for ${duration}s"
timeout "${duration}s" ros2 launch robot_simulation sim.launch.py gui:=true || {
  status=$?
  if [[ "${status}" != "124" ]]; then
    exit "${status}"
  fi
}
