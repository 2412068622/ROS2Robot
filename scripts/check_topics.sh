#!/usr/bin/env bash
set -euo pipefail

required_topics=(
  /scan
  /imu/data
  /odom
  /cmd_vel
  /cmd_vel_mux/active_source
  /emergency_stop/state
  /task_status
  /robot_state
  /chassis/state
)

for topic in "${required_topics[@]}"; do
  ros2 topic info "$topic" >/dev/null
done
