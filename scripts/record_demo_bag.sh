#!/usr/bin/env bash
set -euo pipefail

ros2 bag record \
  /clock /tf /tf_static /scan /imu/data /odom /wheel/odom \
  /reference_path /tracking_error /task_status /robot_state \
  /chassis/state /cmd_vel /cmd_vel_mux/active_source
