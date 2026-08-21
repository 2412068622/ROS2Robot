#!/usr/bin/env bash
set -euo pipefail

ros2 service type /v2/set_cmd_source >/dev/null
ros2 service type /clear_odometry >/dev/null
ros2 topic info /cmd_vel -v | rg "Publisher count: 1"
ros2 topic type /system_health | rg "robot_interfaces/msg/RobotState"
ros2 topic type /diagnostics | rg "diagnostic_msgs/msg/DiagnosticArray"
ros2 topic echo --once /system_health robot_interfaces/msg/RobotState --field state | rg "OK|WARN|ERROR"
