#!/usr/bin/env bash
set -euo pipefail

ros2 service type /v2/list_maps | rg "robot_interfaces_navigation/srv/ListMaps"
ros2 service type /v2/set_active_map | rg "robot_interfaces_navigation/srv/SetActiveMap"
ros2 topic type /map_manager/state | rg "robot_interfaces/msg/RobotState"
ros2 topic type /map_manager/catalog | rg "std_msgs/msg/String"
ros2 topic type /map_manager/zones | rg "std_msgs/msg/String"
ros2 service call /v2/list_maps robot_interfaces_navigation/srv/ListMaps "{}" | rg "indoor_room"
ros2 service call /v2/set_active_map robot_interfaces_navigation/srv/SetActiveMap "{name: indoor_room}" | rg "success=True|success: true"
ros2 topic echo --once /map_manager/state robot_interfaces/msg/RobotState --field state | rg "OK|WARN|ERROR"
