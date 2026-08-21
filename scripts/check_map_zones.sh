#!/usr/bin/env bash
set -euo pipefail

ros2 service type /v2/list_map_zones | rg "robot_interfaces_navigation/srv/ListMapZones"
ros2 service type /v2/evaluate_map_position | rg "robot_interfaces_navigation/srv/EvaluateMapPosition"
ros2 topic type /map_manager/zones | rg "std_msgs/msg/String"
ros2 service call /v2/list_map_zones robot_interfaces_navigation/srv/ListMapZones "{map_name: indoor_room, include_disabled: false}" | rg "dock_slow_zone|charging_keepout"
ros2 service call /v2/evaluate_map_position robot_interfaces_navigation/srv/EvaluateMapPosition "{map_name: indoor_room, point: {x: -1.0, y: -1.0, z: 0.0}}" | rg "speed_limit_mps=0.18|speed_limit_mps: 0.18"
ros2 service call /v2/evaluate_map_position robot_interfaces_navigation/srv/EvaluateMapPosition "{map_name: indoor_room, point: {x: -2.0, y: 1.8, z: 0.0}}" | rg "allowed=False|allowed: false"
