#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_service_type /v2/estimate_mission_cost "robot_interfaces_mission/srv/EstimateMissionCost" 10 /tmp/robot_estimate_mission_service.err
wait_service_type /v2/estimate_station_transport "robot_interfaces_mission/srv/EstimateStationTransport" 10 /tmp/robot_estimate_station_service.err
wait_service_type /v2/list_station_routes "robot_interfaces_mission/srv/ListStationRoutes" 10 /tmp/robot_station_routes_service.err

mission_response="$(
  ros2 service call /v2/estimate_mission_cost robot_interfaces_mission/srv/EstimateMissionCost \
    "{mission_file: '', nominal_speed_mps: 0.5, battery_voltage: 24.0}"
)"
rg "success=True" <<<"${mission_response}"
rg "mission_id='demo_patrol'" <<<"${mission_response}"
rg "battery_sufficient=True" <<<"${mission_response}"

station_response="$(
  ros2 service call /v2/estimate_station_transport robot_interfaces_mission/srv/EstimateStationTransport \
    "{pickup_station: receiving, dropoff_station: storage_a, nominal_speed_mps: 0.5, battery_voltage: 24.0}"
)"
rg "success=True" <<<"${station_response}"
rg "mission_id='estimate_receiving_to_storage_a'" <<<"${station_response}"
rg "waypoint_count=2" <<<"${station_response}"
rg "distance_m=1.2" <<<"${station_response}"
rg "battery_sufficient=True" <<<"${station_response}"

route_response="$(
  ros2 service call /v2/list_station_routes robot_interfaces_mission/srv/ListStationRoutes "{}"
)"
rg "success=True" <<<"${route_response}"
rg "from_station=\\['receiving'" <<<"${route_response}"
rg "to_station=\\['storage_a'" <<<"${route_response}"

low_battery_response="$(
  ros2 service call /v2/estimate_station_transport robot_interfaces_mission/srv/EstimateStationTransport \
    "{pickup_station: receiving, dropoff_station: storage_a, nominal_speed_mps: 0.5, battery_voltage: 21.0}"
)"
rg "success=True" <<<"${low_battery_response}"
rg "battery_sufficient=False" <<<"${low_battery_response}"

echo "mission and station transport estimates passed"
