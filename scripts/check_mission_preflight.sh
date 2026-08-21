#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

wait_submit_order 10 /tmp/robot_preflight_submit_service.err

keepout_response="$(
  call_submit_order blocked_keepout transport 10 \
    "frame_id=map;pickup_x=-2.0;pickup_y=2.0;pickup_yaw=0.0;dropoff_x=0.2;dropoff_y=0.0;dropoff_yaw=0.0;start_if_idle=false" \
    transport_order
)"
rg "accepted=False" <<<"${keepout_response}"
rg "preflight rejected|keepout" <<<"${keepout_response}"

ros2 topic pub -1 /chassis/state robot_interfaces/msg/ChassisState \
  "{backend: mock, kinematics_model: diff_drive, connected: true, battery_voltage: 21.0, linear_velocity: 0.0, angular_velocity: 0.0, status: low_battery}" \
  >/tmp/robot_preflight_chassis_pub.out

low_battery_response="$(
  call_submit_order low_battery_station station_transport 10 \
    "pickup_station=receiving;dropoff_station=storage_a;start_if_idle=false;preempt_current=false" \
    station_transport_order
)"
rg "accepted=False" <<<"${low_battery_response}"
rg "battery insufficient" <<<"${low_battery_response}"

echo "mission preflight rejection checks passed"
