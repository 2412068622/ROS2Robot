#!/usr/bin/env bash
set -euo pipefail

timeout 5s ros2 run tf2_ros tf2_echo odom base_footprint >/dev/null
timeout 5s ros2 run tf2_ros tf2_echo base_footprint base_link >/dev/null
