#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT

fail=0
live_audit_status="skipped because ros2 is unavailable"
live_node_count=0

report_match() {
  local title="$1"
  local file="$2"
  if [[ -s "${file}" ]]; then
    echo "${title}" >&2
    cat "${file}" >&2
    fail=1
  fi
}

rg -n "robot_interfaces/srv/" scripts src/robot_simulation tools \
  --glob '!scripts/check_robot_interfaces_legacy_usage.sh' \
  >"${tmp_dir}/static_service_type.txt" || true
report_match "legacy robot_interfaces/srv service type found in scripts/src/robot_simulation/tools:" \
  "${tmp_dir}/static_service_type.txt"

rg -n \
  "create_client<robot_interfaces::srv|rclcpp::Client<robot_interfaces::srv|service_call\\([^\\n]*robot_interfaces/srv" \
  src scripts tools \
  --glob '!scripts/check_robot_interfaces_legacy_usage.sh' \
  >"${tmp_dir}/static_clients.txt" || true
report_match "legacy robot_interfaces service client found in repository code:" \
  "${tmp_dir}/static_clients.txt"

rg -n \
  "robot_interfaces/srv/Submit|/submit_transport_order|/submit_station_transport_order|/submit_fleet_station_task|/submit_business_order|/submit_scenario_task|/submit_scenario_workflow|/submit_vda5050_order|/submit_picking_wave|/submit_replenishment_order|/submit_milk_run" \
  scripts src/robot_simulation tools \
  --glob '!scripts/check_robot_interfaces_legacy_usage.sh' \
  | grep -v "兼容入口" >"${tmp_dir}/legacy_submit_examples.txt" || true
report_match "legacy Submit* usage found outside compatibility notes:" \
  "${tmp_dir}/legacy_submit_examples.txt"

rg -n \
  '"/v2/submit_(transport_order|vda5050_order|business_order|scenario_task|scenario_workflow|picking_wave|replenishment_order|milk_run)"' \
  src scripts tools \
  --glob '!scripts/check_robot_interfaces_legacy_usage.sh' \
  >"${tmp_dir}/legacy_submit_v2_servers.txt" || true
report_match "legacy Submit* v2 service registration found in repository code:" \
  "${tmp_dir}/legacy_submit_v2_servers.txt"

if command -v ros2 >/dev/null 2>&1; then
  if ros2 node list >"${tmp_dir}/ros_nodes.txt" 2>"${tmp_dir}/ros_nodes.err"; then
    live_audit_status="scanned live ROS graph"
    live_node_count="$(grep -cve '^[[:space:]]*$' "${tmp_dir}/ros_nodes.txt" || true)"
    while IFS= read -r node_name; do
      [[ -n "${node_name}" ]] || continue
      safe_name="$(printf '%s' "${node_name}" | tr '/:' '__')"
      if ros2 node info "${node_name}" >"${tmp_dir}/node_${safe_name}.txt" 2>/dev/null; then
        awk -v node="${node_name}" '
          /^[[:space:]]*Service Clients:/ { in_clients = 1; next }
          /^[[:space:]]*[A-Za-z][A-Za-z ]*:/ && $0 !~ /^[[:space:]]*Service Clients:/ {
            in_clients = 0
          }
          in_clients && /robot_interfaces\/srv\// {
            print node ": " $0
          }
        ' "${tmp_dir}/node_${safe_name}.txt" >>"${tmp_dir}/live_clients.txt"
      fi
    done <"${tmp_dir}/ros_nodes.txt"
    report_match "live ROS graph has legacy robot_interfaces service clients:" \
      "${tmp_dir}/live_clients.txt"
  else
    live_audit_status="skipped because no ROS graph could be queried"
    echo "ros2 is available, but no ROS graph could be queried; skipped live client audit." >&2
  fi
else
  echo "ros2 command not found; skipped live client audit." >&2
fi

if (( fail != 0 )); then
  exit 1
fi

echo "robot_interfaces legacy usage audit passed"
echo "static repository clients: clean"
echo "legacy Submit* examples: compatibility notes only"
if command -v ros2 >/dev/null 2>&1; then
  echo "live ROS client audit: ${live_audit_status}; nodes=${live_node_count}; no legacy clients found"
else
  echo "live ROS client audit: skipped because ros2 is unavailable"
fi
