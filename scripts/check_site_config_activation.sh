#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_site_config_activation.XXXXXX)"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "${PIDS[@]:-}" >/dev/null 2>&1 || true
  rm -rf "${LOG_DIR}"
}
trap cleanup EXIT

start_bg() {
  local name="$1"
  shift
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

make_bundle() {
  local target="$1"
  mkdir -p "${target}"
  cp "${ROOT_DIR}/src/robot_tasks/config/stations/warehouse_stations.yaml" "${target}/stations.yaml"
  cp "${ROOT_DIR}/src/robot_tasks/config/facilities/warehouse_facilities.yaml" "${target}/facilities.yaml"
  cp "${ROOT_DIR}/src/robot_tasks/config/fleet/warehouse_fleet.yaml" "${target}/fleet.yaml"
  cp "${ROOT_DIR}/src/robot_tasks/config/scenarios/industry_scenarios.yaml" "${target}/scenarios.yaml"
  cp "${ROOT_DIR}/src/robot_tasks/config/business_orders/business_orders.yaml" "${target}/business_orders.yaml"
  cp "${ROOT_DIR}/src/robot_tasks/config/docks/docks.yaml" "${target}/docks.yaml"
}

BUNDLE_V1="${LOG_DIR}/bundle_v1"
BUNDLE_V2="${LOG_DIR}/bundle_v2"
make_bundle "${BUNDLE_V1}"
make_bundle "${BUNDLE_V2}"

start_bg site_config ros2 run robot_tasks site_config_manager_node

wait_service_type /v2/activate_site_config "robot_interfaces_site/srv/ActivateSiteConfig" 20 \
  "${LOG_DIR}/wait_activate.err"
wait_service_type /v2/rollback_site_config "robot_interfaces_site/srv/RollbackSiteConfig" 20 \
  "${LOG_DIR}/wait_rollback.err"

ros2 service call /v2/activate_site_config robot_interfaces_site/srv/ActivateSiteConfig \
  "{bundle_dir: ${BUNDLE_V1}, version_id: site_v1, require_valid: true}" \
  >"${LOG_DIR}/activate_v1.out"
rg "success=True" "${LOG_DIR}/activate_v1.out"
rg "active_version_id='site_v1'" "${LOG_DIR}/activate_v1.out"
rg "previous_version_id=''" "${LOG_DIR}/activate_v1.out"

ros2 service call /v2/activate_site_config robot_interfaces_site/srv/ActivateSiteConfig \
  "{bundle_dir: ${BUNDLE_V2}, version_id: site_v2, require_valid: true}" \
  >"${LOG_DIR}/activate_v2.out"
rg "success=True" "${LOG_DIR}/activate_v2.out"
rg "active_version_id='site_v2'" "${LOG_DIR}/activate_v2.out"
rg "previous_version_id='site_v1'" "${LOG_DIR}/activate_v2.out"
rg "active_version=site_v1; next_version=site_v2" "${LOG_DIR}/activate_v2.out"

ros2 service call /v2/rollback_site_config robot_interfaces_site/srv/RollbackSiteConfig \
  "{}" >"${LOG_DIR}/rollback.out"
rg "success=True" "${LOG_DIR}/rollback.out"
rg "active_version_id='site_v1'" "${LOG_DIR}/rollback.out"
rg "rolled_back_from_version_id='site_v2'" "${LOG_DIR}/rollback.out"

echo "site config activation passed"
