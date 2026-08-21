#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/robot_wait.sh"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

LOG_DIR="$(mktemp -d /tmp/robot_site_config_validation.XXXXXX)"
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

VALID_BUNDLE="${LOG_DIR}/valid_bundle"
INVALID_BUNDLE="${LOG_DIR}/invalid_bundle"
make_bundle "${VALID_BUNDLE}"
make_bundle "${INVALID_BUNDLE}"
python3 - "${INVALID_BUNDLE}/scenarios.yaml" <<'PY'
import sys
path = sys.argv[1]
text = open(path, encoding="utf-8").read()
text = text.replace("pickup_station_id: receiving", "pickup_station_id: ghost_station", 1)
open(path, "w", encoding="utf-8").write(text)
PY

start_bg site_config ros2 run robot_tasks site_config_manager_node

wait_service_type /v2/validate_site_config "robot_interfaces_site/srv/ValidateSiteConfig" 20 \
  "${LOG_DIR}/wait_validate.err"

ros2 service call /v2/validate_site_config robot_interfaces_site/srv/ValidateSiteConfig \
  "{bundle_dir: ${VALID_BUNDLE}, version_id: valid_v1}" >"${LOG_DIR}/valid.out"
rg "success=True" "${LOG_DIR}/valid.out"
rg "version_id='valid_v1'" "${LOG_DIR}/valid.out"
rg "site config validation passed" "${LOG_DIR}/valid.out"

ros2 service call /v2/validate_site_config robot_interfaces_site/srv/ValidateSiteConfig \
  "{bundle_dir: ${INVALID_BUNDLE}, version_id: invalid_v1}" >"${LOG_DIR}/invalid.out"
rg "success=False" "${LOG_DIR}/invalid.out"
rg "ghost_station" "${LOG_DIR}/invalid.out"
rg "site config validation failed" "${LOG_DIR}/invalid.out"

echo "site config validation passed"
