#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${1:-/tmp/robot_fulfillment_results}"
RUN_ID="${2:-fulfillment_alpha}"
mkdir -p "${OUT_DIR}"

started_at="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
"${SCRIPT_DIR}/check_warehouse_fulfillment.sh"
finished_at="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"

csv_file="${OUT_DIR}/fulfillment_${RUN_ID}.csv"
json_file="${OUT_DIR}/fulfillment_${RUN_ID}.json"
cat >"${csv_file}" <<CSV
run_id,started_at,finished_at,status,scenario
${RUN_ID},${started_at},${finished_at},PASS,warehouse_fulfillment
CSV
cat >"${json_file}" <<JSON
{
  "run_id": "${RUN_ID}",
  "started_at": "${started_at}",
  "finished_at": "${finished_at}",
  "status": "PASS",
  "scenario": "warehouse_fulfillment",
  "script": "check_warehouse_fulfillment.sh"
}
JSON
python3 -m json.tool "${json_file}" >/dev/null
echo "fulfillment benchmark passed: ${csv_file} ${json_file}"
