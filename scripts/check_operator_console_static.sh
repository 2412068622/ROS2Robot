#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
HTML_FILE="${ROOT_DIR}/tools/operator_console.html"

test -f "${HTML_FILE}"
rg -n "/api/operator/snapshot" "${HTML_FILE}"
rg -n "/api/business_orders" "${HTML_FILE}"
rg -n "/api/workflows/pause" "${HTML_FILE}"
rg -n "/api/workflows/resume" "${HTML_FILE}"
rg -n "/api/workflows/cancel" "${HTML_FILE}"
rg -n "fetch\\(" "${HTML_FILE}"
rg -n "order_id" "${HTML_FILE}"
if rg -n "['\"]workflow_order_id['\"]|workflow_order_id:" "${HTML_FILE}" >/dev/null; then
  echo "operator console must use REST gateway order_id field, not workflow_order_id" >&2
  exit 1
fi
if rg -n "https?://" "${HTML_FILE}" | rg -v "127.0.0.1|localhost" >/dev/null; then
  echo "unexpected external dependency in operator console html" >&2
  exit 1
fi
echo "operator console static check passed"
