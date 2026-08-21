#!/usr/bin/env bash
set -euo pipefail

map_yaml="${1:?usage: scripts/validate_map.sh <map.yaml>}"
map_dir="$(dirname "${map_yaml}")"

if [[ ! -f "${map_yaml}" ]]; then
  echo "map yaml not found: ${map_yaml}" >&2
  exit 1
fi

image="$(awk -F: '/^[[:space:]]*image:/ {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}' "${map_yaml}")"
resolution="$(awk -F: '/^[[:space:]]*resolution:/ {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}' "${map_yaml}")"
origin="$(awk -F: '/^[[:space:]]*origin:/ {sub(/^[^:]*:[[:space:]]*/, ""); print; exit}' "${map_yaml}")"

if [[ -z "${image}" || -z "${resolution}" || -z "${origin}" ]]; then
  echo "map yaml must include image, resolution, and origin fields" >&2
  exit 1
fi

image_path="${image}"
if [[ "${image_path}" != /* ]]; then
  image_path="${map_dir}/${image_path}"
fi

if [[ ! -f "${image_path}" ]]; then
  echo "map image not found: ${image_path}" >&2
  exit 1
fi

if ! awk "BEGIN {exit !(${resolution} > 0.0)}"; then
  echo "map resolution must be positive: ${resolution}" >&2
  exit 1
fi

magic="$(head -c 2 "${image_path}")"
if [[ "${magic}" != "P2" && "${magic}" != "P5" ]]; then
  echo "map image must be PGM P2/P5, got '${magic}'" >&2
  exit 1
fi

echo "Map OK: yaml=${map_yaml} image=${image_path} resolution=${resolution} origin=${origin}"
