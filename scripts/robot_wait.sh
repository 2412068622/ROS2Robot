#!/usr/bin/env bash

wait_service_type() {
  local service_name="$1"
  local expected_type="$2"
  local timeout_sec="${3:-10}"
  local err_file="${4:-/tmp/robot_wait_service.err}"
  local deadline=$((SECONDS + timeout_sec))
  while ! ros2 service type "${service_name}" 2>"${err_file}" | rg "${expected_type}"; do
    if (( SECONDS >= deadline )); then
      echo "${service_name} service did not become available as ${expected_type}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_submit_order() {
  local timeout_sec="${1:-10}"
  local err_file="${2:-/tmp/robot_submit_order.err}"
  wait_service_type /v2/submit_order "robot_interfaces_business/srv/SubmitOrder" "${timeout_sec}" "${err_file}"
}

yaml_single_quote() {
  local value="$1"
  printf "'%s'" "${value//\'/\'\'}"
}

yaml_flow_list() {
  if (($# == 0)); then
    printf "[]"
    return
  fi
  local first=true
  printf "["
  local item
  for item in "$@"; do
    if ${first}; then
      first=false
    else
      printf ", "
    fi
    yaml_single_quote "${item}"
  done
  printf "]"
}

call_submit_order() {
  local order_id="$1"
  local order_type="$2"
  local priority="$3"
  local payload_json="$4"
  shift 4
  local tags
  tags="$(yaml_flow_list "$@")"
  ros2 service call /v2/submit_order robot_interfaces_business/srv/SubmitOrder \
    "{order_id: $(yaml_single_quote "${order_id}"), order_type: $(yaml_single_quote "${order_type}"), priority: ${priority}, payload_json: $(yaml_single_quote "${payload_json}"), tags: ${tags}}"
}

wait_topic_type() {
  local topic_name="$1"
  local expected_type="$2"
  local timeout_sec="${3:-10}"
  local err_file="${4:-/tmp/robot_wait_topic.err}"
  local deadline=$((SECONDS + timeout_sec))
  while ! ros2 topic type "${topic_name}" 2>"${err_file}" | rg "${expected_type}"; do
    if (( SECONDS >= deadline )); then
      echo "${topic_name} topic did not become available as ${expected_type}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_topic_sample() {
  local topic_name="$1"
  local expected_type="$2"
  local timeout_sec="${3:-10}"
  local err_file="${4:-/tmp/robot_wait_topic_sample.err}"
  if ! timeout "${timeout_sec}s" ros2 topic echo --once "${topic_name}" "${expected_type}" >"${err_file}" 2>&1; then
    echo "${topic_name} topic did not publish a ${expected_type} sample" >&2
    cat "${err_file}" >&2 || true
    return 1
  fi
}

wait_action_type() {
  local action_name="$1"
  local expected_type="$2"
  local timeout_sec="${3:-10}"
  local err_file="${4:-/tmp/robot_wait_action.err}"
  local deadline=$((SECONDS + timeout_sec))
  while ! ros2 action list -t 2>"${err_file}" | rg "${action_name}.*${expected_type}"; do
    if (( SECONDS >= deadline )); then
      echo "${action_name} action did not become available as ${expected_type}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_lifecycle_active() {
  local service_name="$1"
  local timeout_sec="${2:-30}"
  local err_file="${3:-/tmp/robot_wait_lifecycle.err}"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    if ros2 service call "${service_name}" lifecycle_msgs/srv/GetState "{}" >"${err_file}" 2>&1 &&
      rg "label(: |=)'?active'?" "${err_file}" >/dev/null; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "${service_name} lifecycle state did not reach active" >&2
      cat "${err_file}" >&2 || true
      return 1
    fi
    sleep 1
  done
}
