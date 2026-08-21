#!/usr/bin/env python3
import json
import re
import subprocess
import sys
import time
from pathlib import Path


def run_ros2(args):
    return subprocess.run(args, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def response_bool(text, name):
    match = re.search(rf"{name}=(True|False)", text)
    return bool(match and match.group(1) == "True")


def response_value(text, name, default=""):
    match = re.search(rf"{name}='([^']*)'", text)
    return match.group(1) if match else default


def wait_finished(mission_id, timeout_sec):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        result = run_ros2([
            "ros2",
            "service",
            "call",
            "/v2/list_mission_events",
            "robot_interfaces_mission/srv/ListMissionEvents",
            f"{{limit: 80, state_filter: FINISHED, mission_id_filter: {mission_id}}}",
        ])
        if "success=True" in result.stdout and "states=['FINISHED']" in result.stdout:
            return True
        time.sleep(1.0)
    return False


def main():
    if len(sys.argv) < 4:
        print(
            "usage: facility_plc_file_adapter.py <plc_state_json> <request_json> <result_json> [timeout_sec]",
            file=sys.stderr,
        )
        return 2
    plc_path = Path(sys.argv[1])
    request_path = Path(sys.argv[2])
    result_path = Path(sys.argv[3])
    timeout_sec = int(sys.argv[4]) if len(sys.argv) > 4 else 45

    plc_state = json.loads(plc_path.read_text(encoding="utf-8")) if plc_path.exists() else {}
    request = json.loads(request_path.read_text(encoding="utf-8"))
    resource_id = request["resource_id"]
    action = request["action"]
    request_id = request.get("request_id", f"plc_{resource_id}_{action}")

    before = plc_state.get(resource_id, {}).get("state", "unknown")
    service_request = (
        "{"
        f"request_id: {request_id}, "
        f"resource_id: {resource_id}, "
        f"resource_type: {request.get('resource_type', '')}, "
        f"action: {action}, "
        f"priority: {int(request.get('priority', 60))}, "
        f"start_if_idle: {'true' if request.get('start_if_idle', True) else 'false'}, "
        f"preempt_current: {'true' if request.get('preempt_current', False) else 'false'}, "
        f"hold_after_action: {'true' if request.get('hold_after_action', True) else 'false'}"
        "}"
    )
    submit = run_ros2([
        "ros2",
        "service",
        "call",
        "/v2/execute_facility_action",
        "robot_interfaces_facility/srv/ExecuteFacilityAction",
        service_request,
    ])
    accepted = response_bool(submit.stdout, "success")
    mission_id = response_value(submit.stdout, "mission_id")
    finished = wait_finished(mission_id, timeout_sec) if accepted and mission_id else False

    plc_state.setdefault(resource_id, {})
    plc_state[resource_id]["state"] = action if finished else before
    plc_state[resource_id]["last_mission_id"] = mission_id
    plc_state[resource_id]["last_action"] = action
    plc_path.write_text(json.dumps(plc_state, indent=2), encoding="utf-8")

    result = {
        "resource_id": resource_id,
        "action": action,
        "state_before": before,
        "state_after": plc_state[resource_id]["state"],
        "mission_id": mission_id,
        "accepted": accepted,
        "finished": finished,
        "submit_response": submit.stdout.strip(),
        "source_chain": "mock PLC JSON -> /v2/execute_facility_action -> facility resource lock -> station mission -> /navigate_sequence -> mission event audit",
    }
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return 0 if accepted and finished else 1


if __name__ == "__main__":
    raise SystemExit(main())
