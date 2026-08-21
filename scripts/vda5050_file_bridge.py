#!/usr/bin/env python3
import json
import re
import subprocess
import sys
import time
from pathlib import Path


def run_ros2(args):
    return subprocess.run(args, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def response_value(text, name, default=""):
    match = re.search(rf"{name}='([^']*)'", text)
    return match.group(1) if match else default


def response_bool(text, name):
    match = re.search(rf"{name}=(True|False)", text)
    return bool(match and match.group(1) == "True")


def wait_finished(mission_id, timeout_sec):
    deadline = time.time() + timeout_sec
    last_output = ""
    while time.time() < deadline:
        result = run_ros2([
            "ros2",
            "service",
            "call",
            "/v2/list_mission_events",
            "robot_interfaces_mission/srv/ListMissionEvents",
            f"{{limit: 80, state_filter: FINISHED, mission_id_filter: {mission_id}}}",
        ])
        last_output = result.stdout
        if "success=True" in last_output and "states=['FINISHED']" in last_output:
            return True, last_output
        time.sleep(1.0)
    return False, last_output


def main():
    if len(sys.argv) < 3:
        print("usage: vda5050_file_bridge.py <order_json> <state_json> [timeout_sec]", file=sys.stderr)
        return 2
    order_path = Path(sys.argv[1])
    state_path = Path(sys.argv[2])
    timeout_sec = int(sys.argv[3]) if len(sys.argv) > 3 else 45

    order = json.loads(order_path.read_text(encoding="utf-8"))
    order_id = order["order_id"]
    serial_number = order.get("serial_number", "robot_1")
    pickup_node_id = order["pickup_node_id"]
    dropoff_node_id = order["dropoff_node_id"]
    priority = int(order.get("priority", 50))
    start_if_idle = bool(order.get("start_if_idle", True))
    preempt_current = bool(order.get("preempt_current", False))
    payload_json = json.dumps({
        "serial_number": serial_number,
        "pickup_node_id": pickup_node_id,
        "dropoff_node_id": dropoff_node_id,
        "start_if_idle": start_if_idle,
        "preempt_current": preempt_current,
    }, sort_keys=True)

    request = (
        "{"
        f"order_id: {order_id}, "
        "order_type: vda5050, "
        f"priority: {priority}, "
        f"payload_json: {json.dumps(payload_json)}, "
        "tags: [vda5050_file_bridge]"
        "}"
    )
    submit = run_ros2([
        "ros2",
        "service",
        "call",
        "/v2/submit_order",
        "robot_interfaces_business/srv/SubmitOrder",
        request,
    ])
    success = response_bool(submit.stdout, "accepted")
    mission_id = response_value(submit.stdout, "mission_id")
    finished = False
    event_output = ""
    if success and mission_id:
        finished, event_output = wait_finished(mission_id, timeout_sec)

    state = {
        "order_id": order_id,
        "serial_number": serial_number,
        "mission_id": mission_id,
        "accepted": success,
        "state": "FINISHED" if finished else ("ACCEPTED" if success else "REJECTED"),
        "submit_response": submit.stdout.strip(),
        "event_response": event_output.strip(),
        "source_chain": "/v2/submit_order(vda5050) -> station order -> preflight -> mission queue -> /navigate_sequence -> /v2/list_mission_events",
    }
    state_path.write_text(json.dumps(state, indent=2), encoding="utf-8")
    return 0 if success and finished else 1


if __name__ == "__main__":
    raise SystemExit(main())
