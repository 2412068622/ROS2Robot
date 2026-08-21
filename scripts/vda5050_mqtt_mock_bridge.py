#!/usr/bin/env python3
import argparse
import json
import re
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path


def run_ros2(args):
    return subprocess.run(args, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def response_value(text, name, default=""):
    match = re.search(rf"{name}='([^']*)'", text)
    return match.group(1) if match else default


def response_bool(text, name):
    match = re.search(rf"{name}=(True|False)", text)
    return bool(match and match.group(1) == "True")


def submit_order(order):
    payload_json = json.dumps({
        "serial_number": order.get("serial_number", "robot_1"),
        "pickup_node_id": order["pickup_node_id"],
        "dropoff_node_id": order["dropoff_node_id"],
        "start_if_idle": bool(order.get("start_if_idle", True)),
        "preempt_current": bool(order.get("preempt_current", False)),
    }, sort_keys=True)
    request = (
        "{"
        f"order_id: {order['order_id']}, "
        "order_type: vda5050, "
        f"priority: {int(order.get('priority', 50))}, "
        f"payload_json: {json.dumps(payload_json)}, "
        "tags: [vda5050_mqtt_mock_bridge]"
        "}"
    )
    return run_ros2([
        "ros2",
        "service",
        "call",
        "/v2/submit_order",
        "robot_interfaces_business/srv/SubmitOrder",
        request,
    ])


def wait_for_state(mission_id, desired_state, timeout_sec):
    deadline = time.time() + timeout_sec
    last_output = ""
    while time.time() < deadline:
        result = run_ros2([
            "ros2",
            "service",
            "call",
            "/v2/list_mission_events",
            "robot_interfaces_mission/srv/ListMissionEvents",
            f"{{limit: 80, state_filter: {desired_state}, mission_id_filter: {mission_id}}}",
        ])
        last_output = result.stdout
        if "success=True" in last_output and f"states=['{desired_state}']" in last_output:
            return True, last_output
        time.sleep(1.0)
    return False, last_output


def state_payload(order, mission_id, accepted, finished, submit_response, event_response):
    action_state = "FINISHED" if finished else ("ACCEPTED" if accepted else "REJECTED")
    return {
        "headerId": int(time.time()),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "version": "2.0.0-mock",
        "manufacturer": "robot_demo",
        "serialNumber": order.get("serial_number", "robot_1"),
        "orderId": order["order_id"],
        "missionId": mission_id,
        "orderState": action_state,
        "nodeStates": [
            {"nodeId": order.get("pickup_node_id", ""), "released": True},
            {"nodeId": order.get("dropoff_node_id", ""), "released": True},
        ],
        "edgeStates": [
            {
                "edgeId": f"{order.get('pickup_node_id', '')}->{order.get('dropoff_node_id', '')}",
                "released": True,
            }
        ],
        "batteryState": {
            "batteryCharge": 80.0,
            "charging": False,
            "reach": 120.0,
        },
        "errors": [] if accepted else [{"errorType": "ORDER_REJECTED", "errorDescription": submit_response}],
        "information": [
            {
                "infoType": "source_chain",
                "infoDescription": "/v2/submit_order(vda5050) -> station order -> mission queue -> /navigate_sequence -> /v2/list_mission_events",
            }
        ],
        "rawSubmitResponse": submit_response,
        "rawEventResponse": event_response,
    }


def connection_payload(serial_number, state):
    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "version": "2.0.0-mock",
        "manufacturer": "robot_demo",
        "serialNumber": serial_number,
        "connectionState": state,
    }


def process_order_file(order_file, state_dir, connection_dir, timeout_sec):
    order = json.loads(order_file.read_text(encoding="utf-8"))
    submit = submit_order(order)
    accepted = response_bool(submit.stdout, "accepted")
    mission_id = response_value(submit.stdout, "mission_id")
    finished = False
    event_response = ""
    if accepted and mission_id:
        finished, event_response = wait_for_state(mission_id, "FINISHED", timeout_sec)
    state = state_payload(order, mission_id, accepted, finished, submit.stdout.strip(), event_response.strip())
    state_dir.mkdir(parents=True, exist_ok=True)
    connection_dir.mkdir(parents=True, exist_ok=True)
    (state_dir / f"{order['order_id']}.state.json").write_text(
        json.dumps(state, indent=2, sort_keys=True), encoding="utf-8")
    (connection_dir / f"{order.get('serial_number', 'robot_1')}.connection.json").write_text(
        json.dumps(connection_payload(order.get("serial_number", "robot_1"), "ONLINE"), indent=2, sort_keys=True),
        encoding="utf-8")
    return accepted and finished


def main():
    parser = argparse.ArgumentParser(description="Mock VDA 5050 MQTT bridge using files as topics")
    parser.add_argument("--mock-broker-dir", required=True)
    parser.add_argument("--timeout-sec", type=int, default=45)
    args = parser.parse_args()
    broker = Path(args.mock_broker_dir)
    order_dir = broker / "orders"
    state_dir = broker / "states"
    connection_dir = broker / "connections"
    order_files = sorted(order_dir.glob("*.json"))
    if not order_files:
        raise SystemExit("no order files in " + str(order_dir))
    ok = True
    for order_file in order_files:
        ok = process_order_file(order_file, state_dir, connection_dir, args.timeout_sec) and ok
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
