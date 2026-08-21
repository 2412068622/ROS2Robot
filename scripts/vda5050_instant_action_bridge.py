#!/usr/bin/env python3
import json
import re
import subprocess
import sys
from pathlib import Path


def run_ros2(args):
    return subprocess.run(args, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def response_bool(text, name):
    match = re.search(rf"{name}=(True|False)", text)
    return bool(match and match.group(1) == "True")


def call_service(service_name, service_type, request):
    return run_ros2(["ros2", "service", "call", service_name, service_type, request]).stdout


def factsheet(action):
    return {
        "serial_number": action.get("serial_number", "robot_1"),
        "manufacturer": "robot_demo",
        "protocol": "mock_vda5050_file_bridge",
        "supported_instant_actions": ["pause", "resume", "cancelOrder", "factsheet"],
        "source_chain": "VDA instant action JSON -> existing ROS workflow services",
    }


def workflow_request(order_id, flag_name):
    flag_value = "true"
    return f"{{order_id: {order_id}, {flag_name}: {flag_value}}}"


def handle_action(action):
    action_type = action.get("action_type", "")
    normalized = action_type.replace("_", "").replace("-", "").lower()
    order_id = action.get("order_id", "")
    if normalized == "factsheet":
        return True, "FINISHED", "factsheet generated", "", factsheet(action)
    if not order_id:
        return False, "FAILED", "order_id is required for workflow instant action", "", {}
    if normalized in ("pause", "pauseorder"):
        output = call_service(
            "/v2/pause_workflow",
            "robot_interfaces_business/srv/PauseWorkflow",
            workflow_request(order_id, "pause_active"),
        )
        return response_bool(output, "success"), "FINISHED", "pause requested", output, {}
    if normalized in ("resume", "resumeorder"):
        output = call_service(
            "/v2/resume_workflow",
            "robot_interfaces_business/srv/ResumeWorkflow",
            workflow_request(order_id, "resume_active"),
        )
        return response_bool(output, "success"), "FINISHED", "resume requested", output, {}
    if normalized in ("cancel", "cancelorder"):
        output = call_service(
            "/v2/cancel_workflow",
            "robot_interfaces_business/srv/CancelWorkflow",
            workflow_request(order_id, "cancel_active"),
        )
        return response_bool(output, "success"), "FINISHED", "cancel requested", output, {}
    return False, "FAILED", f"unsupported instant action: {action_type}", "", {}


def main():
    if len(sys.argv) != 3:
        print(
            "usage: vda5050_instant_action_bridge.py <instant_action_json> <state_json>",
            file=sys.stderr,
        )
        return 2
    action_path = Path(sys.argv[1])
    state_path = Path(sys.argv[2])
    action = json.loads(action_path.read_text(encoding="utf-8"))
    success, action_state, message, service_response, payload = handle_action(action)
    state = {
        "action_id": action.get("action_id", ""),
        "action_type": action.get("action_type", ""),
        "order_id": action.get("order_id", ""),
        "accepted": success,
        "action_state": action_state if success else "FAILED",
        "message": message,
        "service_response": service_response.strip(),
        "payload": payload,
    }
    state_path.write_text(json.dumps(state, indent=2), encoding="utf-8")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
