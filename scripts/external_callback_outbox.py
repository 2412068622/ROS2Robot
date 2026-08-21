#!/usr/bin/env python3
import argparse
import ast
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from urllib import request as urllib_request
from urllib.error import URLError, HTTPError


def run_ros2(args, timeout_sec=30):
    return subprocess.run(
        args, check=False, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        timeout=timeout_sec)


def response_list(text, name):
    values = []
    for match in re.finditer(rf"(?<![A-Za-z_]){name}=\[(.*?)\](?:,|\))", text, re.S):
        try:
            parsed = ast.literal_eval("[" + match.group(1) + "]")
        except (SyntaxError, ValueError):
            parsed = []
        values.extend(parsed)
    return values


def parse_events(stdout):
    stamps = response_list(stdout, "stamp")
    mission_ids = response_list(stdout, "mission_ids")
    states = response_list(stdout, "states")
    previous_states = response_list(stdout, "previous_states")
    messages = response_list(stdout, "messages")
    events = []
    for index, mission_id in enumerate(mission_ids):
        event = {
            "stamp": stamps[index] if index < len(stamps) else "",
            "mission_id": mission_id,
            "state": states[index] if index < len(states) else "",
            "previous_state": previous_states[index] if index < len(previous_states) else "",
            "message": messages[index] if index < len(messages) else "",
        }
        event["event_id"] = event_id(event)
        events.append(event)
    return events


def event_id(event):
    return f"{event.get('stamp', '')}:{event.get('mission_id', '')}:{event.get('state', '')}"


def load_outbox(path):
    file_path = Path(path)
    if not file_path.exists():
        return {}
    return json.loads(file_path.read_text(encoding="utf-8"))


def save_outbox(path, outbox):
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(json.dumps(outbox, indent=2, sort_keys=True), encoding="utf-8")


def list_events(limit, state_filter, mission_id_filter):
    request = (
        "{"
        f"limit: {limit}, "
        f"state_filter: {state_filter}, "
        f"mission_id_filter: {mission_id_filter}"
        "}"
    )
    result = run_ros2([
        "ros2",
        "service",
        "call",
        "/v2/list_mission_events",
        "robot_interfaces_mission/srv/ListMissionEvents",
        request,
    ])
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "failed to list mission events")
    return parse_events(result.stdout)


def post_callback(endpoint, record, timeout_sec):
    payload = json.dumps(record, sort_keys=True).encode("utf-8")
    req = urllib_request.Request(
        endpoint,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib_request.urlopen(req, timeout=timeout_sec) as response:
            body = response.read().decode("utf-8")
            return True, response.status, body
    except HTTPError as exc:
        return False, exc.code, exc.read().decode("utf-8")
    except URLError as exc:
        return False, 0, str(exc)


def enqueue_events(outbox, endpoint, events):
    for event in events:
        key = event["event_id"]
        if key in outbox:
            continue
        outbox[key] = {
            "event_id": key,
            "endpoint": endpoint,
            "event": event,
            "status": "PENDING",
            "attempts": 0,
            "last_status_code": 0,
            "last_error": "",
            "updated_at": datetime.now(timezone.utc).isoformat(),
        }


def send_pending(outbox, endpoint, timeout_sec, replay_failed=False):
    for record in outbox.values():
        if record.get("endpoint") != endpoint:
            continue
        status = record.get("status")
        if status == "SENT":
            continue
        if status == "FAILED" and not replay_failed:
            continue
        record["status"] = "RETRYING" if record.get("attempts", 0) > 0 else "PENDING"
        record["attempts"] = int(record.get("attempts", 0)) + 1
        ok, status_code, body = post_callback(endpoint, record, timeout_sec)
        record["last_status_code"] = status_code
        record["updated_at"] = datetime.now(timezone.utc).isoformat()
        if ok and 200 <= status_code < 300:
            record["status"] = "SENT"
            record["last_error"] = ""
        else:
            record["status"] = "FAILED"
            record["last_error"] = body


def main():
    parser = argparse.ArgumentParser(description="Mission event webhook outbox")
    parser.add_argument("--outbox", required=True)
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--limit", type=int, default=80)
    parser.add_argument("--state-filter", default="")
    parser.add_argument("--mission-id-filter", default="")
    parser.add_argument("--timeout-sec", type=int, default=5)
    parser.add_argument("--replay-failed", action="store_true")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    outbox = load_outbox(args.outbox)
    if not args.list:
        events = list_events(args.limit, args.state_filter, args.mission_id_filter)
        enqueue_events(outbox, args.endpoint, events)
        send_pending(outbox, args.endpoint, args.timeout_sec, args.replay_failed)
        save_outbox(args.outbox, outbox)
    print(json.dumps({"callbacks": list(outbox.values())}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
