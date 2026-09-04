# Copyright 2026 juandanielsg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Serves a browser UI for rosbag2_dynamic_recorder from the machine running it.

Deliberately a separate node rather than an HTTP server embedded in the recorder: the recorder's
write path should not share a process with a web server, and this way the UI can be run, restarted
or omitted without touching recording.

Standard library only. No web framework, no npm, no CDN -- the page must load on a robot with no
internet, and every dependency added here is an install barrier for the people this is for.
"""

import json
import threading
import time
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rosbag2_dynamic_recorder_interfaces.msg import (
    PauseEvent,
    RecorderStatus,
    SubscriptionChangeEvent,
)
from rosbag2_dynamic_recorder_interfaces.srv import (
    GetProfiles,
    SetProfile,
    SetTopics,
    SubscribeTopics,
    UnsubscribeTopics,
)
from rosbag2_interfaces.srv import Pause, Record, Resume, Snapshot, SplitBagfile, Stop

#: How many events to keep. The feed shows a handful; the timeline needs the rest, and the
#: recorder retains only ten for late joiners, so anything earlier than this page simply was
#: not observed -- which the timeline says rather than guesses at.
HISTORY_LIMIT = 400

#: Topics that are never useful to record and only clutter the picker.
HIDDEN_TOPICS = {"/parameter_events", "/rosout"}

ACTION_SERVICES = {
    "set_topics": (SetTopics, "set_topics"),
    "subscribe_topics": (SubscribeTopics, "subscribe_topics"),
    "unsubscribe_topics": (UnsubscribeTopics, "unsubscribe_topics"),
    "pause": (Pause, "pause"),
    "resume": (Resume, "resume"),
    "split_bagfile": (SplitBagfile, "split_bagfile"),
    "snapshot": (Snapshot, "snapshot"),
    "stop": (Stop, "stop"),
    "record": (Record, "record"),
    "set_profile": (SetProfile, "set_profile"),
}


def recent_events(events, limit=15):
    """The newest events first, across both event streams.

    Subscription changes and pauses arrive on separate subscriptions, so the order they land in is
    not reliably the order they happened. Both carry the recorder's own stamp, so that is what to
    sort on -- otherwise a pause could be shown above a topic change that actually preceded it,
    which is exactly the kind of misreading these events exist to prevent.
    """
    return sorted(events, key=lambda event: event["stamp"], reverse=True)[:limit]


def build_state(status, age, recorder, available_topics, events):
    """Shape the recorder status into what the page renders.

    Kept out of the Node so it can be tested without a ROS graph, because the rule it encodes is
    the one most worth protecting: a field the recorder cannot vouch for is reported as unknown,
    never as a convenient zero. `messages_missed` is only meaningful when the middleware supplies
    publication sequence numbers, and `messages_lost` counts only what the transport chose to
    report -- which has been observed reading 0 while messages were genuinely absent from a bag.
    """
    if status is None:
        return {
            "connected": False,
            "recorder": recorder,
            "available_topics": available_topics,
            "events": events,
        }

    sequence_ok = bool(status.sequence_numbers_available)
    return {
        "connected": True,
        "stale": age is not None and age > 5.0,
        "recorder": recorder,
        "uri": status.uri,
        "storage_id": status.storage_id,
        "recording": bool(status.recording),
        "paused": bool(status.paused),
        "snapshot_mode": bool(status.snapshot_mode),
        "elapsed_seconds": status.elapsed_seconds,
        # Epoch seconds on the recorder's clock, which is also what event stamps use. The
        # timeline is drawn entirely in those terms so the browser's clock never enters into
        # it.
        "recording_started": (
            status.recording_started.sec + status.recording_started.nanosec / 1e9),
        "subscribed_topics": list(status.subscribed_topics),
        "active_profile": status.active_profile,
        "available_topics": available_topics,
        "messages_written": status.messages_written,
        # None means "cannot tell". The page renders that as unknown rather than as zero.
        "messages_missed": status.messages_missed if sequence_ok else None,
        "messages_lost_reported": status.messages_lost,
        "write_errors": status.write_errors,
        "bag_splits": status.bag_splits,
        "bag_size_bytes": status.bag_size_bytes,
        "events": events,
    }


class RecorderUi(Node):
    def __init__(self):
        super().__init__("rosbag2_dynamic_recorder_ui")

        self.recorder = self.declare_parameter(
            "recorder_node", "/rosbag2_dynamic_recorder"
        ).value.rstrip("/")
        # 8088 rather than 8080: 8080 is the most commonly occupied port on a development
        # machine, and a port clash is a miserable first-run experience.
        self.port = self.declare_parameter("port", 8088).value
        # Loopback by default. The UI has no authentication of any kind, so binding to all
        # interfaces would let anyone who can reach the robot stop a recording or change what is
        # being captured. Opt in explicitly with bind:=0.0.0.0, and read the warning below.
        self.bind = self.declare_parameter("bind", "127.0.0.1").value

        self._lock = threading.Lock()
        self._status = None
        self._status_stamp = 0.0
        self._events = []

        # The recorder latches ~/status, so a UI started mid-recording gets current state at once
        # instead of waiting for the next tick. Match that or we would receive nothing until then.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            RecorderStatus, f"{self.recorder}/status", self._on_status, latched
        )
        events_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            SubscriptionChangeEvent,
            f"{self.recorder}/events/subscription_change",
            self._on_event,
            events_qos,
        )
        # A pause stops every topic at once, so a feed that only shows topic changes leaves the
        # biggest gaps in the recording unexplained -- the same reason the recorder writes these
        # into the bag.
        self.create_subscription(
            PauseEvent,
            f"{self.recorder}/events/pause",
            self._on_pause_event,
            events_qos,
        )

        # Not `self._clients`: rclpy.node.Node already uses that name for its own list, and
        # shadowing it makes create_client() fail with 'dict' object has no attribute 'append'.
        self._service_clients = {
            name: self.create_client(srv, f"{self.recorder}/{path}")
            for name, (srv, path) in ACTION_SERVICES.items()
        }

        self._profiles = []
        self._profiles_client = self.create_client(
            GetProfiles, f"{self.recorder}/get_profiles"
        )
        # Profiles are fixed at recorder startup, so fetch once in the background rather than on
        # every page poll.
        threading.Thread(target=self._fetch_profiles, daemon=True).start()

        web_root = Path(get_package_share_directory("rosbag2_dynamic_recorder_ui")) / "web"
        server = ThreadingHTTPServer(
            (self.bind, self.port), partial(_Handler, self, web_root)
        )
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()

        self.get_logger().info(
            f"UI on http://localhost:{self.port}  (controlling {self.recorder})"
        )
        if self.bind not in ("127.0.0.1", "localhost", "::1"):
            self.get_logger().warning(
                f"UI is bound to {self.bind}, so anyone who can reach this machine on port "
                f"{self.port} can stop the recording or change what is recorded. There is no "
                "authentication. Use bind:=127.0.0.1 unless the network is trusted."
            )

    def _fetch_profiles(self):
        """Keep trying until profiles are known.

        The UI is routinely started alongside or before the recorder, and a one-shot attempt that
        gave up left the profile buttons permanently absent with no way to recover short of
        restarting the UI. Retrying costs one service call every 15s until it succeeds.
        """
        announced = False
        while rclpy.ok():
            if self._profiles_client.wait_for_service(timeout_sec=15.0):
                future = self._profiles_client.call_async(GetProfiles.Request())
                deadline = time.monotonic() + 15.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if future.done() and future.result() is not None:
                    profiles = [
                        {"name": p.name, "topics": list(p.topics)}
                        for p in future.result().profiles
                    ]
                    with self._lock:
                        self._profiles = profiles
                    if profiles:
                        self.get_logger().info(f"Offering {len(profiles)} recording profile(s)")
                        return
                    # An empty list is a real answer -- the recorder has no profiles configured --
                    # so say so once and stop asking.
                    if not announced:
                        self.get_logger().info("Recorder has no profiles configured")
                    return
            if not announced:
                self.get_logger().info(
                    "Waiting for ~/get_profiles; profile buttons will appear when it responds"
                )
                announced = True
            time.sleep(5.0)

    # --- ROS side ---------------------------------------------------------
    def _on_status(self, msg):
        with self._lock:
            self._status = msg
            self._status_stamp = time.monotonic()

    def _on_event(self, msg):
        with self._lock:
            self._events.append(
                {
                    "kind": "topic",
                    "topic": msg.topic_name,
                    "action": "subscribed" if msg.action == 0 else "unsubscribed",
                    "reason": msg.reason,
                    "stamp": msg.stamp.sec + msg.stamp.nanosec / 1e9,
                }
            )
            del self._events[:-HISTORY_LIMIT]

    def _on_pause_event(self, msg):
        with self._lock:
            self._events.append(
                {
                    "kind": "pause",
                    # No topic: that is the point. A pause affects all of them at once, and the
                    # page says so rather than leaving the row looking like it lost its name.
                    "topic": "",
                    "action": "paused" if msg.action == PauseEvent.PAUSED else "resumed",
                    "reason": msg.reason,
                    "stamp": msg.stamp.sec + msg.stamp.nanosec / 1e9,
                }
            )
            del self._events[:-HISTORY_LIMIT]

    def available_topics(self):
        return sorted(
            name
            for name, _types in self.get_topic_names_and_types()
            if name not in HIDDEN_TOPICS and not name.startswith(f"{self.recorder}/")
        )

    def snapshot_state(self):
        """Everything the page needs, in one response."""
        with self._lock:
            status = self._status
            age = time.monotonic() - self._status_stamp if status else None
            events = recent_events(self._events)
            history = list(self._events)
        with self._lock:
            profiles = list(self._profiles)
        state = build_state(status, age, self.recorder, self.available_topics(), events)
        state["profiles"] = profiles
        # Oldest first: the timeline walks it forward, opening and closing spans.
        state["history"] = sorted(history, key=lambda event: event["stamp"])
        return state

    def call(self, action, payload):
        client = self._service_clients.get(action)
        if client is None:
            return {"ok": False, "error": f"unknown action '{action}'"}
        if not client.wait_for_service(timeout_sec=2.0):
            return {"ok": False, "error": f"service '{action}' unavailable"}

        request = ACTION_SERVICES[action][0].Request()
        if hasattr(request, "topics"):
            request.topics = list(payload.get("topics", []))
        if hasattr(request, "name"):
            request.name = str(payload.get("name", ""))
        # Pattern selection, for the services that take it. Sending a regex instead of an
        # enumerated list is the difference between one call and ticking sixty boxes.
        if hasattr(request, "regex"):
            request.regex = str(payload.get("regex", ""))
            request.exclude_regex = str(payload.get("exclude_regex", ""))

        future = client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {"ok": False, "error": f"'{action}' timed out"}

        response = future.result()
        result = {"ok": True}
        for field in ("return_code", "error_string", "subscribed_topics",
                      "unsubscribed_topics", "unavailable_topics", "topics", "success"):
            if hasattr(response, field):
                value = getattr(response, field)
                result[field] = list(value) if isinstance(value, (list, tuple)) else value
        if result.get("return_code", 0) != 0:
            result["ok"] = False
            result.setdefault("error", result.get("error_string") or "request failed")
        return result


class _Handler(BaseHTTPRequestHandler):
    def __init__(self, node, web_root, *args, **kwargs):
        self.node = node
        self.web_root = web_root
        super().__init__(*args, **kwargs)

    def log_message(self, *_args):
        pass  # One line per poll would drown the ROS log.

    def _send(self, code, body, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, payload, code=200):
        self._send(code, json.dumps(payload).encode(), "application/json")

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            page = self.web_root / "index.html"
            try:
                self._send(200, page.read_bytes(), "text/html; charset=utf-8")
            except OSError as exc:
                self._send(500, f"cannot read {page}: {exc}".encode(), "text/plain")
        elif self.path == "/api/state":
            self._send_json(self.node.snapshot_state())
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path != "/api/action":
            self._send(404, b"not found", "text/plain")
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(length) or b"{}")
            action = payload.get("action", "")
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json({"ok": False, "error": f"bad request: {exc}"}, 400)
            return
        self._send_json(self.node.call(action, payload))


def main():
    rclpy.init()
    node = RecorderUi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
