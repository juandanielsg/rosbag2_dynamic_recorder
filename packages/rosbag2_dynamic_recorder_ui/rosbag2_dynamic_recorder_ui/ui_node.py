# Copyright 2026 Juan Daniel Suárez González
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

Standard library plus dynrec. No web framework, no npm, no CDN -- the page must load on a robot
with no internet, and every dependency added here is an install barrier for the people this is
for. Everything that talks to the recorder goes through :class:`dynrec.Recorder`, so a status
field or an event stream added to the library reaches the page without a second copy of the
plumbing here -- which is how the UI once fell behind the CLI on what it showed.
"""

import json
import threading
import time
from dataclasses import asdict
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from dynrec import DynrecError, Recorder, TopicChange
from rosbag2_dynamic_recorder_ui import DEFAULT_PORT, DEFAULT_RECORDER_NODE

#: How many events to keep. The feed shows a handful; the timeline needs the rest, and the
#: recorder retains only ten for late joiners, so anything earlier than this page simply was
#: not observed -- which the timeline says rather than guesses at.
HISTORY_LIMIT = 400

#: Topics that are never useful to record and only clutter the picker.
HIDDEN_TOPICS = {"/parameter_events", "/rosout"}

#: How long without a status publication before the page says contact is lost. The recorder
#: publishes every second by default, so this is several missed ticks, not one late one.
STALE_AFTER_SECONDS = 5.0

#: How often to retry ~/get_profiles while the recorder is not answering yet.
PROFILES_RETRY_SECONDS = 5.0


def _selection(payload):
    """The topics/regex/exclude_regex triple every topic-changing action takes."""
    return dict(
        topics=payload.get("topics", []),
        regex=payload.get("regex", ""),
        exclude_regex=payload.get("exclude_regex", ""),
    )


#: What the page may ask for, as calls on a :class:`dynrec.Recorder`. The page reads `ok` and
#: `error` from every answer, and `subscribed_topics` from a topic change, for its toast.
ACTIONS = {
    "set_topics": lambda rec, p: rec.set_topics(**_selection(p)),
    "subscribe_topics": lambda rec, p: rec.add(**_selection(p)),
    "unsubscribe_topics": lambda rec, p: rec.remove(**_selection(p)),
    "set_profile": lambda rec, p: rec.profile(p.get("name", "")),
    "pause": lambda rec, _p: rec.pause(),
    "resume": lambda rec, _p: rec.resume(),
    "split_bagfile": lambda rec, _p: rec.split(),
    "snapshot": lambda rec, _p: rec.snapshot(),
    "stop": lambda rec, _p: rec.stop(),
    "record": lambda rec, _p: rec.record(),
}


def build_state(status, age, recorder, available_topics, history, profiles):
    """Shape a :class:`dynrec.Status` into what the page renders.

    Kept out of the Node so it can be tested without a ROS graph. The rule most worth protecting
    -- a field the recorder cannot vouch for is reported as unknown, never as a convenient zero --
    lives in :meth:`dynrec.Status.from_msg`, so this only adds what the page needs on top: whether
    a status has arrived at all, whether it is stale, and what else is on the graph.

    `history` is sorted on the recorder's own stamp, oldest first. The streams arrive on separate
    subscriptions, so the order they land in is not reliably the order they happened, and a pause
    shown above the topic change that preceded it is exactly the misreading the events exist to
    prevent. The page takes its feed from the tail of this list.
    """
    state = {
        "connected": status is not None,
        "recorder": recorder,
        "available_topics": available_topics,
        "history": sorted(history, key=lambda event: event["stamp"]),
        "profiles": profiles,
        "stale_after_seconds": STALE_AFTER_SECONDS,
    }
    if status is not None:
        state.update(status.as_dict(), stale=age is not None and age > STALE_AFTER_SECONDS)
    return state


class RecorderUi(Node):
    def __init__(self):
        super().__init__("rosbag2_dynamic_recorder_ui")

        self.recorder = self.declare_parameter(
            "recorder_node", DEFAULT_RECORDER_NODE
        ).value.rstrip("/")
        self.port = self.declare_parameter("port", DEFAULT_PORT).value
        self.bind = self.declare_parameter("bind", "127.0.0.1").value

        self._lock = threading.Lock()
        self._status = None
        self._status_stamp = 0.0
        self._events = []
        self._profiles = []

        # Named, so the client trusts the name without waiting for discovery: the UI is routinely
        # started alongside or before the recorder, and must come up either way. The client owns
        # its own node and thread; this Node exists for the parameters and the graph query.
        self._client = Recorder(self.recorder, discovery_timeout=0.0)
        self._client.on_status(self._on_status)
        self._client.on_event(self._on_event)
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

        A one-shot attempt that gave up left the profile buttons permanently absent with no way to
        recover short of restarting the UI. Retrying costs one service call per
        PROFILES_RETRY_SECONDS until it succeeds.
        """
        announced = False
        while rclpy.ok():
            try:
                profiles = self._client.profiles().profiles
            except DynrecError:
                if not announced:
                    self.get_logger().info(
                        "Waiting for ~/get_profiles; profile buttons will appear when it responds"
                    )
                    announced = True
                time.sleep(PROFILES_RETRY_SECONDS)
                continue
            with self._lock:
                self._profiles = [
                    {"name": name, "topics": list(topics)} for name, topics in profiles.items()
                ]
            if profiles:
                self.get_logger().info(f"Offering {len(profiles)} recording profile(s)")
            else:
                self.get_logger().info("Recorder has no profiles configured")
            return

    def _on_status(self, status):
        with self._lock:
            self._status = status
            self._status_stamp = time.monotonic()

    def _on_event(self, event):
        with self._lock:
            self._events.append(asdict(event))
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
            history = list(self._events)
            profiles = list(self._profiles)
        return build_state(status, age, self.recorder, self.available_topics(), history, profiles)

    def call(self, action, payload):
        """Run one page action against the recorder. Refusals come back as `ok: false`."""
        run = ACTIONS.get(action)
        if run is None:
            return {"ok": False, "error": f"unknown action '{action}'"}
        try:
            result = run(self._client, payload)
        except DynrecError as exc:
            return {"ok": False, "error": str(exc)}
        out = {"ok": True}
        if isinstance(result, TopicChange):
            out["subscribed_topics"] = result.subscribed
        return out

    def destroy_node(self):
        self._client.close()
        super().destroy_node()


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
