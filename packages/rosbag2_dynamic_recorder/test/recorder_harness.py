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

"""The shared harness of the integration tests, which drive a real recorder process.

They lock down the claims that would be expensive to rediscover: changing the topic set does not
interrupt the topics you did not touch; the bag explains its own sparse channels; services refuse
for accurate reasons; stopping is recoverable.

Runs its own publishers on a private ROS_DOMAIN_ID, so it needs no simulator and cannot collide
with anything else on the machine. conftest.py imports this before any test runs, which is what
sets that domain.
"""

import os
import shutil
import subprocess
import tempfile
import time
from collections import defaultdict

import rclpy
import rosbag2_dynamic_recorder_interfaces.srv as _own
import rosbag2_interfaces.srv as _stock
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosbag2_dynamic_recorder_interfaces.msg import SubscriptionChangeEvent
from rosbag2_dynamic_recorder_interfaces.srv import GetStatus, SetTopics
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from std_msgs.msg import String


def _service_type(name, part, field):
    stock = getattr(_stock, name, None)
    if stock is not None and hasattr(getattr(stock, part), field):
        return stock
    return getattr(_own, name)


Record = _service_type("Record", "Request", "start_time")
Resume = _service_type("Resume", "Request", "resume_mode")
SplitBagfile = _service_type("SplitBagfile", "Request", "split_mode")
Stop = _service_type("Stop", "Response", "return_code")

EVENT_TYPE = "rosbag2_dynamic_recorder_interfaces/msg/SubscriptionChangeEvent"
SUBSCRIBED, UNSUBSCRIBED = 0, 1
PAUSE_EVENT_TYPE = "rosbag2_dynamic_recorder_interfaces/msg/PauseEvent"
PAUSED, RESUMED = 0, 1
LOW_DISK_EVENT_TYPE = "rosbag2_dynamic_recorder_interfaces/msg/LowDiskEvent"
BAG_SIZE_LIMIT_EVENT_TYPE = "rosbag2_dynamic_recorder_interfaces/msg/BagSizeLimitEvent"

# An untouched topic's largest gap across a topic change is one publish interval (0.05s at 20 Hz).
# The bound is loose because an occasional storage stall, which stock `ros2 bag record` shows
# too, can cost every topic a second at once; that is not the behaviour under test.
MAX_TOLERATED_GAP_S = 2.0

# Away from the default 0, so a developer's own nodes cannot join the test graph; overridable for
# anyone who works on 71. Set before any rclpy.init(), so the harness and the recorder subprocess
# land on the same domain.
TEST_DOMAIN_ID = os.environ.get("RDR_TEST_DOMAIN_ID", "71")
os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID
NODE = "/rosbag2_dynamic_recorder"
TOPICS = ["/rdr_test/alpha", "/rdr_test/beta", "/rdr_test/gamma"]
EVENT_TOPIC = f"{NODE}/events/subscription_change"
PAUSE_TOPIC = f"{NODE}/events/pause"
LOW_DISK_TOPIC = f"{NODE}/events/low_disk"
BAG_SIZE_LIMIT_TOPIC = f"{NODE}/events/bag_size_limit"
FILE_SPLIT_TOPIC = f"{NODE}/events/file_split"
FILE_SPLIT_EVENT_TYPE = "rosbag2_dynamic_recorder_interfaces/msg/FileSplitEvent"


class Harness(Node):
    """Publishes test traffic and drives the recorder's services."""

    def __init__(self):
        super().__init__("rdr_test_harness")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = [self.create_publisher(String, t, qos) for t in TOPICS]
        self._seq = 0
        # Appended to every message. Empty by default; the bag-size tests enlarge it so the bag
        # grows on disk in seconds rather than minutes.
        self.payload = ""
        self.create_timer(0.05, self._tick)
        # Not `self._clients`: rclpy.node.Node already uses that name for its own
        # list, and shadowing it breaks create_client().
        self._service_clients = {}

    def _tick(self):
        self._seq += 1
        for pub in self._pubs:
            pub.publish(String(data=f"m{self._seq}{self.payload}"))

    def client(self, srv_type, service):
        key = (srv_type, service)
        if key not in self._service_clients:
            self._service_clients[key] = self.create_client(srv_type, f"{NODE}/{service}")
        return self._service_clients[key]

    # `service`, not `name`: request fields are passed as **fields, and SetProfile has a field
    # called `name`, which would collide with a parameter of that name.
    def call(self, srv_type, service, timeout=20.0, **fields):
        client = self.client(srv_type, service)
        assert client.wait_for_service(timeout_sec=timeout), f"service {service} never appeared"
        request = srv_type.Request()
        for key, value in fields.items():
            setattr(request, key, value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        assert future.done(), f"service {service} timed out"
        return future.result()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def status(self):
        return self.call(GetStatus, "get_status").status


def start_recorder(extra_args=()):
    """Start a recorder process and a harness. Returns (harness, bag, cleanup)."""
    env = dict(os.environ, ROS_DOMAIN_ID=TEST_DOMAIN_ID)
    tmp = tempfile.mkdtemp(prefix="rdr_test_")
    bag = os.path.join(tmp, "bag")
    exe = os.path.join(
        get_package_prefix("rosbag2_dynamic_recorder"),
        "lib", "rosbag2_dynamic_recorder", "dynamic_recorder",
    )
    assert os.path.exists(exe), f"recorder executable not found at {exe}"
    proc = subprocess.Popen(
        [exe, "--ros-args", "-p", f"uri:={bag}", "-p", "status_publish_period:=0.2",
         *extra_args],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )

    rclpy.init(args=None)
    harness = Harness()

    def cleanup():
        harness.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)

    try:
        # Let the recorder come up and the harness's publishers reach the graph.
        harness.spin_for(4.0)
        if proc.poll() is not None:
            raise AssertionError("recorder exited early:\n" + proc.stdout.read())
    except BaseException:
        cleanup()
        raise
    return harness, bag, cleanup


#: Well under one MCAP chunk (~768 KiB), so the first chunk flushed to disk is already past it.
BAG_SIZE_LIMIT = 200_000


def read_bag(uri):
    """Return per-topic receive timestamps (seconds, bag-relative) and the recorded events."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    raw = defaultdict(list)
    events = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        raw[topic].append(stamp)
        if types.get(topic) == EVENT_TYPE:
            events.append(deserialize_message(data, SubscriptionChangeEvent))

    assert raw, "the bag is empty"
    origin = min(min(v) for v in raw.values())
    stamps = {k: sorted((s - origin) / 1e9 for s in v) for k, v in raw.items()}
    return stamps, events


def read_events(uri, type_name, msg_class):
    """The events of one type in the bag, as (bag-relative seconds, event) pairs in recorded order.

    Same time origin as read_bag(), so an event's offset can be compared against the gap or the
    end it is supposed to explain.
    """
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    every_stamp = []
    found = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        every_stamp.append(stamp)
        if types.get(topic) == type_name:
            found.append((stamp, deserialize_message(data, msg_class)))

    assert every_stamp, "the bag is empty"
    origin = min(every_stamp)
    return [((stamp - origin) / 1e9, event) for stamp, event in found]


def widest_gap_window(times):
    """The (start, end) of the largest gap in a sorted series."""
    if len(times) < 2:
        return (0.0, 0.0)
    return max(zip(times, times[1:]), key=lambda pair: pair[1] - pair[0])


def largest_gap(times):
    return max((b - a for a, b in zip(times, times[1:])), default=0.0)


def record_a_topic_change(harness, keep, drop, add, settle=4.0):
    """Record `keep` and `drop`, then swap `drop` for `add`, and close the bag."""
    harness.call(SetTopics, "set_topics", topics=[keep, drop])
    harness.spin_for(settle)
    harness.call(SetTopics, "set_topics", topics=[keep, add])
    harness.spin_for(settle)
    harness.call(Stop, "stop")
    harness.spin_for(1.0)
