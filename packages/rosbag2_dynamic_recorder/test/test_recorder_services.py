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

"""Integration tests against a real recorder process.

Everything this project claims has, until now, been verified once by hand. These tests lock down
the claims that would be expensive to rediscover:

- changing the topic set does not interrupt the topics you did not touch;
- the bag explains its own sparse channels;
- services refuse for accurate reasons rather than convenient ones;
- stopping is recoverable.

Runs its own publishers on a private ROS_DOMAIN_ID, so it needs no simulator and cannot collide
with anything else on the machine.
"""

import glob
import os
import shutil
import subprocess
import tempfile
import time
from collections import defaultdict

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from rosbag2_dynamic_recorder_interfaces.srv import (
    GetProfiles,
    GetStatus,
    GetSubscribedTopics,
    SetProfile,
    SetTopics,
    SubscribeTopics,
    UnsubscribeTopics,
)
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

from rosbag2_dynamic_recorder_interfaces.msg import (
    BagSizeLimitEvent,
    LowDiskEvent,
    PauseEvent,
    SubscriptionChangeEvent,
)
# The recorder offers Record, Resume, SplitBagfile and Stop under the stock rosbag2_interfaces
# type where the installed definition matches the rosbag2 0.34 shape and under the copy in
# rosbag2_dynamic_recorder_interfaces where it does not (Jazzy, Kilted). dynrec.services encodes
# that rule for clients; this suite cannot import dynrec without a dependency cycle, so it applies
# the same rule here.
from rosbag2_interfaces.srv import Pause, Snapshot, TogglePaused
import rosbag2_interfaces.srv as _stock
import rosbag2_dynamic_recorder_interfaces.srv as _own


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

# In practice the untouched topic's largest gap across two set_topics calls measures 0.05s --
# exactly one publish interval at 20Hz, i.e. no interruption at all. The bound is loose only
# because an intermittent rosbag2 stall -- ~1.2s lost on every topic at once, roughly every 31.2s
# -- costs far more than the behaviour does, and is not ours: it shows up under stock
# `ros2 bag record` too. Tightening this would flake for reasons unrelated to the behaviour under
# test.
#
# Verified non-vacuous: setting this to 0.001 makes the assertion fire and report the real 0.05s.
MAX_TOLERATED_GAP_S = 2.0

# Away from the default 0 so a developer's own nodes cannot join the test graph.
# Set before any rclpy.init(): the test node and the recorder subprocess must land on the SAME
# domain, or they simply never see each other and every service call times out.
# Overridable, because 71 is only "probably unused" -- anyone who happens to work on that domain
# would otherwise see their own nodes join the test graph and get confusing failures.
TEST_DOMAIN_ID = os.environ.get("RDR_TEST_DOMAIN_ID", "71")
os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID
NODE = "/rosbag2_dynamic_recorder"
TOPICS = ["/rdr_test/alpha", "/rdr_test/beta", "/rdr_test/gamma"]
EVENT_TOPIC = f"{NODE}/events/subscription_change"
PAUSE_TOPIC = f"{NODE}/events/pause"
LOW_DISK_TOPIC = f"{NODE}/events/low_disk"
BAG_SIZE_LIMIT_TOPIC = f"{NODE}/events/bag_size_limit"


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


def _start_recorder(extra_args=()):
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


@pytest.fixture()
def recorder():
    """A freshly started recorder plus a harness publishing on the test topics."""
    harness, bag, cleanup = _start_recorder()
    try:
        yield harness, bag
    finally:
        cleanup()


@pytest.fixture()
def paused_recorder():
    """A recorder that opened its bag already paused, so the bag begins with a hole."""
    harness, bag, cleanup = _start_recorder(("-p", "start_paused:=true"))
    try:
        yield harness, bag
    finally:
        cleanup()


@pytest.fixture()
def unexplained_recorder():
    """A recorder with pause events switched off, for the opt-out."""
    harness, bag, cleanup = _start_recorder(("-p", "record_pause_events:=false"))
    try:
        yield harness, bag
    finally:
        cleanup()


@pytest.fixture()
def low_disk_recorder():
    """A recorder whose free-space guard fires at the first check, so it stops itself.

    The minimum is larger than any real disk, so the condition is certain without having to fill
    one. A percentage probe is not used because a fresh tmpfs can legitimately be ~100% free.
    """
    harness, bag, cleanup = _start_recorder((
        "-p", "min_free_space:=9223372036854775807",
        "-p", "storage_check_period:=0.2"))
    try:
        yield harness, bag
    finally:
        cleanup()


def test_low_free_space_stops_recording(low_disk_recorder):
    """The guard protects the filesystem, not the bag: a full disk also stops logging and DDS."""
    harness, _ = low_disk_recorder
    harness.spin_for(1.0)
    status = harness.status()
    assert status.recording is False, "the disk guard did not stop the recording"
    assert status.stopped_for_low_disk is True
    assert status.free_space_bytes > 0, "the status should report the measured free space"
    assert status.total_space_bytes >= status.free_space_bytes
    assert status.min_free_space > 0


def test_low_disk_stop_is_explained_in_the_bag(low_disk_recorder):
    """A recording that just ends is indistinguishable from a crash, so the bag must say why."""
    harness, bag = low_disk_recorder
    harness.spin_for(1.0)
    events = read_low_disk_events(bag)
    assert events, "no LowDiskEvent was recorded; the stop is unexplained"
    _offset, event = events[-1]
    assert event.action == 0, f"expected STOPPED, got {event.action}"
    assert event.free_space_bytes > 0
    assert event.total_space_bytes >= event.free_space_bytes
    assert event.reason, "the event should say what caused it"
    assert event.node_name, "the event should name its sender"


def test_a_recorder_without_the_guard_keeps_recording_and_reports_space(recorder):
    """Non-vacuous: with no minimum set, nothing stops, and free space is still reported."""
    harness, _ = recorder
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.spin_for(1.0)
    status = harness.status()
    assert status.recording is True
    assert status.stopped_for_low_disk is False
    assert status.min_free_space == 0
    assert status.free_space_bytes > 0
    assert status.stopped_for_max_bag_size is False
    assert status.max_bag_size == 0


#: Well under one MCAP chunk (~768 KiB), so the first chunk flushed to disk is already past it.
BAG_SIZE_LIMIT = 200_000


@pytest.fixture()
def size_limited_recorder():
    """A recorder with a bag size cap, fed messages fat enough to reach it within seconds.

    Size on disk is what the guard measures, and MCAP only writes a chunk once it holds ~768 KiB,
    so the harness's usual few-byte strings would take minutes to register. 50 KB per message on
    one topic at 20 Hz is ~1 MB/s: the first chunk lands within a second or two.
    """
    harness, bag, cleanup = _start_recorder((
        "-p", f"max_bag_size:={BAG_SIZE_LIMIT}",
        "-p", "storage_check_period:=0.2"))
    try:
        harness.payload = "x" * 50_000
        harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
        yield harness, bag
    finally:
        cleanup()


def test_a_bag_past_its_size_limit_stops_recording(size_limited_recorder):
    """The cap is on the recording, not the disk: it fires with plenty of free space left."""
    harness, _ = size_limited_recorder
    harness.spin_for(6.0)
    status = harness.status()
    assert status.recording is False, (
        f"the size guard did not stop the recording at {status.bag_size_bytes} bytes")
    assert status.stopped_for_max_bag_size is True
    assert status.stopped_for_low_disk is False, "the wrong guard is being credited"
    assert status.max_bag_size == BAG_SIZE_LIMIT
    assert status.bag_size_bytes > BAG_SIZE_LIMIT, "stopped without the bag being over the limit"


def test_a_size_limit_stop_is_explained_in_the_bag(size_limited_recorder):
    """Same contract as the disk guard: the bag must say why it ended."""
    harness, bag = size_limited_recorder
    harness.spin_for(6.0)
    assert harness.status().recording is False
    events = read_bag_size_limit_events(bag)
    assert events, "no BagSizeLimitEvent was recorded; the stop is unexplained"
    _offset, event = events[-1]
    assert event.action == 0, f"expected STOPPED, got {event.action}"
    assert event.bag_size_bytes > BAG_SIZE_LIMIT
    assert event.max_bag_size == BAG_SIZE_LIMIT
    assert event.reason, "the event should say what caused it"
    assert event.node_name, "the event should name its sender"


def test_record_after_a_size_limit_stop_opens_a_fresh_bag_and_re_arms(size_limited_recorder):
    """The cap is per bag, not per node: a new recording starts from zero and is capped again."""
    harness, _ = size_limited_recorder
    harness.spin_for(6.0)
    assert harness.status().stopped_for_max_bag_size is True
    harness.payload = ""  # So the second bag stays under the cap long enough to be observed.
    assert harness.call(Record, "record").return_code == 0
    status = harness.status()
    assert status.recording is True
    assert status.stopped_for_max_bag_size is False
    assert status.max_bag_size == BAG_SIZE_LIMIT


@pytest.fixture()
def profiled_recorder():
    """A recorder configured with two deliberately overlapping profiles."""
    harness, bag, cleanup = _start_recorder((
        "-p", "profile_names:=[small,large,ghost]",
        "-p", f"profiles.small:=[{TOPICS[0]}]",
        "-p", f"profiles.large:=[{TOPICS[0]},{TOPICS[1]}]",
        "-p", "profiles.ghost:=[/rdr_test/nobody_publishes_this]",
    ))
    try:
        yield harness, bag
    finally:
        cleanup()


def test_subscribe_then_status_reports_it(recorder):
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    assert response.return_code == 0, response.error_string
    assert list(response.subscribed_topics) == [TOPICS[0]]
    assert list(harness.status().subscribed_topics) == [TOPICS[0]]


def test_set_topics_leaves_shared_topics_alone(recorder):
    """The project's core claim, at the API level: a topic present before and after a change is
    never reported as touched."""
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=TOPICS[:2])
    response = harness.call(SetTopics, "set_topics", topics=[TOPICS[1], TOPICS[2]])

    assert response.return_code == 0, response.error_string
    assert TOPICS[0] in response.unsubscribed_topics, "the dropped topic should be reported"
    assert TOPICS[1] not in response.unsubscribed_topics, (
        "a topic in both the old and new set must not be torn down"
    )
    assert set(harness.status().subscribed_topics) == {TOPICS[1], TOPICS[2]}


def test_regex_selects_topics_without_naming_them(recorder):
    """The point of the feature: "record all of these" without enumerating them."""
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", regex="/rdr_test/")
    assert response.return_code == 0, response.error_string
    assert sorted(response.subscribed_topics) == sorted(TOPICS)


def test_regex_and_explicit_topics_combine(recorder):
    harness, _ = recorder
    response = harness.call(
        SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]], regex="gamma")
    assert response.return_code == 0, response.error_string
    assert sorted(response.subscribed_topics) == sorted([TOPICS[0], TOPICS[2]])


def test_exclude_regex_filters_explicitly_named_topics_too(recorder):
    """Applied to the combined set, which is what makes "all of X except Y" one call."""
    harness, _ = recorder
    response = harness.call(
        SubscribeTopics, "subscribe_topics",
        topics=[TOPICS[1]], regex="/rdr_test/", exclude_regex="beta")
    assert response.return_code == 0, response.error_string
    assert TOPICS[1] not in response.subscribed_topics, (
        "exclude_regex should filter a topic even when it was named explicitly"
    )
    assert sorted(response.subscribed_topics) == sorted([TOPICS[0], TOPICS[2]])


def test_a_pattern_never_matches_the_recorders_own_topics(recorder):
    """Its event channels are written into the bag directly; subscribing would double-record."""
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", regex=".*")
    assert response.return_code == 0, response.error_string
    ours = [t for t in response.subscribed_topics if t.startswith(NODE + "/")]
    assert ours == [], f"a pattern pulled in our own topics: {ours}"
    # Non-vacuous: '.*' really did match a great deal else.
    assert set(TOPICS).issubset(set(response.subscribed_topics))


def test_invalid_regex_is_refused_with_the_reason(recorder):
    """A typo must not look identical to a pattern that legitimately matched nothing."""
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", regex="/rdr_test/[")
    assert response.return_code != 0
    assert "invalid regular expression" in response.error_string
    assert harness.call(GetSubscribedTopics, "get_subscribed_topics").topics == [], (
        "a refused request should not have subscribed anything"
    )


def test_regex_matching_nothing_is_refused_on_subscribe(recorder):
    """Subscribe exists to add topics, so adding none is a request that was not met."""
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", regex="^/nothing_matches_this")
    assert response.return_code != 0
    assert "matched no topics" in response.error_string


def test_set_topics_regex_replaces_the_selection(recorder):
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=[TOPICS[1]])
    response = harness.call(SetTopics, "set_topics", regex="alpha")
    assert response.return_code == 0, response.error_string
    assert response.subscribed_topics == [TOPICS[0]]
    assert TOPICS[1] in response.unsubscribed_topics


def test_set_topics_reports_total_failure(recorder):
    """Regression: a set_topics whose every topic failed used to report success, so a caller
    could not tell "all eight missing" from "all eight recorded" without parsing the lists."""
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=TOPICS[:2])

    response = harness.call(
        SetTopics, "set_topics", topics=["/rdr_test/nobody_publishes_this"])
    assert response.return_code != 0, "an all-unavailable set_topics is a failure"
    assert "/rdr_test/nobody_publishes_this" in response.unavailable_topics
    # The old selection was dropped before the add failed, and the add did not land.
    assert list(response.subscribed_topics) == []
    assert not harness.status().subscribed_topics


def test_unsubscribe_regex_matches_what_is_recorded_not_the_graph(recorder):
    """Only the recorded subset can be dropped, so that is the pool a pattern searches."""
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=TOPICS[:2])
    response = harness.call(UnsubscribeTopics, "unsubscribe_topics", regex="/rdr_test/")
    assert response.return_code == 0, response.error_string
    # gamma is on the graph and matches the pattern, but was never recorded, so it is not
    # reported as dropped and not reported as an error either.
    assert sorted(response.unsubscribed_topics) == sorted(TOPICS[:2])
    assert TOPICS[2] not in response.unsubscribed_topics
    assert response.not_subscribed_topics == []


def test_unsubscribe_reports_topics_it_was_not_recording(recorder):
    harness, _ = recorder
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    response = harness.call(UnsubscribeTopics, "unsubscribe_topics", topics=TOPICS[:2])
    assert list(response.unsubscribed_topics) == [TOPICS[0]]
    assert list(response.not_subscribed_topics) == [TOPICS[1]]


def test_stopped_recorder_refuses_with_the_real_reason(recorder):
    """Regression: this used to report the topics as 'unavailable', which means absent from the
    graph or ambiguous -- a confidently wrong answer."""
    harness, _ = recorder
    assert harness.call(Stop, "stop").return_code == 0

    response = harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    assert response.return_code != 0
    assert "stopped" in response.error_string.lower()
    assert not response.unavailable_topics, "the topics are fine; the recorder is not"


def test_resume_is_refused_while_stopped(recorder):
    """A resume against no open bag can only arm a timer for a recording that will not exist."""
    harness, _ = recorder
    assert harness.call(Stop, "stop").return_code == 0

    response = harness.call(Resume, "resume")
    assert response.return_code != 0, "resuming a stopped recorder should be refused"
    assert "stopped" in response.error_string.lower()


def test_second_stop_is_reported_not_silently_accepted(recorder):
    harness, _ = recorder
    assert harness.call(Stop, "stop").return_code == 0
    assert harness.call(Stop, "stop").return_code != 0


def test_record_reopens_and_restores_the_topic_selection(recorder):
    """Regression: stop used to be a dead end with no way to reopen a bag."""
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=TOPICS[:2])
    before = set(harness.status().subscribed_topics)
    harness.call(Stop, "stop")
    assert harness.status().recording is False

    assert harness.call(Record, "record").return_code == 0
    harness.spin_for(3.0)
    status = harness.status()
    assert status.recording is True
    assert set(status.subscribed_topics) == before, "the previous selection should come back"
    assert status.uri, "a new bag should have been opened"


def test_scheduled_resume_fires_on_node_time(recorder):
    """A node-time schedule must fire from a timer, so it works on a robot with no traffic."""
    harness, _ = recorder
    from builtin_interfaces.msg import Time

    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.call(Pause, "pause")
    assert harness.status().paused is True

    soon = harness.get_clock().now().nanoseconds + 3_000_000_000
    response = harness.call(
        Resume, "resume",
        resume_time=Time(sec=soon // 10**9, nanosec=soon % 10**9), resume_mode=0)
    assert response.return_code == 0, response.error_string
    assert harness.status().paused is True, "it should not have resumed yet"

    harness.spin_for(5.0)
    assert harness.status().paused is False, "the scheduled resume did not fire"


def test_scheduled_resume_fires_on_message_time(recorder):
    """Receive-time mode is evaluated against arriving messages, not the clock."""
    harness, _ = recorder
    from builtin_interfaces.msg import Time

    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.call(Pause, "pause")

    soon = harness.get_clock().now().nanoseconds + 2_000_000_000
    response = harness.call(
        Resume, "resume",
        resume_time=Time(sec=soon // 10**9, nanosec=soon % 10**9),
        resume_mode=2, tracking_topic_name=TOPICS[0])
    assert response.return_code == 0, response.error_string

    harness.spin_for(5.0)
    assert harness.status().paused is False


def test_scheduled_split_produces_a_second_file(recorder):
    harness, bag = recorder
    from builtin_interfaces.msg import Time

    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    soon = harness.get_clock().now().nanoseconds + 3_000_000_000
    response = harness.call(
        SplitBagfile, "split_bagfile",
        split_time=Time(sec=soon // 10**9, nanosec=soon % 10**9), split_mode=0)
    assert response.return_code == 0, response.error_string
    assert harness.status().bag_splits == 0, "it should not have split yet"

    harness.spin_for(5.0)
    assert harness.status().bag_splits == 1, "the scheduled split did not fire"


def test_rescheduling_replaces_the_pending_node_time_timer(recorder):
    """A second node-time schedule must replace the first, not be cancelled by it.

    Regression: the timer callback cancelled whichever timer the member currently pointed at, so a
    superseded timer would cancel the live replacement and then, being a repeating timer, fire its
    action again on every period.
    """
    harness, _ = recorder
    from builtin_interfaces.msg import Time

    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    first = harness.get_clock().now().nanoseconds + 3_000_000_000
    second = harness.get_clock().now().nanoseconds + 8_000_000_000
    assert harness.call(
        SplitBagfile, "split_bagfile",
        split_time=Time(sec=first // 10**9, nanosec=first % 10**9),
        split_mode=0).return_code == 0
    assert harness.call(
        SplitBagfile, "split_bagfile",
        split_time=Time(sec=second // 10**9, nanosec=second % 10**9),
        split_mode=0).return_code == 0

    # Past the superseded first timer's deadline: it must not have fired.
    harness.spin_for(5.0)
    assert harness.status().bag_splits == 0, "the superseded schedule fired"

    # The replacement fires once, and does not keep firing.
    harness.spin_for(5.0)
    assert harness.status().bag_splits == 1, "the replacement did not fire exactly once"
    harness.spin_for(4.0)
    assert harness.status().bag_splits == 1, "the timer fired more than once"


def test_scheduled_record_starts_later(recorder):
    harness, _ = recorder
    from builtin_interfaces.msg import Time

    harness.call(Stop, "stop")
    soon = harness.get_clock().now().nanoseconds + 3_000_000_000
    response = harness.call(
        Record, "record", start_time=Time(sec=soon // 10**9, nanosec=soon % 10**9))
    assert response.return_code == 0, response.error_string
    assert harness.status().recording is False, "it should not have started yet"

    harness.spin_for(5.0)
    assert harness.status().recording is True, "the scheduled recording did not start"


def test_invalid_schedule_is_rejected(recorder):
    """A mode we cannot honour, or a topic nobody records, would wait forever. Refuse instead."""
    harness, _ = recorder
    from builtin_interfaces.msg import Time

    soon = Time(sec=2_000_000_000, nanosec=0)
    bad_mode = harness.call(Resume, "resume", resume_time=soon, resume_mode=99)
    assert bad_mode.return_code == Resume.Response.RETURN_CODE_INVALID_RESUME_MODE

    bad_topic = harness.call(
        Resume, "resume", resume_time=soon, resume_mode=2,
        tracking_topic_name="/nobody/records/this")
    assert bad_topic.return_code == Resume.Response.RETURN_CODE_INVALID_TRACKING_TOPIC


def test_snapshot_without_snapshot_mode_fails_clearly(recorder):
    harness, _ = recorder
    assert harness.call(Snapshot, "snapshot").success is False


def test_status_never_claims_loss_it_cannot_measure(recorder):
    """messages_missed is only meaningful alongside sequence_numbers_available."""
    harness, _ = recorder
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.spin_for(2.0)
    status = harness.status()
    assert status.messages_written > 0, "nothing was recorded; the harness may not be publishing"
    if not status.sequence_numbers_available:
        pytest.skip("middleware does not supply publication sequence numbers")
    assert status.messages_missed >= 0


def test_loss_is_reported_by_origin_and_the_total_is_their_sum(recorder):
    """Transport loss and writer loss have opposite remedies, so the status carries both, and
    the legacy total is exactly their sum rather than a third number that could drift."""
    harness, _ = recorder
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.spin_for(2.0)
    status = harness.status()
    assert status.messages_lost == (
        status.messages_lost_in_transport + status.messages_lost_in_recorder)


def test_storage_options_reach_the_writer():
    """max_bagfile_duration is passed straight to rosbag2, so a one-second limit shows up as a
    split the recorder counts. Proves the pass-through rather than the parameter declaration."""
    harness, _, cleanup = _start_recorder(("-p", "max_bagfile_duration:=1"))
    try:
        harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
        harness.spin_for(4.0)
        assert harness.status().bag_splits >= 1, "the duration limit never split the bag"
    finally:
        cleanup()


# A time bound on the writer cache reached rosbag2 on Rolling only. rosbag2_py mirrors the C++
# StorageOptions field for field, so this is the same probe the recorder's CMake runs.
WRITER_HAS_CACHE_DURATION = hasattr(StorageOptions(uri=""), "max_cache_duration")


@pytest.mark.skipif(not WRITER_HAS_CACHE_DURATION, reason="this rosbag2 has no max_cache_duration")
def test_snapshot_mode_accepts_a_duration_bound_alone():
    """The writer treats a duration-limited cache as a valid buffer, so snapshot_mode must not
    insist on max_cache_size when max_cache_duration is set."""
    harness, _, cleanup = _start_recorder((
        "-p", "snapshot_mode:=true", "-p", "max_cache_size:=0", "-p", "max_cache_duration:=2"))
    try:
        assert harness.status().snapshot_mode is True
    finally:
        cleanup()


@pytest.mark.skipif(WRITER_HAS_CACHE_DURATION, reason="this rosbag2 has max_cache_duration")
def test_cache_duration_is_refused_where_the_writer_has_none():
    """On Jazzy and Kilted the parameter exists so a params file loads everywhere, but a non-zero
    value would be silently dropped. Refusing at startup is the honest alternative."""
    with pytest.raises(AssertionError, match="exited early"):
        _start_recorder(("-p", "max_cache_duration:=2"))


def test_unknown_storage_preset_is_refused_at_startup():
    """A typo in the preset is a configuration error, and it should fail before a bag exists
    rather than record on defaults while claiming otherwise."""
    with pytest.raises(AssertionError, match="exited early"):
        _start_recorder(("-p", "storage_preset_profile:=fastwrit"))


def test_an_impossible_free_space_percentage_is_refused_at_startup():
    """A percentage over 100 can never be satisfied, so it would stop every recording the instant
    it started. Refusing it at startup says so instead of producing empty bags."""
    with pytest.raises(AssertionError, match="exited early"):
        _start_recorder(("-p", "min_free_space_percent:=150"))


def test_a_negative_bag_size_limit_is_refused_at_startup():
    """Rejected like the other byte counts rather than wrapping to an enormous unsigned limit."""
    with pytest.raises(AssertionError, match="exited early"):
        _start_recorder(("-p", "max_bag_size:=-1"))


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


def read_pause_events(uri):
    """PauseEvents in the bag, as (bag-relative seconds, event) pairs in recorded order.

    Same time origin as read_bag(), so an event's offset can be compared directly against the gap
    it is supposed to explain. That comparison is the whole point: an event carrying the right
    action but landing nowhere near the hole would explain nothing.
    """
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    every_stamp = []
    found = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        every_stamp.append(stamp)
        if types.get(topic) == PAUSE_EVENT_TYPE:
            found.append((stamp, deserialize_message(data, PauseEvent)))

    assert every_stamp, "the bag is empty"
    origin = min(every_stamp)
    return [((stamp - origin) / 1e9, event) for stamp, event in found]


def read_low_disk_events(uri):
    """LowDiskEvents in the bag, as (bag-relative seconds, event) pairs.

    Same time origin as read_bag(), so the event's offset can be compared against the end of the
    data it explains -- the two must coincide, or the bag does not actually say why it ended.
    """
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    every_stamp = []
    found = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        every_stamp.append(stamp)
        if types.get(topic) == LOW_DISK_EVENT_TYPE:
            found.append((stamp, deserialize_message(data, LowDiskEvent)))

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


def test_topic_change_does_not_split_the_bag(recorder):
    """A change must not close the file. Stock rosbag2 cannot avoid this, which is the point."""
    harness, bag = recorder
    record_a_topic_change(harness, TOPICS[0], TOPICS[1], TOPICS[2])

    files = glob.glob(os.path.join(bag, "*.mcap"))
    assert len(files) == 1, f"expected one continuous file, got {files}"


def test_untouched_topic_is_not_interrupted_by_a_change(recorder):
    """The project's central claim, measured in the bag rather than asserted.

    The kept topic must span the whole recording with no meaningful hole, while the swap happens
    around it.
    """
    harness, bag = recorder
    keep, drop, add = TOPICS
    record_a_topic_change(harness, keep, drop, add)

    stamps, _ = read_bag(bag)
    assert keep in stamps, "the kept topic recorded nothing"
    kept = stamps[keep]

    assert largest_gap(kept) < MAX_TOLERATED_GAP_S, (
        f"the untouched topic was interrupted: largest gap {largest_gap(kept):.2f}s"
    )
    # It must straddle the switch, not merely exist: data before the dropped topic ended and
    # after the added one began.
    assert kept[0] < stamps[drop][-1], "kept topic started after the dropped one had ended"
    assert kept[-1] > stamps[add][0], "kept topic ended before the added one began"


def test_dropped_and_added_topics_are_sparse_channels(recorder):
    """MCAP supports channels that cover only part of the recording; that is what makes a
    continuous single-file bag possible at all."""
    harness, bag = recorder
    keep, drop, add = TOPICS
    record_a_topic_change(harness, keep, drop, add)

    stamps, _ = read_bag(bag)
    assert stamps[drop][0] < stamps[add][0], "the dropped topic should start first"
    assert stamps[drop][-1] < stamps[add][-1], "the dropped topic should end first"
    assert stamps[add][0] > 1.0, "the added topic should begin partway through, not at the start"


def test_bag_explains_its_own_sparse_channels(recorder):
    """A channel that stops mid-bag is otherwise indistinguishable from a dropout or a crash.

    The recorded events are what make a deliberate change legible after the fact, so they have to
    be in the bag, carry the reason, and line up with the channel boundary.
    """
    harness, bag = recorder
    keep, drop, add = TOPICS
    record_a_topic_change(harness, keep, drop, add)

    stamps, events = read_bag(bag)
    assert events, "no SubscriptionChangeEvent was recorded; sparse channels are unexplained"

    dropped = [e for e in events if e.topic_name == drop and e.action == UNSUBSCRIBED]
    added = [e for e in events if e.topic_name == add and e.action == SUBSCRIBED]
    assert dropped, f"no UNSUBSCRIBED event for {drop}"
    assert added, f"no SUBSCRIBED event for {add}"

    assert "set_topics" in dropped[0].reason, (
        f"the event should say what caused it, got {dropped[0].reason!r}"
    )
    assert dropped[0].node_name, "the event should name its sender"

    # The explanation is only useful if it sits where the data stops.
    event_offset = min(stamps[EVENT_TOPIC])
    assert EVENT_TOPIC in stamps, "the event channel itself should be in the bag"
    assert event_offset >= 0.0


PAUSE_SECONDS = 6.0


def test_bag_explains_its_own_pause(recorder):
    """A pause leaves a hole in EVERY topic at once, which is what a crash looks like too.

    SubscriptionChangeEvent explains a channel that stops; this is the case it does not cover, and
    the one the provenance mechanism was built for. The assertion is deliberately two-sided: the
    gap has to be really there, and the events have to sit at its edges. Either half alone would
    pass on a bag that explains nothing.
    """
    harness, bag = recorder
    harness.call(SetTopics, "set_topics", topics=[TOPICS[0]])
    harness.spin_for(3.0)
    harness.call(Pause, "pause")
    harness.spin_for(PAUSE_SECONDS)
    harness.call(Resume, "resume")
    harness.spin_for(3.0)
    harness.call(Stop, "stop")
    harness.spin_for(1.0)

    stamps, _ = read_bag(bag)
    events = read_pause_events(bag)

    # The hole is real, and far larger than the ~1.2s environmental stall in recording-stall.md.
    gap_start, gap_end = widest_gap_window(stamps[TOPICS[0]])
    assert gap_end - gap_start > PAUSE_SECONDS / 2, (
        f"expected a pause-sized hole, got {gap_end - gap_start:.2f}s"
    )

    actions = [event.action for _offset, event in events]
    assert actions == [PAUSED, RESUMED], f"expected one pause and one resume, got {actions}"

    (paused_at, paused_event), (resumed_at, resumed_event) = events
    assert abs(paused_at - gap_start) < 1.0, (
        f"the PAUSED event is at {paused_at:.2f}s but the hole starts at {gap_start:.2f}s"
    )
    assert abs(resumed_at - gap_end) < 1.0, (
        f"the RESUMED event is at {resumed_at:.2f}s but the hole ends at {gap_end:.2f}s"
    )
    assert paused_event.reason == "service:pause", paused_event.reason
    assert resumed_event.reason == "service:resume", resumed_event.reason
    assert paused_event.node_name, "the event should name its sender"


def test_pause_events_are_written_even_though_the_pause_discards_everything_else(recorder):
    """The event has to escape the gate it is describing, or it could never be recorded."""
    harness, bag = recorder
    harness.call(SetTopics, "set_topics", topics=[TOPICS[0]])
    harness.spin_for(2.0)
    harness.call(Pause, "pause")
    harness.spin_for(3.0)

    written_while_paused = harness.status().messages_written
    harness.spin_for(3.0)
    assert harness.status().messages_written == written_while_paused, (
        "messages were written while paused; the gate is not doing its job"
    )

    harness.call(Stop, "stop")
    harness.spin_for(1.0)
    events = read_pause_events(bag)
    assert [event.action for _offset, event in events] == [PAUSED], (
        "the PAUSED event was suppressed by the very pause it describes"
    )


def test_toggle_and_schedule_are_distinguishable_from_a_plain_pause(recorder):
    """The reason is what tells an operator's pause apart from one a schedule caused."""
    harness, bag = recorder
    from builtin_interfaces.msg import Time

    harness.call(SetTopics, "set_topics", topics=[TOPICS[0]])
    harness.spin_for(2.0)
    harness.call(TogglePaused, "toggle_paused")
    harness.spin_for(1.0)

    soon = harness.get_clock().now().nanoseconds + 3_000_000_000
    harness.call(
        Resume, "resume",
        resume_time=Time(sec=soon // 10**9, nanosec=soon % 10**9), resume_mode=0)
    harness.spin_for(5.0)
    harness.call(Stop, "stop")
    harness.spin_for(1.0)

    reasons = [event.reason for _offset, event in read_pause_events(bag)]
    assert reasons == ["service:toggle_paused", "schedule:resume"], reasons


def test_a_bag_that_starts_paused_says_why_its_head_is_empty(paused_recorder):
    """Otherwise the channels look like they failed the moment they were created."""
    harness, bag = paused_recorder
    harness.call(SetTopics, "set_topics", topics=[TOPICS[0]])
    harness.spin_for(3.0)
    harness.call(Resume, "resume")
    harness.spin_for(2.0)
    harness.call(Stop, "stop")
    harness.spin_for(1.0)

    events = read_pause_events(bag)
    assert events, "a recorder started paused recorded nothing to explain it"
    _offset, first = events[0]
    assert first.action == PAUSED
    assert first.reason == "startup", first.reason


def test_pause_events_can_be_turned_off(unexplained_recorder):
    """Same opt-out as record_subscription_events, and non-vacuous: the pause still happens."""
    harness, bag = unexplained_recorder
    harness.call(SetTopics, "set_topics", topics=[TOPICS[0]])
    harness.spin_for(2.0)
    harness.call(Pause, "pause")
    harness.spin_for(3.0)
    assert harness.status().paused is True, "the pause itself should still take effect"
    harness.call(Resume, "resume")
    harness.spin_for(2.0)
    harness.call(Stop, "stop")
    harness.spin_for(1.0)

    assert read_pause_events(bag) == [], "record_pause_events:=false still wrote events"


def test_events_are_recorded_for_the_initial_subscription_too(recorder):
    """Otherwise a channel that starts at t=0 has no provenance at all."""
    harness, bag = recorder
    harness.call(SetTopics, "set_topics", topics=[TOPICS[0]])
    harness.spin_for(3.0)
    harness.call(Stop, "stop")
    harness.spin_for(1.0)

    _stamps, events = read_bag(bag)
    first = [e for e in events if e.topic_name == TOPICS[0] and e.action == SUBSCRIBED]
    assert first, "the initial subscription was not explained"
    assert first[0].reason, "every event should carry a reason"


def test_profiles_are_offered_as_configured(profiled_recorder):
    harness, _ = profiled_recorder
    response = harness.call(GetProfiles, "get_profiles")
    names = [p.name for p in response.profiles]
    assert names == ["small", "large", "ghost"], "declaration order should be preserved for a UI to list"


def test_switching_profiles_does_not_touch_shared_topics(profiled_recorder):
    """The fleet case: small and large share a topic, and switching must not interrupt it."""
    harness, _ = profiled_recorder
    harness.call(SetProfile, "set_profile", name="small")
    response = harness.call(SetProfile, "set_profile", name="large")

    assert response.return_code == 0, response.error_string
    assert TOPICS[0] not in response.unsubscribed_topics, (
        "the topic shared by both profiles was torn down"
    )
    assert set(response.subscribed_topics) == {TOPICS[0], TOPICS[1]}


def test_active_profile_is_derived_not_remembered(profiled_recorder):
    """A remembered name would still read 'large' after someone changed a topic by hand, which
    would be a lie in the status."""
    harness, _ = profiled_recorder
    harness.call(SetProfile, "set_profile", name="large")
    assert harness.status().active_profile == "large"

    # Add a topic no profile lists, so the selection matches nothing.
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[2]])
    assert harness.status().active_profile == "", (
        "the selection no longer matches any profile, so none is active"
    )

    # Returning to exactly a profile's set makes it active again, with no set_profile call.
    harness.call(UnsubscribeTopics, "unsubscribe_topics", topics=[TOPICS[2]])
    assert harness.status().active_profile == "large"


def test_active_profile_follows_the_topics_not_the_last_command(profiled_recorder):
    """Derived means derived: dropping a topic from `large` lands exactly on `small`, and the
    status says so even though set_profile was never called with that name.

    This is the behaviour a remembered label could not produce, and it is the honest one -- the
    recorder really is recording the small profile at that point.
    """
    harness, _ = profiled_recorder
    harness.call(SetProfile, "set_profile", name="large")
    harness.call(UnsubscribeTopics, "unsubscribe_topics", topics=[TOPICS[1]])
    assert harness.status().active_profile == "small"


def test_unknown_profile_says_what_is_configured(profiled_recorder):
    harness, _ = profiled_recorder
    response = harness.call(SetProfile, "set_profile", name="nonexistent")
    assert response.return_code != 0
    assert "small" in response.error_string and "large" in response.error_string, (
        "the error should tell the caller what they could have asked for"
    )


def test_profile_reports_total_failure(profiled_recorder):
    """Mirror of the set_topics rule: a profile none of whose topics could be subscribed is a
    failure the caller has to see, not a success with an empty list."""
    harness, _ = profiled_recorder
    harness.call(SetProfile, "set_profile", name="small")

    response = harness.call(SetProfile, "set_profile", name="ghost")
    assert response.return_code != 0, "an all-unavailable profile is a failure"
    assert "/rdr_test/nobody_publishes_this" in response.unavailable_topics
    assert list(response.subscribed_topics) == []


def test_profiles_absent_when_none_configured(recorder):
    harness, _ = recorder
    assert harness.call(GetProfiles, "get_profiles").profiles == []
    assert harness.status().active_profile == ""


def test_invalid_topic_name_is_refused_not_fatal(recorder):
    """Regression: this used to terminate the process.

    Supplying topic_types skips the graph lookup, so a bad name reaches
    create_generic_subscription directly. An exception there escapes the service callback and
    takes the whole node down, losing the recording along with it.
    """
    harness, _ = recorder
    response = harness.call(
        SubscribeTopics, "subscribe_topics",
        topics=["not a valid topic name!"], topic_types=["std_msgs/msg/String"],
    )
    assert response.return_code != 0
    assert response.error_string, "the failure should say what went wrong"

    # The point of the test: the recorder is still alive and usable afterwards.
    assert harness.status().recording is True
    ok = harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    assert ok.return_code == 0, "the node should still work after a rejected request"


def test_unloadable_type_is_refused_not_fatal(recorder):
    """Same path, reached through a type that has no message library to load."""
    harness, _ = recorder
    response = harness.call(
        SubscribeTopics, "subscribe_topics",
        topics=[TOPICS[0]], topic_types=["no_such_pkg/msg/NoSuchType"],
    )
    assert response.return_code != 0
    assert harness.status().recording is True


def test_changed_type_on_a_known_topic_is_refused(recorder):
    """A bag channel is bound to one type. Writing a different type into it would silently
    corrupt the recording, so the subscribe is refused instead."""
    harness, _ = recorder
    assert harness.call(
        SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]]).return_code == 0
    harness.call(UnsubscribeTopics, "unsubscribe_topics", topics=[TOPICS[0]])

    response = harness.call(
        SubscribeTopics, "subscribe_topics",
        topics=[TOPICS[0]], topic_types=["sensor_msgs/msg/Imu"],
    )
    assert response.return_code != 0
    assert "already in this bag" in response.error_string, response.error_string
    assert harness.status().recording is True


def test_status_reports_no_write_errors_on_a_healthy_run(recorder):
    """write_errors is known-lost data, so a healthy recording must report exactly zero."""
    harness, _ = recorder
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.spin_for(2.0)
    assert harness.status().write_errors == 0


def read_bag_size_limit_events(uri):
    """BagSizeLimitEvents in the bag, as (bag-relative seconds, event) pairs. See
    read_low_disk_events()."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    every_stamp = []
    found = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        every_stamp.append(stamp)
        if types.get(topic) == BAG_SIZE_LIMIT_EVENT_TYPE:
            found.append((stamp, deserialize_message(data, BagSizeLimitEvent)))

    assert every_stamp, "the bag is empty"
    origin = min(every_stamp)
    return [((stamp - origin) / 1e9, event) for stamp, event in found]
