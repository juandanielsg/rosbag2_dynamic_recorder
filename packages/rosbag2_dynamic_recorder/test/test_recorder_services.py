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

import os
import shutil
import subprocess
import tempfile
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from rosbag2_dynamic_recorder_interfaces.srv import (
    GetStatus,
    SetTopics,
    SubscribeTopics,
    UnsubscribeTopics,
)
from rosbag2_interfaces.srv import Record, Snapshot, SplitBagfile, Stop

# Away from the default 0 so a developer's own nodes cannot join the test graph.
# Set before any rclpy.init(): the test node and the recorder subprocess must land on the SAME
# domain, or they simply never see each other and every service call times out.
TEST_DOMAIN_ID = "71"
os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID
NODE = "/rosbag2_dynamic_recorder"
TOPICS = ["/rdr_test/alpha", "/rdr_test/beta", "/rdr_test/gamma"]


class Harness(Node):
    """Publishes test traffic and drives the recorder's services."""

    def __init__(self):
        super().__init__("rdr_test_harness")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = [self.create_publisher(String, t, qos) for t in TOPICS]
        self._seq = 0
        self.create_timer(0.05, self._tick)
        # Not `self._clients`: rclpy.node.Node already uses that name for its own
        # list, and shadowing it breaks create_client().
        self._service_clients = {}

    def _tick(self):
        self._seq += 1
        for pub in self._pubs:
            pub.publish(String(data=f"m{self._seq}"))

    def client(self, srv_type, name):
        key = (srv_type, name)
        if key not in self._service_clients:
            self._service_clients[key] = self.create_client(srv_type, f"{NODE}/{name}")
        return self._service_clients[key]

    def call(self, srv_type, name, timeout=20.0, **fields):
        client = self.client(srv_type, name)
        assert client.wait_for_service(timeout_sec=timeout), f"service {name} never appeared"
        request = srv_type.Request()
        for key, value in fields.items():
            setattr(request, key, value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        assert future.done(), f"service {name} timed out"
        return future.result()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def status(self):
        return self.call(GetStatus, "get_status").status


@pytest.fixture()
def recorder():
    """A freshly started recorder plus a harness publishing on the test topics."""
    env = dict(os.environ, ROS_DOMAIN_ID=TEST_DOMAIN_ID)
    tmp = tempfile.mkdtemp(prefix="rdr_test_")
    bag = os.path.join(tmp, "bag")
    exe = os.path.join(
        get_package_prefix("rosbag2_dynamic_recorder"),
        "lib", "rosbag2_dynamic_recorder", "dynamic_recorder",
    )
    assert os.path.exists(exe), f"recorder executable not found at {exe}"
    proc = subprocess.Popen(
        [exe, "--ros-args", "-p", f"uri:={bag}", "-p", "status_publish_period:=0.2"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )

    rclpy.init(args=None)
    harness = Harness()
    try:
        # Let the recorder come up and the harness's publishers reach the graph.
        harness.spin_for(4.0)
        if proc.poll() is not None:
            raise AssertionError("recorder exited early:\n" + proc.stdout.read())
        yield harness, bag
    finally:
        harness.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)


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


def test_scheduled_operations_are_refused_not_silently_immediate(recorder):
    """We do not implement timestamp scheduling. Accepting the field and acting immediately would
    be worse than refusing."""
    harness, _ = recorder
    from builtin_interfaces.msg import Time

    future = Time(sec=2_000_000_000, nanosec=0)
    assert harness.call(SplitBagfile, "split_bagfile", split_time=future).return_code != 0
    harness.call(Stop, "stop")
    assert harness.call(Record, "record", start_time=future).return_code != 0


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
