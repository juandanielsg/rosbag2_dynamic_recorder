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

"""The free-space and bag-size guards: each stops the recording and says why in the bag."""

from rosbag2_dynamic_recorder_interfaces.msg import BagSizeLimitEvent, LowDiskEvent
from rosbag2_dynamic_recorder_interfaces.srv import SubscribeTopics

from recorder_harness import (
    BAG_SIZE_LIMIT,
    BAG_SIZE_LIMIT_EVENT_TYPE,
    LOW_DISK_EVENT_TYPE,
    Record,
    TOPICS,
    read_events,
)


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
    events = read_events(bag, LOW_DISK_EVENT_TYPE, LowDiskEvent)
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
    events = read_events(bag, BAG_SIZE_LIMIT_EVENT_TYPE, BagSizeLimitEvent)
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
