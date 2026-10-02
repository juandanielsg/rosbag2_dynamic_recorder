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

"""Stop, record, pause, resume, split and snapshot, now and scheduled; the status they report;
and parameters refused at startup."""

import pytest
from rclpy.qos import QoSProfile
from rclpy.serialization import deserialize_message
from rosbag2_dynamic_recorder_interfaces.msg import FileSplitEvent, ScheduledAction
from rosbag2_dynamic_recorder_interfaces.srv import SetTopics, SubscribeTopics
from rosbag2_interfaces.srv import Pause, Snapshot
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

from recorder_harness import (
    FILE_SPLIT_EVENT_TYPE,
    FILE_SPLIT_TOPIC,
    Record,
    Resume,
    SplitBagfile,
    Stop,
    TOPICS,
    start_recorder,
)


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


def test_a_stopped_recorder_stops_counting_but_its_stamp_does_not(recorder):
    """elapsed_seconds kept growing after a stop, and the UI's timeline grew with it."""
    harness, _ = recorder
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.spin_for(1.0)
    harness.call(Stop, "stop")
    first = harness.status()
    harness.spin_for(2.0)
    second = harness.status()
    assert first.elapsed_seconds > 0, "the bag ran for a while before the stop"
    assert second.elapsed_seconds == first.elapsed_seconds, "a stopped bag does not keep running"
    seconds = lambda t: t.sec + t.nanosec / 1e9  # noqa: E731
    assert seconds(second.stamp) - seconds(first.stamp) >= 1.5, "the recorder's clock keeps going"


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


def test_status_lists_pending_schedules_until_they_fire_or_a_stop_clears_them(recorder):
    """A UI can only show what is queued if the recorder says so, whoever queued it."""
    harness, _ = recorder
    from builtin_interfaces.msg import Time

    def at(nanoseconds):
        return Time(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)

    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.call(Pause, "pause")
    assert list(harness.status().schedules) == [], "nothing queued yet"

    now = harness.get_clock().now().nanoseconds
    assert harness.call(Resume, "resume", resume_time=at(now + 3 * 10**9),
                        resume_mode=0).return_code == 0
    far = now + 3600 * 10**9
    assert harness.call(SplitBagfile, "split_bagfile", split_time=at(far), split_mode=2,
                        tracking_topic_name=TOPICS[0]).return_code == 0

    resume, split = harness.status().schedules
    assert (resume.action, resume.mode) == (ScheduledAction.RESUME, ScheduledAction.NODE_TIME)
    assert abs(resume.time.sec - (now + 3 * 10**9) // 10**9) <= 1
    assert (split.action, split.mode) == (ScheduledAction.SPLIT, ScheduledAction.RECEIVE_TIME)
    assert split.tracking_topic == TOPICS[0]
    assert split.time.sec == far // 10**9

    harness.spin_for(5.0)
    status = harness.status()
    assert status.paused is False, "the scheduled resume did not fire"
    assert [s.action for s in status.schedules] == [ScheduledAction.SPLIT], (
        "a fired schedule must leave the list")

    harness.call(Stop, "stop")
    assert list(harness.status().schedules) == [], "a stop clears pending resumes and splits"

    later = harness.get_clock().now().nanoseconds + 3600 * 10**9
    assert harness.call(Record, "record", start_time=at(later)).return_code == 0
    (record,) = harness.status().schedules
    assert record.action == ScheduledAction.RECORD, "a scheduled record shows while stopped"


def test_a_rollover_is_explained_by_a_stamped_event_in_the_new_file(recorder):
    """WriteSplitEvent has no stamp and no cause; FileSplitEvent carries both, into the bag too.

    Also pins the split count: rosbag2 reports closing the bag the way it reports a rollover, and
    counting that put one file too many in the status of every stopped recorder.
    """
    harness, bag = recorder
    from rclpy.qos import DurabilityPolicy

    received = []
    harness.create_subscription(
        FileSplitEvent, FILE_SPLIT_TOPIC, received.append,
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.spin_for(1.0)

    assert harness.call(SplitBagfile, "split_bagfile").return_code == 0
    harness.spin_for(2.0)
    harness.call(Stop, "stop")
    harness.spin_for(0.5)

    assert len(received) == 1, f"expected one FileSplitEvent, got {len(received)}"
    event = received[0]
    assert event.reason == "service:split_bagfile"
    assert event.closed_file and event.opened_file and event.closed_file != event.opened_file
    assert event.stamp.sec > 0, "the event must be placeable on the recording's timeline"
    assert harness.status().bag_splits == 1, "closing the bag is not a rollover"

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    recorded = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if types.get(topic) == FILE_SPLIT_EVENT_TYPE:
            recorded.append(deserialize_message(data, FileSplitEvent))
    assert [e.reason for e in recorded] == ["service:split_bagfile"], (
        "the rollover must be explained inside the bag, not only on the topic")


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


def test_status_reports_no_write_errors_on_a_healthy_run(recorder):
    """write_errors is known-lost data, so a healthy recording must report exactly zero."""
    harness, _ = recorder
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    harness.spin_for(2.0)
    status = harness.status()
    assert status.write_errors == 0
    # The per-topic breakdown lists only topics that lost something, so nothing here.
    assert list(status.topic_losses) == []


def test_storage_options_reach_the_writer():
    """max_bagfile_duration is passed straight to rosbag2, so a one-second limit shows up as a
    split the recorder counts. Proves the pass-through rather than the parameter declaration."""
    harness, _, cleanup = start_recorder(("-p", "max_bagfile_duration:=1"))
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
    harness, _, cleanup = start_recorder((
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
        start_recorder(("-p", "max_cache_duration:=2"))


def test_unknown_storage_preset_is_refused_at_startup():
    """A typo in the preset is a configuration error, and it should fail before a bag exists
    rather than record on defaults while claiming otherwise."""
    with pytest.raises(AssertionError, match="exited early"):
        start_recorder(("-p", "storage_preset_profile:=fastwrit"))


def test_an_impossible_free_space_percentage_is_refused_at_startup():
    """A percentage over 100 can never be satisfied, so it would stop every recording the instant
    it started. Refusing it at startup says so instead of producing empty bags."""
    with pytest.raises(AssertionError, match="exited early"):
        start_recorder(("-p", "min_free_space_percent:=150"))


def test_a_negative_bag_size_limit_is_refused_at_startup():
    """Rejected like the other byte counts rather than wrapping to an enormous unsigned limit."""
    with pytest.raises(AssertionError, match="exited early"):
        start_recorder(("-p", "max_bag_size:=-1"))
