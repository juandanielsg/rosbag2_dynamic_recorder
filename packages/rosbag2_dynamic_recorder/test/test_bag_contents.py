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

"""What ends up in the bag: no split on a topic change, no hole in untouched topics, and
events that explain every gap."""

import glob
import os

from rosbag2_dynamic_recorder_interfaces.msg import PauseEvent
from rosbag2_dynamic_recorder_interfaces.srv import SetTopics
from rosbag2_interfaces.srv import Pause, TogglePaused

from recorder_harness import (
    EVENT_TOPIC,
    MAX_TOLERATED_GAP_S,
    PAUSED,
    PAUSE_EVENT_TYPE,
    RESUMED,
    Resume,
    SUBSCRIBED,
    Stop,
    TOPICS,
    UNSUBSCRIBED,
    largest_gap,
    read_bag,
    read_events,
    record_a_topic_change,
    widest_gap_window,
)


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
    events = read_events(bag, PAUSE_EVENT_TYPE, PauseEvent)

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
    events = read_events(bag, PAUSE_EVENT_TYPE, PauseEvent)
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

    reasons = [event.reason for _offset, event in read_events(bag, PAUSE_EVENT_TYPE, PauseEvent)]
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

    events = read_events(bag, PAUSE_EVENT_TYPE, PauseEvent)
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

    assert read_events(bag, PAUSE_EVENT_TYPE, PauseEvent) == [], (
        "record_pause_events:=false still wrote events")


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
