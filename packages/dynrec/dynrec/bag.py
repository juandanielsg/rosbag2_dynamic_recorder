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

"""Reading a recorded bag back, and answering what each channel really did.

`mcap info` and `ros2 bag info` compute a channel's rate as its count over the whole bag, so a
topic that published at 20 Hz for a quarter of the bag reads as 5 Hz. Dividing by the channel's
own first-to-last span fixes a channel that stopped, but not one with a hole in the middle; only
the recorder's own events locate those holes.

When the bag does not carry what is needed, the answer is unknown rather than a plausible number:
`ChannelStats.rate` is None, and `basis` says which evidence was available.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from dynrec.results import EVENT_STREAMS, Event
from dynrec.schedule import NANOSECONDS_PER_SECOND

#: How long a simultaneous, unexplained hole must be to be reported. Below this, scheduling
#: jitter across a handful of topics would raise a permanent false alarm.
UNEXPLAINED_GAP_SECONDS = 1.0


@dataclass(frozen=True)
class ChannelStats:
    """What one topic in the bag actually did."""

    topic: str
    message_type: str
    count: int
    #: Epoch seconds of the first and last message on this channel, on the recorder's clock.
    first: float
    last: float
    #: Seconds this channel was subscribed and the recorder was not paused. None when the bag
    #: does not carry the events needed to work it out.
    recorded_seconds: Optional[float]
    #: The same, trimmed to the span in which messages actually arrived. A large difference from
    #: `recorded_seconds` is itself a finding: a sensor that went quiet while still subscribed.
    active_seconds: Optional[float]
    #: Messages per second over `active_seconds`: the rate the publisher actually ran at. None
    #: when it cannot be determined.
    rate: Optional[float]
    #: Count over the whole bag duration, as `mcap info` and `ros2 bag info` report it.
    averaged_rate: float
    #: ``events`` when subscription events located the windows and the rate is trustworthy;
    #: ``span`` when no event names this topic and its own first-to-last span was used, which is
    #: wrong for a channel with an interior hole; ``unknown`` when there was no evidence at all.
    basis: str
    #: The whole bag's duration, so `sparse` has something to compare against.
    bag_seconds: float = 0.0

    @property
    def sparse(self):
        """True when this channel was not live for the whole bag."""
        return self.recorded_seconds is not None and self.recorded_seconds < self.bag_seconds


@dataclass(frozen=True)
class BagSummary:
    """A whole bag, described in terms of what was being recorded when."""

    uri: str
    start: float
    end: float
    channels: List[ChannelStats] = field(default_factory=list)
    #: Windows during which the recorder was paused, from the PauseEvents in the bag.
    pause_windows: List[Tuple[float, float]] = field(default_factory=list)
    #: Every recorder event in the bag, oldest first.
    events: List[Event] = field(default_factory=list)
    #: Holes shared by every live channel that no PauseEvent accounts for: a crash, a stall, or a
    #: pause recorded with `record_pause_events:=false`.
    unexplained_gaps: List[Tuple[float, float]] = field(default_factory=list)
    #: The LowDiskEvent if the recorder stopped itself for free space, else None. None also when
    #: the event was not recorded (`record_low_disk_events:=false`).
    low_disk_stop: Optional[Event] = None
    #: The BagSizeLimitEvent if the recorder stopped itself at `max_bag_size`, else None. Same
    #: caveat (`record_bag_size_limit_events:=false`).
    bag_size_limit_stop: Optional[Event] = None
    #: Things the reader could not establish, worded for a reader of the report.
    warnings: List[str] = field(default_factory=list)

    @property
    def duration(self):
        return self.end - self.start

    def as_dict(self):
        """The report as plain data, times relative to the bag's start, unknown values as None."""
        def relative(windows):
            return [{'start': a - self.start, 'end': b - self.start} for a, b in windows]

        return {
            'uri': self.uri,
            'duration_seconds': self.duration,
            'channels': [
                {
                    'topic': channel.topic,
                    'type': channel.message_type,
                    'count': channel.count,
                    'recorded_seconds': channel.recorded_seconds,
                    'active_seconds': channel.active_seconds,
                    'rate': channel.rate,
                    'averaged_rate': channel.averaged_rate,
                    'basis': channel.basis,
                }
                for channel in self.channels
            ],
            'pause_windows': relative(self.pause_windows),
            'unexplained_gaps': relative(self.unexplained_gaps),
            'stopped_for_low_disk': self.low_disk_stop is not None,
            'stopped_for_max_bag_size': self.bag_size_limit_stop is not None,
            'warnings': list(self.warnings),
        }


# -- interval arithmetic, free of ROS so it can be tested without a bag -------------------------


def merge_windows(windows):
    """Collapse overlapping or touching intervals into the smallest equivalent set, dropping
    empty ones."""
    merged = []
    for start, end in sorted((a, b) for a, b in windows if b > a):
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
        else:
            merged.append((start, end))
    return merged


def intersect_windows(first, second):
    """The intervals covered by both sets."""
    return merge_windows(
        (max(a[0], b[0]), min(a[1], b[1])) for a in first for b in second)


def overlap(first, second):
    """Seconds two intervals share."""
    return max(0.0, min(first[1], second[1]) - max(first[0], second[0]))


def _event_windows(events, matches, opening, closing, end):
    """Intervals from each `opening` event to the next `closing` one, among events `matches`
    accepts. One still open at the end is closed at `end`. A repeated opening keeps the earlier
    stamp rather than restarting the window."""
    windows = []
    opened = None
    for event in sorted(events, key=lambda e: e.stamp):
        if not matches(event):
            continue
        if event.action == opening and opened is None:
            opened = event.stamp
        elif event.action == closing and opened is not None:
            windows.append((opened, event.stamp))
            opened = None
    if opened is not None:
        windows.append((opened, end))
    return merge_windows(windows)


def subscribed_windows(events, topic, bag_end):
    """When `topic` was subscribed, from the subscription events in the bag. Still subscribed at
    the end means subscribed until `bag_end`."""
    return _event_windows(
        events, lambda e: e.kind == 'subscription' and e.topic == topic,
        'subscribed', 'unsubscribed', bag_end)


def pause_windows(events, bag_end=None):
    """When the recorder was paused, from the pause events in the bag. Still paused at the end
    means paused until `bag_end`, or, without one, until the last event."""
    if bag_end is None:
        bag_end = max((e.stamp for e in events), default=0.0)
    return _event_windows(events, lambda e: e.kind == 'pause', 'paused', 'resumed', bag_end)


def live_seconds(windows, pauses):
    """Seconds covered by `windows` with every paused interval taken out."""
    pauses = merge_windows(pauses)
    total = sum(
        (end - start) - sum(overlap((start, end), p) for p in pauses)
        for start, end in merge_windows(windows))
    return max(0.0, total)


def rate_over(count, seconds):
    """Messages per second, or None when there is no interval to divide by."""
    if seconds is None or seconds <= 0.0:
        return None
    return count / seconds


def unexplained_gaps(channel_gaps, pauses, threshold=UNEXPLAINED_GAP_SECONDS):
    """Holes that every channel shares and no pause accounts for.

    `channel_gaps` is one list of (start, end) per channel. A hole in one channel is what
    unsubscribing looks like; a hole in every channel at once is what a pause looks like, so one
    with no PauseEvent behind it is worth reporting.
    """
    if not channel_gaps:
        return []
    shared = merge_windows(channel_gaps[0])
    for gaps in channel_gaps[1:]:
        shared = intersect_windows(shared, gaps)
        if not shared:
            return []
    pauses = merge_windows(pauses)
    return [
        window for window in shared
        if window[1] - window[0] >= threshold
        and not any(overlap(window, p) > (window[1] - window[0]) / 2 for p in pauses)
    ]


def gaps_in(stamps, threshold):
    """Intervals between consecutive messages at least `threshold` long."""
    ordered = sorted(stamps)
    return [(a, b) for a, b in zip(ordered, ordered[1:]) if b - a >= threshold]


# -- reading the bag ----------------------------------------------------------------------------


def describe(uri, storage_id=''):
    """Read a bag and report what each channel really did.

    Leave `storage_id` empty to let rosbag2 detect it from the bag's metadata.

    Needs `rosbag2_py`, imported here so that everything above stays importable without ROS.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    # The recorder's event channels, with the Event constructor for each. Every other is data.
    event_parsers = {
        name: parse
        for name in types
        for suffix, (_, parse) in EVENT_STREAMS.items() if name.endswith(suffix)
    }

    # The receive stamp throughout: it is the bag's log time, the clock `ros2 bag info` measures
    # duration on, and under use_sim_time the only one on the simulation's clock. read_next() is
    # deprecated where read_next_ext() exists.
    extended = hasattr(reader, 'read_next_ext')
    stamps = {name: [] for name in types}
    events = []
    while reader.has_next():
        topic, data, nanoseconds = reader.read_next_ext()[:3] if extended else reader.read_next()
        stamps.setdefault(topic, []).append(nanoseconds / NANOSECONDS_PER_SECOND)
        parse = event_parsers.get(topic)
        if parse:
            events.append(parse(deserialize_message(data, get_message(types[topic]))))

    populated = {name: values for name, values in stamps.items() if values}
    if not populated:
        return BagSummary(uri=str(uri), start=0.0, end=0.0,
                          warnings=['the bag contains no messages'])

    start = min(min(values) for values in populated.values())
    end = max(max(values) for values in populated.values())
    duration = end - start
    events.sort(key=lambda event: event.stamp)
    pauses = pause_windows(events, bag_end=end)

    channels = []
    for topic in sorted(name for name in populated if name not in event_parsers):
        values = sorted(populated[topic])
        windows = subscribed_windows(events, topic, end)
        if windows:
            basis = 'events'
            recorded = live_seconds(windows, pauses)
            active = live_seconds(intersect_windows(windows, [(values[0], values[-1])]), pauses)
        elif len(values) > 1:
            basis = 'span'
            recorded = active = live_seconds([(values[0], values[-1])], pauses)
        else:
            basis = 'unknown'
            recorded = active = None
        channels.append(ChannelStats(
            topic=topic,
            message_type=types.get(topic, ''),
            count=len(values),
            first=values[0],
            last=values[-1],
            recorded_seconds=recorded,
            active_seconds=active,
            rate=rate_over(len(values), active),
            averaged_rate=len(values) / duration if duration > 0 else 0.0,
            basis=basis,
            bag_seconds=duration,
        ))

    # Only channels with gaps to compare can share one.
    unexplained = unexplained_gaps(
        [gaps_in(populated[c.topic], UNEXPLAINED_GAP_SECONDS) for c in channels if c.count > 1],
        pauses)

    def last_event(kind):
        return next((event for event in reversed(events) if event.kind == kind), None)

    low_disk_stop = last_event('low_disk')
    bag_size_limit_stop = last_event('bag_size_limit')
    return BagSummary(
        uri=str(uri), start=start, end=end, channels=channels, pause_windows=pauses,
        events=events, unexplained_gaps=unexplained, low_disk_stop=low_disk_stop,
        bag_size_limit_stop=bag_size_limit_stop,
        warnings=_warnings(events, start, unexplained, low_disk_stop, bag_size_limit_stop))


def _warnings(events, start, unexplained, low_disk_stop, bag_size_limit_stop):
    """What a reader of the report needs to know the bag could not establish, or why it ended."""
    warnings = []
    if not any(event.kind == 'subscription' for event in events):
        warnings.append(
            'this bag carries no subscription events, so each channel was measured over its own '
            'first-to-last span. That is right for a channel that stopped and wrong for one with '
            'a hole in the middle. Was it recorded with record_subscription_events:=false?')
    if not any(event.kind == 'pause' for event in events):
        warnings.append(
            'this bag carries no pause events. Either the recorder was never paused, or it ran '
            'with record_pause_events:=false -- the bag cannot tell those apart, because the '
            'event channel is only created when the first event is written.')
    for gap_start, gap_end in unexplained:
        warnings.append(
            'every live channel stops together for {:.2f}s at {:.1f}s into the bag and no pause '
            'event explains it. A crash, a stall, or a pause recorded with '
            'record_pause_events:=false all look like this.'.format(
                gap_end - gap_start, gap_start - start))
    if low_disk_stop is not None:
        warnings.append(
            'this recording ended because free space on the bag filesystem fell below the '
            'configured minimum ({} bytes available of {}). The recorder stopped itself to leave '
            'the disk usable, so the end of the bag is deliberate rather than a crash or a power '
            'loss.'.format(low_disk_stop.free_space_bytes, low_disk_stop.total_space_bytes))
    if bag_size_limit_stop is not None:
        warnings.append(
            'this recording ended because the bag reached its configured size limit ({} bytes on '
            'disk, limit {}). The recorder stopped itself, so the end of the bag is deliberate '
            'rather than a crash or a power loss.'.format(
                bag_size_limit_stop.bag_size_bytes, bag_size_limit_stop.max_bag_size))
    return warnings
