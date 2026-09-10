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

This exists because of a number that is wrong in every tool that reports it. `mcap info` and
`ros2 bag info` compute a channel's rate as its message count over the *whole bag duration*, so a
topic that published at 20 Hz for eleven seconds and was then unsubscribed is reported at 5.95 Hz.
Nothing is corrupted; the summary line simply describes a healthy sensor as a slow one, which is
precisely the misreading this project exists to prevent.

The obvious fix -- divide by the channel's own first-to-last span instead -- only half works, and
the half it misses is the interesting one:

    topic      msgs   whole bag    own span   subscribed
    alpha       590     16.12 Hz    16.12 Hz     19.99 Hz
    beta        218      5.95 Hz    19.87 Hz     19.78 Hz
    gamma       365      9.97 Hz    14.46 Hz     20.07 Hz
    delta        80      2.19 Hz    20.25 Hz     20.06 Hz

All four published at 20 Hz. Span arithmetic fixes the channels that *stopped* (beta, delta) and
cannot fix the ones with a hole in the middle (alpha, whose span is the whole bag; gamma, whose
span still contains the pause). Only the recorder's own events locate those holes -- which is why
this lives here rather than as a patch to somebody else's summary line.

The rule this module holds to is the project's: when the bag does not carry what is needed, the
answer is unknown rather than a plausible number. `ChannelStats.rate` is None in that case, and
`basis` says which evidence was available.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

#: How much of a simultaneous, unexplained hole is worth mentioning. Below this, ordinary
#: scheduling jitter across a handful of topics would raise a permanent false alarm.
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
    #: does not carry the events needed to work it out. This is how long it was *being recorded*.
    recorded_seconds: Optional[float]
    #: The same, further trimmed to the span in which messages actually arrived. Shorter than
    #: `recorded_seconds` by the subscription warm-up, and by any silence before the bag closed.
    #: A large gap between the two is itself a finding: a sensor that went quiet while still
    #: subscribed looks exactly like that.
    active_seconds: Optional[float]
    #: Messages per second over `active_seconds` -- the rate the publisher was actually running
    #: at, rather than one diluted by the moments around it. None when it cannot be determined.
    rate: Optional[float]
    #: Count over the whole bag duration -- what `mcap info` and `ros2 bag info` report. Kept so
    #: the two can be compared, since the gap between them is the entire point.
    averaged_rate: float
    #: Which evidence was available. ``events`` means subscription events located the windows and
    #: the rate is trustworthy. ``span`` means no event names this topic, so its own first-to-last
    #: span was used instead -- right for a channel that merely stopped, wrong for one with an
    #: interior hole. ``unknown`` means there was not enough evidence for any answer.
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
    #: Every SubscriptionChangeEvent and PauseEvent, oldest first, as `dynrec.results.Event`.
    events: List[object] = field(default_factory=list)
    #: Simultaneous holes across every live channel that no PauseEvent accounts for. A crash, a
    #: stall, or a pause recorded with `record_pause_events:=false` all look like this.
    unexplained_gaps: List[Tuple[float, float]] = field(default_factory=list)
    #: Things the reader could not establish, in the words a reader of the report needs.
    warnings: List[str] = field(default_factory=list)

    @property
    def duration(self):
        return self.end - self.start


# -- the judgement, kept free of ROS so it can be tested without a bag -------------------------


def merge_windows(windows):
    """Collapse overlapping or touching intervals into the smallest equivalent set."""
    ordered = sorted((a, b) for a, b in windows if b > a)
    merged = []
    for start, end in ordered:
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
        else:
            merged.append((start, end))
    return [tuple(window) for window in merged]


def overlap(first, second):
    """Seconds two intervals share."""
    return max(0.0, min(first[1], second[1]) - max(first[0], second[0]))


def subscribed_windows(events, topic, bag_end):
    """When `topic` was subscribed, from the subscription events in the bag.

    A window left open at the end of the bag is closed at the bag's end rather than dropped: the
    recorder was still recording that topic when the bag stopped, and treating that as "never
    closed, so unknown" would discard the most ordinary case there is.
    """
    windows = []
    opened = None
    for event in sorted(events, key=lambda e: e.stamp):
        if event.kind != 'subscription' or event.topic != topic:
            continue
        if event.action == 'subscribed':
            # Two SUBSCRIBED in a row should not happen; if it does, keep the earlier one rather
            # than silently restarting the window and under-reporting the time recorded.
            if opened is None:
                opened = event.stamp
        elif event.action == 'unsubscribed' and opened is not None:
            windows.append((opened, event.stamp))
            opened = None
    if opened is not None:
        windows.append((opened, bag_end))
    return merge_windows(windows)


def pause_windows(events, bag_end=None):
    """When the recorder was paused, from the pause events in the bag.

    A PAUSED still open at the end is closed at `bag_end`: the recording ended while paused, and
    everything after that instant was genuinely not being written. Without a `bag_end` the last
    event stands in for it, which in practice is the PAUSED event itself -- a paused recorder
    writes nothing else -- so the window is empty and nothing is wrongly deducted.
    """
    windows = []
    opened = None
    stamps = [e.stamp for e in events] or [0.0]
    for event in sorted(events, key=lambda e: e.stamp):
        if event.kind != 'pause':
            continue
        if event.action == 'paused':
            if opened is None:
                opened = event.stamp
        elif event.action == 'resumed' and opened is not None:
            windows.append((opened, event.stamp))
            opened = None
    if opened is not None:
        windows.append((opened, bag_end if bag_end is not None else max(stamps)))
    return merge_windows(windows)


def clip_windows(windows, bounds):
    """Trim windows to `bounds`, dropping any part outside it.

    Used to cut a subscribed window down to when the channel was actually delivering. Subscribing
    is not instantaneous -- the first message cannot arrive until the topic's message definition
    has been resolved -- ~0.4-0.6s per new topic by measurement, nearly all of it definition
    resolution (a nested `sensor_msgs/Imu` cost 300-470ms against 71ms for `std_msgs/String`) --
    and DDS matching adds more. Counting that silence as time the channel was producing messages
    would report a healthy 20 Hz sensor at 16 Hz purely because it was subscribed shortly before
    it started arriving.
    """
    clipped = []
    for start, end in windows:
        low = max(start, bounds[0])
        high = min(end, bounds[1])
        if high > low:
            clipped.append((low, high))
    return merge_windows(clipped)


def live_seconds(windows, pauses):
    """Seconds covered by `windows` with every paused interval taken out."""
    total = 0.0
    for window in merge_windows(windows):
        total += (window[1] - window[0]) - sum(overlap(window, p) for p in merge_windows(pauses))
    return max(0.0, total)


def rate_over(count, seconds):
    """Messages per second, or None when there is no interval to divide by.

    A single message spans no time, so its rate is genuinely unknown rather than infinite -- and
    reporting it as unknown is the same rule the rest of this project follows.
    """
    if seconds is None or seconds <= 0.0:
        return None
    return count / seconds


def unexplained_gaps(channel_gaps, pauses, threshold=UNEXPLAINED_GAP_SECONDS):
    """Holes that every live channel shares and no pause accounts for.

    `channel_gaps` is one list of (start, end) per channel. A hole in a single channel is ordinary
    -- it is what unsubscribing looks like, and the subscription events explain it. A hole in
    *every* channel at once is the signature the recorder writes PauseEvents to explain, so one
    with no PauseEvent behind it is worth surfacing: a crash, a stall, or a pause recorded with
    `record_pause_events:=false` all look exactly like this.
    """
    if not channel_gaps:
        return []
    shared = merge_windows(channel_gaps[0])
    for gaps in channel_gaps[1:]:
        overlapping = []
        for candidate in merge_windows(gaps):
            for existing in shared:
                start = max(candidate[0], existing[0])
                end = min(candidate[1], existing[1])
                if end - start > 0:
                    overlapping.append((start, end))
        shared = merge_windows(overlapping)
        if not shared:
            return []
    merged_pauses = merge_windows(pauses)
    return [
        window for window in shared
        if window[1] - window[0] >= threshold
        and not any(overlap(window, p) > (window[1] - window[0]) / 2 for p in merged_pauses)
    ]


def gaps_in(stamps, threshold):
    """Intervals between consecutive messages longer than `threshold`."""
    ordered = sorted(stamps)
    return [
        (first, second) for first, second in zip(ordered, ordered[1:])
        if second - first >= threshold
    ]


# -- reading the bag ---------------------------------------------------------------------------


def describe(uri, storage_id=''):
    """Read a bag and report what each channel really did.

    `storage_id` is normally best left empty, which lets rosbag2 detect it from the bag's own
    metadata; pass one only when that fails.

    Needs `rosbag2_py`, which is imported here rather than at module scope so that everything
    above stays importable -- and testable -- without a ROS installation.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    from dynrec.results import Event

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    # read_next() is deprecated in favour of read_next_ext(), which returns the send timestamp as
    # well as the receive one. The receive timestamp is the one taken here either way: it is the
    # bag's log_time, and therefore the same clock `ros2 bag info` and `mcap info` measure a bag's
    # duration on -- which is what makes the honest and averaged rates below comparable.
    extended = hasattr(reader, 'read_next_ext')

    stamps = {name: [] for name in types}
    events = []
    while reader.has_next():
        if extended:
            topic, data, _send_ns, nanoseconds = reader.read_next_ext()
        else:
            topic, data, nanoseconds = reader.read_next()
        seconds = nanoseconds / 1e9
        stamps.setdefault(topic, []).append(seconds)
        if topic.endswith('/events/subscription_change'):
            events.append(Event.from_subscription_msg(
                deserialize_message(data, get_message(types[topic]))))
        elif topic.endswith('/events/pause'):
            events.append(Event.from_pause_msg(
                deserialize_message(data, get_message(types[topic]))))

    populated = {name: values for name, values in stamps.items() if values}
    if not populated:
        return BagSummary(uri=str(uri), start=0.0, end=0.0,
                          warnings=['the bag contains no messages'])

    start = min(min(values) for values in populated.values())
    end = max(max(values) for values in populated.values())
    duration = end - start
    events.sort(key=lambda event: event.stamp)

    pauses = pause_windows(events, bag_end=end)
    has_subscription_events = any(event.kind == 'subscription' for event in events)
    has_pause_events = any(event.kind == 'pause' for event in events)

    channels = []
    data_topics = sorted(
        name for name in populated
        if not (name.endswith('/events/subscription_change') or name.endswith('/events/pause')))
    for topic in data_topics:
        values = sorted(populated[topic])
        windows = subscribed_windows(events, topic, end)
        if windows:
            basis = 'events'
            recorded = live_seconds(windows, pauses)
            active = live_seconds(clip_windows(windows, (values[0], values[-1])), pauses)
        elif len(values) > 1:
            # No events name this topic, so the best available evidence is when its messages
            # actually arrived. Right for a channel that stopped, wrong for one with a hole.
            basis = 'span'
            recorded = live_seconds([(values[0], values[-1])], pauses)
            active = recorded
        else:
            basis = 'unknown'
            recorded = None
            active = None
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

    # A hole is only "shared" if it is shared by channels that were live to begin with, so a
    # channel that had already been unsubscribed cannot make every gap look simultaneous.
    live_gaps = [
        gaps_in(populated[channel.topic], UNEXPLAINED_GAP_SECONDS)
        for channel in channels if channel.count > 1
    ]
    unexplained = unexplained_gaps(live_gaps, pauses)

    warnings = []
    if not has_subscription_events:
        warnings.append(
            'this bag carries no subscription events, so each channel was measured over its own '
            'first-to-last span. That is right for a channel that stopped and wrong for one with '
            'a hole in the middle. Was it recorded with record_subscription_events:=false?')
    if not has_pause_events:
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

    return BagSummary(
        uri=str(uri), start=start, end=end, channels=channels, pause_windows=pauses,
        events=events, unexplained_gaps=unexplained, warnings=warnings)


def format_summary(summary):
    """The report as lines of text, for a terminal."""
    if not summary.channels:
        return ['{}: nothing to report'.format(summary.uri)] + list(summary.warnings)

    lines = [
        '{}  {:.1f}s  {} messages on {} channels'.format(
            summary.uri, summary.duration,
            sum(channel.count for channel in summary.channels), len(summary.channels)),
        '',
        '{:<34}{:>8}{:>10}{:>9}{:>11}{:>12}'.format(
            'topic', 'msgs', 'recorded', 'active', 'rate', 'averaged'),
    ]
    for channel in summary.channels:
        recorded = ('{:.1f}s'.format(channel.recorded_seconds)
                    if channel.recorded_seconds is not None else '?')
        active = ('{:.1f}s'.format(channel.active_seconds)
                  if channel.active_seconds is not None else '?')
        rate = '{:.2f} Hz'.format(channel.rate) if channel.rate is not None else 'unknown'
        marker = '' if channel.basis == 'events' else '  ({})'.format(channel.basis)
        lines.append('{:<34}{:>8}{:>10}{:>9}{:>11}{:>12}{}'.format(
            channel.topic, channel.count, recorded, active, rate,
            '{:.2f} Hz'.format(channel.averaged_rate), marker))

    lines += ['',
              'recorded is how long the channel was subscribed and not paused; active trims that',
              'to when messages were really arriving, and rate is counted over active.',
              'averaged is count over the whole bag -- what ros2 bag info and mcap info report.']

    if summary.pause_windows:
        lines.append('')
        lines.append('paused:')
        for begin, finish in summary.pause_windows:
            lines.append('  {:.1f}s to {:.1f}s  ({:.2f}s)'.format(
                begin - summary.start, finish - summary.start, finish - begin))

    if summary.warnings:
        lines.append('')
        for warning in summary.warnings:
            lines.append('warning: ' + warning)
    return lines


def summary_dict(summary):
    """The report as plain data, with unknown values left as null."""
    return {
        'uri': summary.uri,
        'duration_seconds': summary.duration,
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
            for channel in summary.channels
        ],
        'pause_windows': [
            {'start': begin - summary.start, 'end': finish - summary.start}
            for begin, finish in summary.pause_windows
        ],
        'unexplained_gaps': [
            {'start': begin - summary.start, 'end': finish - summary.start}
            for begin, finish in summary.unexplained_gaps
        ],
        'warnings': list(summary.warnings),
    }
