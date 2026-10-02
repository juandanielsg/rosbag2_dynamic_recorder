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

"""Turning recorder state and bag reports into text for a terminal.

Pure functions with no ROS dependency beyond the fields they read, so they can be tested directly.
As in the browser UI, a number the recorder cannot vouch for reads as unknown, never as zero.
"""

import time
from datetime import datetime

#: Width of the label column in `status`: the longest label, 'lost (transport):', plus a gap.
LABEL_WIDTH = 14


def format_scheduled(done, noun, at, mode=None):
    """What a verb that can be scheduled prints: `done` when it happened now, else when it will.

    `at` is the epoch time dynrec returned for a scheduled call, or None for an immediate one.
    """
    if at is None:
        return done
    text = '{} scheduled in {}'.format(noun, format_duration(at - time.time()))
    return text + (' ({} time)'.format(mode) if mode else '')


def format_duration(seconds):
    """Elapsed time, at the precision a person reading a terminal actually wants."""
    seconds = max(0.0, float(seconds))
    if seconds < 60.0:
        return '{:.1f}s'.format(seconds)
    whole = int(seconds)
    hours, remainder = divmod(whole, 3600)
    minutes, secs = divmod(remainder, 60)
    if hours:
        return '{}h {:02d}m {:02d}s'.format(hours, minutes, secs)
    return '{}m {:02d}s'.format(minutes, secs)


def format_bytes(count):
    """Binary-prefixed size. 0 stays "0 B" -- see the caveat where this is used."""
    size = float(count)
    unit = 'B'
    for larger in ('KiB', 'MiB', 'GiB', 'TiB'):
        if size < 1024.0:
            break
        size, unit = size / 1024.0, larger
    return '{:.0f} {}'.format(size, unit) if unit == 'B' else '{:.1f} {}'.format(size, unit)


def format_topic_group(label, topics):
    """A named group of topics, or nothing at all when the group is empty.

    Printing empty groups would make every topic change three lines long regardless of what
    happened, which buries the one line that did.
    """
    if not topics:
        return []
    return ['{} ({}):'.format(label, len(topics))] + ['  ' + topic for topic in topics]


def state_word(status):
    """One word for what the recorder is doing, plus any qualifier worth seeing."""
    if not status.recording:
        if status.waiting_for_clock:
            return 'waiting for /clock (use_sim_time is set; no bag is open until it arrives)'
        # A stop the recorder chose is the one a reader most needs explained.
        if status.stopped_for_low_disk:
            return 'stopped (free space on the bag filesystem fell below the minimum)'
        if status.stopped_for_max_bag_size:
            return 'stopped (the bag reached max_bag_size)'
        return 'stopped'
    word = 'paused' if status.paused else 'recording'
    if status.snapshot_mode:
        word += ' (snapshot mode: nothing is written until ~/snapshot)'
    return word


def free_space_text(status):
    """Free space on the bag filesystem and the floor the guard enforces, or unknown."""
    if not status.total_space_bytes:
        return 'unknown (the bag filesystem could not be read)'
    text = '{} of {}'.format(
        format_bytes(status.free_space_bytes), format_bytes(status.total_space_bytes))
    floors = []
    if status.min_free_space:
        floors.append(format_bytes(status.min_free_space))
    if status.min_free_space_percent:
        floors.append('{:g}%'.format(status.min_free_space_percent))
    if floors:
        text += ' (recording stops below {})'.format(' or '.join(floors))
    return text


def format_clock_time(seconds, use_sim_time):
    """A time on the recorder's clock. Under sim time that is the simulation's, not a date."""
    if use_sim_time:
        return 't={:.1f}s on the simulation clock'.format(seconds)
    return datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')


def format_schedule(scheduled, use_sim_time):
    """One pending :class:`dynrec.Scheduled`, saying which clock it waits on."""
    when = format_clock_time(scheduled.at, use_sim_time)
    if scheduled.mode == 'node':
        return '{} at {} (node time, fires on a timer)'.format(scheduled.action, when)
    return '{} at {} by {} time, on {} (waits for traffic)'.format(
        scheduled.action, when, scheduled.mode,
        scheduled.topic or 'any recorded topic')


def format_topic_loss(loss):
    """One :class:`dynrec.TopicLoss`, with missed unknown where the totals say so too."""
    missed = 'unknown' if loss.missed is None else str(loss.missed)
    return '{}: missed {}, lost {} in transport, {} in recorder'.format(
        loss.topic, missed, loss.lost_in_transport, loss.lost_in_recorder)


def _row(label, value):
    """One aligned line of the status block. An empty label continues the row above."""
    return '{:<{}}{}'.format(label + ':' if label else '', LABEL_WIDTH, value)


def format_status(status):
    """The human-readable status block for a :class:`dynrec.Status`, as a list of lines.

    `messages_missed` is None when the middleware supplies no publication sequence numbers,
    because the recorder genuinely cannot tell; that is rendered as unknown rather than as a zero
    that means "no idea".
    """
    rows = [
        ('recorder', status.recorder),
        ('state', state_word(status)),
        ('bag', '{} [{}]'.format(status.uri, status.storage_id)),
        ('elapsed', format_duration(status.elapsed_seconds)),
        ('profile', status.active_profile or '(none matches the current selection)'),
        ('topics', '{} subscribed'.format(len(status.subscribed_topics))),
    ]
    lines = [_row(label, value) for label, value in rows]
    lines.extend(_row('', topic) for topic in status.subscribed_topics)

    if status.messages_missed is None:
        missed = 'unknown (this middleware supplies no publication sequence numbers)'
    else:
        missed = str(status.messages_missed)

    counters = [
        ('written', str(status.messages_written)),
        ('missed', missed),
        ('lost (transport)', '{} (as reported by the transport, which can miss losses)'.format(
            status.messages_lost_in_transport)),
        ('lost (recorder)', '{} (dropped by the writer: cache full or write failed)'.format(
            status.messages_lost_in_recorder)),
        ('write errors', str(status.write_errors)),
        ('splits', str(status.bag_splits)),
        ('on disk', '{}{} (flushed; the writer caches, so this lags what is captured)'.format(
            format_bytes(status.bag_size_bytes),
            ' of {} allowed'.format(format_bytes(status.max_bag_size))
            if status.max_bag_size else '')),
        ('free space', free_space_text(status)),
    ]
    lines.extend(_row(label, value) for label, value in counters)
    # Only when there is something to say, as with empty topic groups: a line per topic that lost
    # nothing, or a "scheduled: none", would bury the lines that matter.
    for index, loss in enumerate(status.topic_losses):
        lines.append(_row('by topic' if index == 0 else '', format_topic_loss(loss)))
    for index, scheduled in enumerate(status.schedules):
        lines.append(_row('scheduled' if index == 0 else '',
                          format_schedule(scheduled, status.use_sim_time)))
    return lines


def format_bag_summary(summary):
    """A :class:`dynrec.BagSummary` as lines of text: one row per channel, then pauses and
    warnings."""
    if not summary.channels:
        return ['{}: nothing to report'.format(summary.uri)] + list(summary.warnings)

    def seconds(value):
        return '?' if value is None else '{:.1f}s'.format(value)

    row = '{:<34}{:>8}{:>10}{:>9}{:>11}{:>12}{}'
    lines = [
        '{}  {:.1f}s  {} messages on {} channels'.format(
            summary.uri, summary.duration,
            sum(channel.count for channel in summary.channels), len(summary.channels)),
        '',
        row.format('topic', 'msgs', 'recorded', 'active', 'rate', 'averaged', ''),
    ]
    for channel in summary.channels:
        lines.append(row.format(
            channel.topic, channel.count, seconds(channel.recorded_seconds),
            seconds(channel.active_seconds),
            'unknown' if channel.rate is None else '{:.2f} Hz'.format(channel.rate),
            '{:.2f} Hz'.format(channel.averaged_rate),
            '' if channel.basis == 'events' else '  ({})'.format(channel.basis)))
    lines += ['',
              'recorded is how long the channel was subscribed and not paused; active trims that',
              'to when messages were really arriving, and rate is counted over active.',
              'averaged is count over the whole bag -- what ros2 bag info and mcap info report.']
    if summary.pause_windows:
        lines += ['', 'paused:']
        for begin, end in summary.pause_windows:
            lines.append('  {:.1f}s to {:.1f}s  ({:.2f}s)'.format(
                begin - summary.start, end - summary.start, end - begin))
    if summary.warnings:
        lines.append('')
        lines += ['warning: ' + warning for warning in summary.warnings]
    return lines
