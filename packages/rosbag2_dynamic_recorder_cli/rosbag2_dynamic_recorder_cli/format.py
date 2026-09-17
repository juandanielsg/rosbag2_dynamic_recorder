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

"""Turning recorder state into text, and defining the scheduling arguments.

Pure functions with no ROS dependency beyond the message fields they read, so the rules below can
be tested directly. The rule worth protecting is the same one the browser UI encodes: a number the
recorder cannot vouch for is reported as unknown, never as a convenient zero.

The time dialect and the mode mapping are dynrec.schedule's -- the verbs hand `--at` straight to
`dynrec.Recorder`, so there is no second copy here to drift from it.
"""

from dynrec.schedule import TIME_MODES

#: Width of the label column in `status`: the longest label, 'lost (transport):', plus a gap.
LABEL_WIDTH = 14


def add_schedule_arguments(parser, noun):
    """The --at/--mode/--topic trio shared by `resume` and `split`."""
    parser.add_argument(
        '--at', metavar='TIME', default=None,
        help='Schedule the {} for a future time instead of doing it now. Accepts +30s, 14:05, '
             '2026-09-03T14:05, or epoch seconds'.format(noun))
    parser.add_argument(
        '--mode', choices=sorted(TIME_MODES), default='node',
        help='Clock the scheduled time is compared against (default: %(default)s). '
             'node fires on a timer and works on a robot that has gone quiet; publish and '
             'receive are evaluated as messages arrive and cannot fire without traffic')
    parser.add_argument(
        '--topic', metavar='TOPIC', default='',
        help='For publish and receive mode, evaluate against this topic only. It must be one '
             'the recorder is subscribed to. Default: any recorded topic')


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
    return lines
