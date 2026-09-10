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

"""Turning recorder state into text, and command-line time into a ROS timestamp.

Pure functions with no ROS dependency beyond the message fields they read, so the rules below can
be tested directly. The rule worth protecting is the same one the browser UI encodes: a number the
recorder cannot vouch for is reported as unknown, never as a convenient zero.
"""

import re
import time
from datetime import datetime, timedelta

from rosbag2_dynamic_recorder_cli.api import RecorderError

#: Mode names accepted by --mode, mapping to the constants shared by Resume and SplitBagfile.
TIME_MODES = {
    'node': 0,
    'publish': 1,
    'receive': 2,
}

_RELATIVE = re.compile(r'^\+(\d+(?:\.\d+)?)([smh]?)$')
_UNIT_SECONDS = {'': 1.0, 's': 1.0, 'm': 60.0, 'h': 3600.0}


def parse_time(value, now=None):
    """Parse a --at argument into (sec, nanosec) for a builtin_interfaces/Time.

    Three forms, in the order they are worth reaching for over SSH:

      +30s, +5m, +1h   relative to now
      14:05            today at that local wall-clock time, tomorrow if it has already passed
      2026-09-03T14:05 an ISO 8601 instant, local time unless it carries an offset
      1756900000.5     raw epoch seconds, for scripts

    The relative and wall-clock forms are resolved against *this* machine's clock, while the
    recorder compares against its own node clock. On one machine, or a clock-synced fleet, those
    agree; across a robot whose clock has drifted they do not, which is why the absolute forms
    exist.
    """
    if now is None:
        now = time.time()
    text = value.strip()

    relative = _RELATIVE.match(text)
    if relative:
        amount, unit = relative.groups()
        return _split(now + float(amount) * _UNIT_SECONDS[unit])

    # A bare number is epoch seconds. Checked before the date parsing so "1756900000" is not read
    # as a year.
    try:
        return _split(float(text))
    except ValueError:
        pass

    if re.match(r'^\d{1,2}:\d{2}(:\d{2})?$', text):
        return _split(_next_wall_clock(text, now))

    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        raise RecorderError(
            "cannot read '{}' as a time. Try +30s, 14:05, 2026-09-03T14:05, or epoch "
            'seconds.'.format(value))
    if parsed.tzinfo is None:
        parsed = parsed.astimezone()
    return _split(parsed.timestamp())


def _next_wall_clock(text, now):
    """Today at `text`, or tomorrow if that moment has already gone by."""
    fields = [int(part) for part in text.split(':')]
    while len(fields) < 3:
        fields.append(0)
    today = datetime.fromtimestamp(now).replace(
        hour=fields[0], minute=fields[1], second=fields[2], microsecond=0)
    if today.timestamp() > now:
        return today.timestamp()
    # A day added in calendar terms rather than as 86400 seconds, so "05:00" the night before a
    # daylight-saving change still means five in the morning.
    return (today + timedelta(days=1)).timestamp()


def _split(seconds):
    whole = int(seconds)
    nanoseconds = int(round((seconds - whole) * 1e9))
    # A fraction within half a nanosecond of the next second rounds to 1e9, which is not a legal
    # nanosec field. Rare, but it would surface as an opaque message-assignment error.
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return whole, nanoseconds


def parse_mode(value):
    """Map a --mode name to its numeric constant."""
    try:
        return TIME_MODES[value]
    except KeyError:
        raise RecorderError(
            "unknown mode '{}'. Choose one of: {}".format(value, ', '.join(sorted(TIME_MODES))))


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


def apply_schedule(request, args, time_field, mode_field):
    """Fill a scheduled request's time, mode and tracking topic from the parsed arguments.

    Leaving the timestamp at zero is how these services are told to act immediately, and the
    recorder reads a zero stamp as absent rather than as time zero -- so the untouched request is
    already the "do it now" request.
    """
    request.tracking_topic_name = args.topic
    setattr(request, mode_field, parse_mode(args.mode))
    if args.at is None:
        return None
    seconds, nanoseconds = parse_time(args.at)
    stamp = getattr(request, time_field)
    stamp.sec = seconds
    stamp.nanosec = nanoseconds
    return seconds + nanoseconds / 1e9


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
    for unit in ('B', 'KiB', 'MiB', 'GiB', 'TiB'):
        if size < 1024.0 or unit == 'TiB':
            return '{:.0f} {}'.format(size, unit) if unit == 'B' else '{:.1f} {}'.format(
                size, unit)
        size /= 1024.0
    return '{:.1f} TiB'.format(size)


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
        return 'stopped'
    word = 'paused' if status.paused else 'recording'
    if status.snapshot_mode:
        word += ' (snapshot mode: nothing is written until ~/snapshot)'
    return word


def status_dict(status):
    """The status as plain data, for --json.

    `messages_missed` is None rather than 0 when the middleware supplies no publication sequence
    numbers, because the recorder genuinely cannot tell. A consumer that wants a number can then
    decide what to do about not having one, instead of being handed a zero that means "no idea".
    """
    sequence_ok = bool(status.sequence_numbers_available)
    return {
        'uri': status.uri,
        'storage_id': status.storage_id,
        'recording': bool(status.recording),
        'paused': bool(status.paused),
        'snapshot_mode': bool(status.snapshot_mode),
        'elapsed_seconds': status.elapsed_seconds,
        'subscribed_topics': list(status.subscribed_topics),
        'active_profile': status.active_profile,
        'messages_written': status.messages_written,
        'messages_missed': status.messages_missed if sequence_ok else None,
        'messages_lost_reported': status.messages_lost,
        'write_errors': status.write_errors,
        'bag_splits': status.bag_splits,
        'bag_size_bytes': status.bag_size_bytes,
    }


def format_status(status, recorder_name):
    """The human-readable status block, as a list of lines."""
    rows = [
        ('recorder', recorder_name),
        ('state', state_word(status)),
        ('bag', '{} [{}]'.format(status.uri, status.storage_id)),
        ('elapsed', format_duration(status.elapsed_seconds)),
        ('profile', status.active_profile or '(none matches the current selection)'),
        ('topics', '{} subscribed'.format(len(status.subscribed_topics))),
    ]
    lines = ['{:<14}{}'.format(label + ':', value) for label, value in rows]
    lines.extend('{:<14}{}'.format('', topic) for topic in status.subscribed_topics)

    if status.sequence_numbers_available:
        missed = str(status.messages_missed)
    else:
        missed = 'unknown (this middleware supplies no publication sequence numbers)'

    counters = [
        ('written', str(status.messages_written)),
        ('missed', missed),
        ('lost', '{} (as reported by the transport, which can miss losses)'.format(
            status.messages_lost)),
        ('write errors', str(status.write_errors)),
        ('splits', str(status.bag_splits)),
        ('on disk', '{} (flushed; the writer caches, so this lags what is captured)'.format(
            format_bytes(status.bag_size_bytes))),
    ]
    lines.extend('{:<14}{}'.format(label + ':', value) for label, value in counters)
    return lines
