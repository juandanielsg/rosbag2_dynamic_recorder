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

"""Command-line-shaped times, turned into what the scheduled services take.

Deliberately the same dialect `ros2 dynrec --at` accepts, so a value that worked in a shell
command keeps working when the shell command becomes a script. Standard library only.
"""

import re
import time
from datetime import datetime, timedelta

from dynrec.errors import InvalidRequest

#: Mode names accepted by `mode=`, mapping to the constants shared by Resume and SplitBagfile.
#:
#: `node` fires on a timer and therefore works on a robot that has gone quiet; `publish` and
#: `receive` are evaluated as messages arrive and cannot fire without traffic. That difference is
#: the whole reason the choice exists, so it is worth knowing before picking one.
TIME_MODES = {
    'node': 0,
    'publish': 1,
    'receive': 2,
}

_RELATIVE = re.compile(r'^\+(\d+(?:\.\d+)?)([smh]?)$')
_UNIT_SECONDS = {'': 1.0, 's': 1.0, 'm': 60.0, 'h': 3600.0}


def parse_time(value, now=None):
    """Parse an `at=` argument into `(sec, nanosec)` for a builtin_interfaces/Time.

    Accepts, besides a `datetime` or a number of epoch seconds:

      +30s, +5m, +1h   relative to now
      14:05            today at that local wall-clock time, tomorrow if it has already passed
      2026-09-03T14:05 an ISO 8601 instant, local time unless it carries an offset
      1756900000.5     raw epoch seconds

    The relative and wall-clock forms are resolved against *this* machine's clock, while the
    recorder compares against its own node clock. On one machine, or a clock-synced fleet, those
    agree; across a robot whose clock has drifted they do not, which is why the absolute forms
    exist and why a script running off-robot should prefer them.
    """
    if now is None:
        now = time.time()
    if isinstance(value, datetime):
        parsed = value if value.tzinfo is not None else value.astimezone()
        return _split(parsed.timestamp())
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return _split(float(value))
    if not isinstance(value, str):
        raise InvalidRequest(
            'cannot read {!r} as a time. Pass a string, a datetime, or epoch '
            'seconds.'.format(value))
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
        raise InvalidRequest(
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
    """Map a mode name to its numeric constant, or pass a constant through."""
    if isinstance(value, int) and not isinstance(value, bool):
        if value not in TIME_MODES.values():
            raise InvalidRequest('unknown time mode {}'.format(value))
        return value
    try:
        return TIME_MODES[value]
    except KeyError:
        raise InvalidRequest(
            "unknown mode '{}'. Choose one of: {}".format(value, ', '.join(sorted(TIME_MODES))))


def apply_schedule(request, at, mode, tracking_topic, time_field, mode_field):
    """Fill a scheduled request's time, mode and tracking topic. Returns the epoch time, or None.

    Leaving the timestamp at zero is how these services are told to act immediately, and the
    recorder reads a zero stamp as absent rather than as time zero -- so the untouched request is
    already the "do it now" request.
    """
    request.tracking_topic_name = tracking_topic or ''
    setattr(request, mode_field, parse_mode(mode))
    if at is None:
        return None
    seconds, nanoseconds = parse_time(at)
    stamp = getattr(request, time_field)
    stamp.sec = seconds
    stamp.nanosec = nanoseconds
    return seconds + nanoseconds / 1e9
