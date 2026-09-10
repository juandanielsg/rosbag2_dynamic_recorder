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

"""What `at=` accepts, and what an unscheduled request looks like on the wire.

The dialect is deliberately the one `ros2 dynrec --at` takes, so a value that worked in a shell
command keeps working when that command becomes a script. The case worth protecting is the last
one: an untouched timestamp is how these services are told to act immediately, so a request built
with `at=None` must leave the stamp at zero rather than filling in now.
"""

from datetime import datetime, timedelta, timezone
from types import SimpleNamespace

import pytest

from dynrec.errors import InvalidRequest
from dynrec.schedule import apply_schedule, parse_mode, parse_time

# 2026-09-03 12:00:00 UTC, as a fixed "now" so none of this depends on when it runs.
NOW = datetime(2026, 9, 3, 12, 0, 0, tzinfo=timezone.utc).timestamp()


def request():
    """A stand-in for Resume.Request: the fields apply_schedule touches, nothing else."""
    return SimpleNamespace(
        resume_time=SimpleNamespace(sec=0, nanosec=0),
        resume_mode=0,
        tracking_topic_name='')


def test_relative_times_in_each_unit():
    assert parse_time('+30s', NOW)[0] == int(NOW) + 30
    assert parse_time('+5m', NOW)[0] == int(NOW) + 300
    assert parse_time('+1h', NOW)[0] == int(NOW) + 3600
    # A bare +N is seconds, matching how anyone types it in a hurry.
    assert parse_time('+45', NOW)[0] == int(NOW) + 45


def test_a_bare_number_is_epoch_seconds_not_a_year():
    """Checked before the date parsing, or '1756900000' would be read as a calendar year."""
    assert parse_time('1756900000.5', NOW) == (1756900000, 500000000)


def test_wall_clock_today_or_tomorrow():
    """A time still to come today is today; one already past is tomorrow, not in the past."""
    local_now = datetime.fromtimestamp(NOW)
    later = (local_now + timedelta(hours=2)).strftime('%H:%M')
    earlier = (local_now - timedelta(hours=2)).strftime('%H:%M')
    assert parse_time(later, NOW)[0] > NOW
    assert NOW < parse_time(earlier, NOW)[0] <= NOW + 24 * 3600


def test_iso_instants_with_and_without_an_offset():
    with_offset = parse_time('2026-09-03T14:05:00+00:00', NOW)[0]
    assert with_offset == int(datetime(2026, 9, 3, 14, 5, tzinfo=timezone.utc).timestamp())
    # Without one, local time -- the same reading `date` would give on this machine.
    naive = parse_time('2026-09-03T14:05', NOW)[0]
    assert naive == int(datetime(2026, 9, 3, 14, 5).astimezone().timestamp())


def test_datetimes_and_numbers_are_taken_as_they_are():
    """A script has a datetime already; making it format one back into a string is silly."""
    moment = datetime(2026, 9, 3, 14, 5, tzinfo=timezone.utc)
    assert parse_time(moment, NOW)[0] == int(moment.timestamp())
    assert parse_time(1756900000, NOW) == (1756900000, 0)


def test_an_unreadable_time_says_what_it_would_have_accepted():
    with pytest.raises(InvalidRequest, match=r'\+30s'):
        parse_time('next tuesday', NOW)


def test_a_fraction_that_rounds_up_stays_a_legal_timestamp():
    """1e9 is not a valid nanosec field, and the failure would surface as an opaque ROS error."""
    seconds, nanoseconds = parse_time(10.9999999999, NOW)
    assert (seconds, nanoseconds) == (11, 0)


def test_modes_map_to_the_shared_constants():
    assert (parse_mode('node'), parse_mode('publish'), parse_mode('receive')) == (0, 1, 2)
    assert parse_mode(2) == 2


def test_an_unknown_mode_lists_the_real_ones():
    with pytest.raises(InvalidRequest, match='publish'):
        parse_mode('wallclock')


def test_no_schedule_leaves_the_stamp_at_zero():
    """Which is how the recorder is told to act now: it reads a zero stamp as absent."""
    req = request()
    assert apply_schedule(req, None, 'node', '', 'resume_time', 'resume_mode') is None
    assert (req.resume_time.sec, req.resume_time.nanosec) == (0, 0)


def test_a_schedule_fills_the_stamp_the_mode_and_the_tracking_topic():
    req = request()
    at = apply_schedule(req, 1756900000.5, 'publish', '/scan', 'resume_time', 'resume_mode')
    assert (req.resume_time.sec, req.resume_time.nanosec) == (1756900000, 500000000)
    assert req.resume_mode == 1
    assert req.tracking_topic_name == '/scan'
    assert at == pytest.approx(1756900000.5)
