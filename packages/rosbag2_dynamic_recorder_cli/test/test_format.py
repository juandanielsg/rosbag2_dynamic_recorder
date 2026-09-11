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

"""What the CLI prints, and how it reads a time off the command line.

The rule under test is the project's: a number the recorder cannot vouch for is reported as
unknown, never as a convenient zero. The UI package pins the same rule for the browser; this pins
it for the terminal and for --json, where a zero would be even easier to pipe into a report that
then claims nothing was missed.
"""

from datetime import datetime
from types import SimpleNamespace

import pytest

from rosbag2_dynamic_recorder_cli.api import RecorderError
from rosbag2_dynamic_recorder_cli.format import (
    format_bytes,
    format_duration,
    format_status,
    format_topic_group,
    parse_mode,
    parse_time,
    state_word,
    status_dict,
)


def _status(**overrides):
    base = dict(
        uri='/tmp/bag',
        storage_id='mcap',
        recording=True,
        paused=False,
        snapshot_mode=False,
        elapsed_seconds=75.0,
        subscribed_topics=['/a', '/b'],
        active_profile='small',
        messages_written=1987,
        messages_missed=0,
        messages_lost_in_transport=0,
        messages_lost_in_recorder=0,
        messages_lost=0,
        sequence_numbers_available=True,
        write_errors=0,
        bag_splits=0,
        bag_size_bytes=0,
    )
    base.update(overrides)
    return SimpleNamespace(**base)


def test_missed_is_unknown_without_sequence_numbers():
    """Zero here would be a claim the recorder is in no position to make."""
    status = _status(sequence_numbers_available=False, messages_missed=0)
    assert status_dict(status)['messages_missed'] is None
    rendered = '\n'.join(format_status(status, '/rec'))
    assert 'unknown' in rendered
    # And the number that IS knowable is still printed, so this is not just blanket vagueness.
    assert '1987' in rendered


def test_missed_is_a_number_when_the_middleware_supplies_one():
    status = _status(sequence_numbers_available=True, messages_missed=42)
    assert status_dict(status)['messages_missed'] == 42
    assert '42' in '\n'.join(format_status(status, '/rec'))


def test_reported_losses_are_labelled_as_reported():
    """messages_lost has been observed reading 0 while messages were genuinely absent."""
    rendered = '\n'.join(format_status(_status(), '/rec'))
    assert 'as reported by the transport' in rendered


def test_writer_losses_get_their_own_row():
    """A slow disk must not read as a network problem. The writer's own drops have a local
    remedy and are shown on a separate line that says what they are."""
    rendered = format_status(
        _status(messages_lost_in_transport=1, messages_lost_in_recorder=9, messages_lost=10),
        '/rec')
    transport = next(line for line in rendered if line.startswith('lost (transport)'))
    recorder = next(line for line in rendered if line.startswith('lost (recorder)'))
    assert '1 ' in transport
    assert '9 ' in recorder
    assert 'cache' in recorder
    data = status_dict(_status(messages_lost_in_transport=1, messages_lost_in_recorder=9))
    assert data['messages_lost_in_transport'] == 1
    assert data['messages_lost_in_recorder'] == 9


def test_size_on_disk_is_labelled_as_flushed():
    """It reads 0 early in a healthy recording, which looks like a fault unless explained."""
    rendered = '\n'.join(format_status(_status(bag_size_bytes=0), '/rec'))
    assert '0 B' in rendered
    assert 'lags what is captured' in rendered


def test_state_word_distinguishes_the_three_states():
    assert state_word(_status()) == 'recording'
    assert state_word(_status(paused=True)) == 'paused'
    assert state_word(_status(recording=False)) == 'stopped'
    assert 'snapshot mode' in state_word(_status(snapshot_mode=True))


def test_no_matching_profile_says_so_rather_than_showing_a_blank():
    rendered = '\n'.join(format_status(_status(active_profile=''), '/rec'))
    assert '(none matches the current selection)' in rendered


def test_empty_topic_groups_are_omitted():
    """Otherwise every topic change prints three lines regardless of what happened."""
    assert format_topic_group('unavailable', []) == []
    assert format_topic_group('now recording', ['/a']) == ['now recording (1):', '  /a']


def test_format_duration_switches_units_where_a_reader_would():
    assert format_duration(12.34) == '12.3s'
    assert format_duration(75) == '1m 15s'
    assert format_duration(3725) == '1h 02m 05s'
    assert format_duration(-5) == '0.0s'


def test_format_bytes():
    assert format_bytes(0) == '0 B'
    assert format_bytes(512) == '512 B'
    assert format_bytes(1536) == '1.5 KiB'
    assert format_bytes(4 * 1024 * 1024) == '4.0 MiB'


def test_relative_times():
    now = 1_000_000.0
    assert parse_time('+30s', now) == (1_000_030, 0)
    assert parse_time('+5m', now) == (1_000_300, 0)
    assert parse_time('+1h', now) == (1_003_600, 0)
    # A bare +N is seconds, which is what someone typing in a hurry means.
    assert parse_time('+30', now) == (1_000_030, 0)


def test_a_bare_number_is_epoch_seconds_not_a_year():
    assert parse_time('1756900000.5') == (1756900000, 500000000)


def test_wall_clock_time_means_the_next_time_it_comes_round():
    now = datetime(2026, 9, 3, 12, 0, 0).timestamp()
    assert parse_time('14:05', now)[0] == int(datetime(2026, 9, 3, 14, 5).timestamp())
    # Already gone by today, so it means tomorrow -- otherwise the schedule would be in the past
    # and the recorder would fire it immediately.
    assert parse_time('11:00', now)[0] == int(datetime(2026, 9, 4, 11, 0).timestamp())


def test_iso_time_without_an_offset_is_local():
    assert parse_time('2026-09-03T14:05')[0] == int(datetime(2026, 9, 3, 14, 5).timestamp())


def test_unreadable_time_names_the_forms_that_work():
    with pytest.raises(RecorderError) as excinfo:
        parse_time('half past two')
    assert '+30s' in str(excinfo.value)


def test_modes_map_to_the_service_constants():
    assert (parse_mode('node'), parse_mode('publish'), parse_mode('receive')) == (0, 1, 2)
    with pytest.raises(RecorderError, match='unknown mode'):
        parse_mode('wall')
