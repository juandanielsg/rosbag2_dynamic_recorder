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

"""What the CLI prints.

The rule under test is the project's: a number the recorder cannot vouch for is reported as
unknown, never as a convenient zero. The UI package pins the same rule for the browser; this pins
it for the terminal and for --json, where a zero would be even easier to pipe into a report that
then claims nothing was missed.

How a time is read off the command line is no longer this package's rule: the CLI shares
`dynrec.schedule` with scripts, and that dialect is pinned in dynrec/test/test_schedule.py.
"""

from types import SimpleNamespace

from dynrec.results import Status
from rosbag2_dynamic_recorder_cli.format import (
    format_bytes,
    format_duration,
    format_status,
    format_topic_group,
    state_word,
)


def _status(**overrides):
    """A status as the verb hands it to the formatter: read through dynrec, which is where the
    unknown-versus-zero rule for `messages_missed` lives. Overrides are message fields."""
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
        free_space_bytes=0,
        total_space_bytes=0,
        min_free_space=0,
        min_free_space_percent=0.0,
        stopped_for_low_disk=False,
        max_bag_size=0,
        stopped_for_max_bag_size=False,
        use_sim_time=False,
        waiting_for_clock=False,
    )
    base.update(overrides)
    base.setdefault('recording_started', SimpleNamespace(sec=1788500000, nanosec=0))
    return Status.from_msg(SimpleNamespace(**base), recorder='/rec')


def test_missed_is_unknown_without_sequence_numbers():
    """Zero here would be a claim the recorder is in no position to make."""
    status = _status(sequence_numbers_available=False, messages_missed=0)
    assert status.as_dict()['messages_missed'] is None
    rendered = '\n'.join(format_status(status))
    assert 'unknown' in rendered
    # And the number that IS knowable is still printed, so this is not just blanket vagueness.
    assert '1987' in rendered


def test_missed_is_a_number_when_the_middleware_supplies_one():
    status = _status(sequence_numbers_available=True, messages_missed=42)
    assert status.as_dict()['messages_missed'] == 42
    assert '42' in '\n'.join(format_status(status))


def test_reported_losses_are_labelled_as_reported():
    """messages_lost has been observed reading 0 while messages were genuinely absent."""
    rendered = '\n'.join(format_status(_status()))
    assert 'as reported by the transport' in rendered


def test_writer_losses_get_their_own_row():
    """A slow disk must not read as a network problem. The writer's own drops have a local
    remedy and are shown on a separate line that says what they are."""
    rendered = format_status(
        _status(messages_lost_in_transport=1, messages_lost_in_recorder=9, messages_lost=10))
    transport = next(line for line in rendered if line.startswith('lost (transport)'))
    recorder = next(line for line in rendered if line.startswith('lost (recorder)'))
    assert '1 ' in transport
    assert '9 ' in recorder
    assert 'cache' in recorder
    data = _status(messages_lost_in_transport=1, messages_lost_in_recorder=9).as_dict()
    assert data['messages_lost_in_transport'] == 1
    assert data['messages_lost_in_recorder'] == 9


def test_size_on_disk_is_labelled_as_flushed():
    """It reads 0 early in a healthy recording, which looks like a fault unless explained."""
    rendered = '\n'.join(format_status(_status(bag_size_bytes=0)))
    assert '0 B' in rendered
    assert 'lags what is captured' in rendered


def test_state_word_distinguishes_the_three_states():
    assert state_word(_status()) == 'recording'
    assert state_word(_status(paused=True)) == 'paused'
    assert state_word(_status(recording=False)) == 'stopped'
    assert 'snapshot mode' in state_word(_status(snapshot_mode=True))


def test_a_self_inflicted_stop_says_why():
    """After the recorder stops itself, 'stopped' alone is the one answer a reader cannot use."""
    assert 'free space' in state_word(_status(recording=False, stopped_for_low_disk=True))
    assert 'max_bag_size' in state_word(_status(recording=False, stopped_for_max_bag_size=True))


def test_waiting_for_the_sim_clock_is_not_reported_as_stopped():
    assert '/clock' in state_word(_status(recording=False, waiting_for_clock=True))


def test_free_space_is_unknown_when_the_filesystem_could_not_be_read():
    """The recorder reports 0/0 in that case, and 0 B free would read as a full disk."""
    rendered = '\n'.join(format_status(_status()))
    assert 'free space:' in rendered and 'unknown' in rendered


def test_free_space_shows_the_floor_the_guard_enforces():
    rendered = '\n'.join(format_status(_status(
        free_space_bytes=3 * 1024 ** 3, total_space_bytes=8 * 1024 ** 3,
        min_free_space=1024 ** 3, min_free_space_percent=10.0, max_bag_size=2 * 1024 ** 3)))
    assert '3.0 GiB of 8.0 GiB (recording stops below 1.0 GiB or 10%)' in rendered
    assert 'of 2.0 GiB allowed' in rendered


def test_no_matching_profile_says_so_rather_than_showing_a_blank():
    rendered = '\n'.join(format_status(_status(active_profile='')))
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
