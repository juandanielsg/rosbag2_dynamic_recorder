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

"""The interval arithmetic that turns events into an honest rate.

This is the part with the judgement in it, so it is tested against hand-built events rather than
against a recorded bag: every case below can be stated exactly, including the ones that are
awkward to produce on a real recorder -- a bag that ends while paused, a topic recorded twice in
two separate stretches, a hole that no pause explains.

No ROS, and no bag. `dynrec.bag`'s analysis half imports neither.
"""

import pytest

from dynrec.bag import (
    gaps_in,
    live_seconds,
    merge_windows,
    overlap,
    pause_windows,
    rate_over,
    subscribed_windows,
    unexplained_gaps,
)
from dynrec.results import Event


def sub(topic, action, stamp):
    return Event(kind='subscription', action=action, stamp=stamp, reason='test',
                 node_name='/rec', topic=topic)


def pause(action, stamp):
    return Event(kind='pause', action=action, stamp=stamp, reason='test', node_name='/rec')


def test_windows_merge_when_they_touch_or_overlap():
    assert merge_windows([(0, 5), (3, 8), (20, 25)]) == [(0, 8), (20, 25)]
    # A zero-length window is not a window.
    assert merge_windows([(4, 4)]) == []


def test_overlap_is_zero_for_disjoint_intervals():
    assert overlap((0, 5), (10, 15)) == 0.0
    assert overlap((0, 10), (5, 20)) == 5.0


def test_a_topic_recorded_to_the_end_gets_a_window_that_ends_with_the_bag():
    """The most ordinary case there is: still recording when the bag stopped."""
    events = [sub('/scan', 'subscribed', 10.0)]
    assert subscribed_windows(events, '/scan', bag_end=100.0) == [(10.0, 100.0)]


def test_a_topic_dropped_mid_bag_gets_a_window_that_ends_there():
    events = [sub('/scan', 'subscribed', 10.0), sub('/scan', 'unsubscribed', 40.0)]
    assert subscribed_windows(events, '/scan', bag_end=100.0) == [(10.0, 40.0)]


def test_a_topic_recorded_twice_gets_two_windows():
    """Dropped and later re-added: the time in between is not time it was recording."""
    events = [
        sub('/scan', 'subscribed', 10.0), sub('/scan', 'unsubscribed', 20.0),
        sub('/scan', 'subscribed', 60.0), sub('/scan', 'unsubscribed', 70.0),
    ]
    assert subscribed_windows(events, '/scan', 100.0) == [(10.0, 20.0), (60.0, 70.0)]
    assert live_seconds(subscribed_windows(events, '/scan', 100.0), []) == 20.0


def test_events_for_other_topics_are_ignored():
    events = [sub('/other', 'subscribed', 0.0), sub('/scan', 'subscribed', 10.0)]
    assert subscribed_windows(events, '/scan', 100.0) == [(10.0, 100.0)]


def test_pause_windows_pair_up():
    events = [pause('paused', 10.0), pause('resumed', 15.0),
              pause('paused', 30.0), pause('resumed', 33.0)]
    assert pause_windows(events) == [(10.0, 15.0), (30.0, 33.0)]


def test_a_bag_that_ends_while_paused_stays_paused_to_the_end():
    """Everything after that instant genuinely was not written, so it is not recorded time."""
    events = [sub('/scan', 'subscribed', 0.0), pause('paused', 40.0)]
    assert pause_windows(events, bag_end=50.0) == [(40.0, 50.0)]

    # Without a bag end the last event stands in for it. Here that is the PAUSED event itself,
    # because a paused recorder writes nothing after it -- so the window is empty and no recorded
    # time is wrongly deducted. Pinned because the alternative, guessing a length, would be worse.
    assert pause_windows(events) == []


def test_pausing_removes_time_from_every_window_it_overlaps():
    """The case span arithmetic cannot see: a hole in the middle of a live channel."""
    windows = [(0.0, 100.0)]
    assert live_seconds(windows, [(20.0, 30.0)]) == 90.0
    # A pause outside the window costs nothing.
    assert live_seconds(windows, [(200.0, 300.0)]) == 100.0
    # A pause straddling the end only counts the part inside.
    assert live_seconds(windows, [(95.0, 120.0)]) == 95.0


def test_a_pause_while_a_topic_was_not_subscribed_costs_that_topic_nothing():
    """Which is why beta in the worked example has no pause-shaped hole -- it had already gone."""
    windows = [(0.0, 10.0)]
    assert live_seconds(windows, [(20.0, 30.0)]) == 10.0


def test_the_rate_is_unknown_rather_than_infinite_when_there_is_no_interval():
    assert rate_over(5, 0.0) is None
    assert rate_over(5, None) is None
    assert rate_over(200, 10.0) == 20.0


def test_the_honest_rate_and_the_averaged_rate_differ_exactly_as_measured():
    """The worked example from the module docstring, as arithmetic.

    beta published at 20 Hz for 11s of a 36.6s bag. Averaging over the bag says 5.95 Hz.
    """
    count = 218
    assert round(count / 36.6, 2) == 5.96          # what mcap info reports
    assert round(rate_over(count, 11.0), 1) == 19.8  # what it was really doing


def test_gaps_are_found_only_above_the_threshold():
    stamps = [0.0, 0.05, 0.10, 5.10, 5.15]
    assert gaps_in(stamps, 1.0) == [(0.10, 5.10)]
    assert gaps_in(stamps, 10.0) == []


def test_a_hole_in_one_channel_alone_is_not_unexplained():
    """That is just an unsubscription, and the subscription events already explain it."""
    channel_gaps = [[(10.0, 20.0)], []]
    assert unexplained_gaps(channel_gaps, pauses=[]) == []


def test_a_hole_in_every_channel_with_no_pause_behind_it_is_reported():
    """A crash, a stall, or a pause recorded with record_pause_events:=false."""
    channel_gaps = [[(10.0, 20.0)], [(11.0, 19.0)]]
    assert unexplained_gaps(channel_gaps, pauses=[]) == [(11.0, 19.0)]


def test_the_same_hole_is_not_reported_once_a_pause_explains_it():
    channel_gaps = [[(10.0, 20.0)], [(10.0, 20.0)]]
    assert unexplained_gaps(channel_gaps, pauses=[(10.0, 20.0)]) == []


def test_a_brief_shared_hiccup_is_not_worth_alarming_about():
    """Otherwise ordinary jitter across a few topics would raise a permanent false alarm."""
    channel_gaps = [[(10.0, 10.2)], [(10.0, 10.2)]]
    assert unexplained_gaps(channel_gaps, pauses=[], threshold=1.0) == []


@pytest.mark.parametrize('pauses', [[], [(0.0, 1.0)]])
def test_no_channels_means_nothing_to_report(pauses):
    assert unexplained_gaps([], pauses) == []
