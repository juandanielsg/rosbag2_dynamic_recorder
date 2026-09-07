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

"""The rule that must not erode: unmeasurable is None, never a convenient zero.

This is the third place it is enforced -- after the browser UI's `build_state()` and the CLI's
`--json` -- and the most consequential, because a library return value is the one a script will
compare against a threshold and act on without a human reading it first.

Message objects are stood up as SimpleNamespace, which is exactly what the `from_msg`
constructors were kept attribute-only for: no ROS installation is needed to run any of this.
"""

from types import SimpleNamespace

from dynrec.results import Event, Profiles, Status, TopicChange


def stamp(seconds=0, nanoseconds=0):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def status_msg(**overrides):
    fields = dict(
        uri='/tmp/bag',
        storage_id='mcap',
        recording=True,
        paused=False,
        snapshot_mode=False,
        recording_started=stamp(1756900000, 500000000),
        elapsed_seconds=12.5,
        subscribed_topics=['/scan', '/odom'],
        active_profile='small',
        messages_written=1000,
        messages_lost=0,
        messages_missed=7,
        sequence_numbers_available=True,
        write_errors=0,
        bag_splits=1,
        bag_size_bytes=4096,
    )
    fields.update(overrides)
    return SimpleNamespace(**fields)


def test_missed_messages_are_unknown_when_the_middleware_cannot_count_them():
    """Not zero. Zero is a claim the recorder is in no position to make."""
    status = Status.from_msg(status_msg(sequence_numbers_available=False, messages_missed=0))
    assert status.messages_missed is None
    assert status.as_dict()['messages_missed'] is None


def test_missed_messages_are_reported_when_they_are_measurable():
    assert Status.from_msg(status_msg()).messages_missed == 7


def test_reported_losses_keep_their_name():
    """`messages_lost` is what the transport admitted to, and has read 0 against a lossy bag.

    Renaming it on the way out is the point: a caller reaching for `messages_lost_reported` is
    likelier to remember it is a floor rather than a total.
    """
    status = Status.from_msg(status_msg(messages_lost=3))
    assert status.messages_lost_reported == 3
    assert not hasattr(status, 'messages_lost')


def test_recording_started_is_seconds_on_the_recorder_clock():
    assert Status.from_msg(status_msg()).recording_started == 1756900000.5


def test_the_recorder_name_travels_with_the_status():
    """So a script holding two of these can tell them apart after the fact."""
    assert Status.from_msg(status_msg(), recorder='/left').recorder == '/left'


def test_a_change_that_left_something_out_says_so_without_raising():
    """The service calls this success, so the unavailable list is the only warning there is."""
    change = TopicChange.from_msg(SimpleNamespace(
        subscribed_topics=['/scan'], unavailable_topics=['/camera'], return_code=0))
    assert change.subscribed == ['/scan']
    assert change.unavailable == ['/camera']
    assert not change.complete


def test_a_change_that_got_everything_is_complete():
    change = TopicChange.from_msg(SimpleNamespace(
        subscribed_topics=['/scan'], unsubscribed_topics=[], unavailable_topics=[]))
    assert change.complete


def test_a_removal_reports_what_was_not_being_recorded_anyway():
    change = TopicChange.from_msg(SimpleNamespace(
        unsubscribed_topics=['/scan'], not_subscribed_topics=['/never']))
    assert change.unsubscribed == ['/scan']
    assert change.not_subscribed == ['/never']
    # The fields this service does not fill stay empty rather than becoming absent, so one result
    # type serves all four calls.
    assert change.subscribed == []


def test_profiles_read_like_a_mapping_and_remember_the_active_one():
    profiles = Profiles.from_msg(SimpleNamespace(
        profiles=[
            SimpleNamespace(name='small', topics=['/scan']),
            SimpleNamespace(name='large', topics=['/scan', '/odom']),
        ],
        active_profile='small'))
    assert profiles.names == ['small', 'large']
    assert profiles['large'] == ['/scan', '/odom']
    assert 'small' in profiles
    assert profiles.active == 'small'
    assert len(profiles) == 2


def test_no_active_profile_is_the_normal_answer_after_a_manual_change():
    """The recorder derives this every time, so an empty value is the truth, not a gap."""
    profiles = Profiles.from_msg(SimpleNamespace(profiles=[], active_profile=''))
    assert profiles.active == ''
    assert profiles.names == []


def test_subscription_events_come_out_as_words():
    """A script comparing against 0 and 1 is one constant rename away from inverting itself."""
    event = Event.from_subscription_msg(SimpleNamespace(
        action=1, stamp=stamp(10, 0), reason='service:set_topics',
        node_name='/rosbag2_dynamic_recorder', topic_name='/scan',
        topic_type='sensor_msgs/msg/LaserScan'))
    assert (event.kind, event.action, event.topic) == ('subscription', 'unsubscribed', '/scan')
    assert event.stamp == 10.0


def test_pause_events_name_no_topic_because_they_affect_every_one():
    event = Event.from_pause_msg(SimpleNamespace(
        action=0, stamp=stamp(20, 250000000), reason='service:pause',
        node_name='/rosbag2_dynamic_recorder'))
    assert (event.kind, event.action, event.topic) == ('pause', 'paused', '')
    assert event.stamp == 20.25


def test_both_event_streams_sort_together_on_the_recorder_stamp():
    """They arrive on separate subscriptions, so arrival order is not what happened."""
    pause = Event.from_pause_msg(SimpleNamespace(
        action=0, stamp=stamp(5), reason='service:pause', node_name='/r'))
    change = Event.from_subscription_msg(SimpleNamespace(
        action=0, stamp=stamp(3), reason='startup', node_name='/r',
        topic_name='/scan', topic_type='std_msgs/msg/String'))
    assert [e.stamp for e in sorted([pause, change], key=lambda e: e.stamp)] == [3.0, 5.0]
