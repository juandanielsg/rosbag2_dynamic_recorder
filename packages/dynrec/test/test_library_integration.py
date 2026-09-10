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

"""The library driven against a real recorder process.

The unit tests all import the pure modules directly, so every one of them would still pass if
`Recorder` never connected to anything. These start a recorder and talk to it, which is the only
way to find that out -- and the only way to test the two things that are genuinely hard here: that
the client's own context and executor thread work at all, and that a script can watch the event
stream while making calls on the same object.

Runs its own publishers on a private ROS_DOMAIN_ID, so it needs no simulator and cannot collide
with anything else on the machine -- including the CLI's integration tests, which take 72.
"""

import os
import shutil
import subprocess
import tempfile
import threading
import time

import pytest

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from dynrec import CallFailed, InvalidRequest, Recorder, ServiceUnavailable, discover

# A domain of its own, passed explicitly to every participant rather than exported into the
# environment. Setting ROS_DOMAIN_ID at module scope would be a global write at import time, and
# pytest imports every test module before running any of them -- so whichever integration module
# was imported last would silently decide the domain for all of them. That is not hypothetical:
# it is exactly how this module and test_bag_integration.py first collided.
TEST_DOMAIN_ID = os.environ.get('DYNREC_LIB_TEST_DOMAIN_ID', '73')
DOMAIN = int(TEST_DOMAIN_ID)

TOPICS = ['/dynrec_lib_test/alpha', '/dynrec_lib_test/beta', '/dynrec_lib_test/gamma']
NODE = '/rosbag2_dynamic_recorder'


class Publishers(Node):
    """Traffic on the test topics, so the recorder has something to resolve and record."""

    def __init__(self, context):
        super().__init__('dynrec_lib_test_publishers', context=context)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = [self.create_publisher(String, topic, qos) for topic in TOPICS]
        self._seq = 0
        self.create_timer(0.05, self._tick)

    def _tick(self):
        self._seq += 1
        for pub in self._pubs:
            pub.publish(String(data=str(self._seq)))


@pytest.fixture(scope='module')
def running_recorder():
    """One recorder process and one set of publishers for the whole module.

    Module-scoped because starting a recorder costs several seconds and nothing here needs a fresh
    one. Each test therefore states the topic set it wants rather than assuming what the previous
    one left behind.
    """
    env = dict(os.environ, ROS_DOMAIN_ID=TEST_DOMAIN_ID)
    tmp = tempfile.mkdtemp(prefix='dynrec_lib_test_')
    bag = os.path.join(tmp, 'bag')
    exe = os.path.join(
        get_package_prefix('rosbag2_dynamic_recorder'),
        'lib', 'rosbag2_dynamic_recorder', 'dynamic_recorder',
    )
    assert os.path.exists(exe), 'recorder executable not found at {}'.format(exe)
    proc = subprocess.Popen(
        [exe, '--ros-args', '-p', 'uri:={}'.format(bag),
         '-p', 'topics:=[{}]'.format(TOPICS[0]),
         '-p', 'profile_names:=[small,large]',
         '-p', 'profiles.small:=[{}]'.format(TOPICS[0]),
         '-p', 'profiles.large:=[{},{}]'.format(TOPICS[0], TOPICS[1])],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )

    # A context of its own rather than rclpy's global one: the global context cannot be
    # initialised again after shutdown, so a second test module calling rclpy.init() in the same
    # pytest session would fail. Owning it is what dynrec.client does, for the same reason.
    context = rclpy.Context()
    context.init(domain_id=DOMAIN)
    publishers = Publishers(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(publishers)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    try:
        threading.Event().wait(5.0)
        if proc.poll() is not None:
            raise AssertionError('recorder exited early:\n' + proc.stdout.read())
        yield bag
    finally:
        executor.shutdown()
        publishers.destroy_node()
        context.try_shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)


@pytest.fixture(scope='module')
def rec(running_recorder):
    """One long-lived client, which is how the library is meant to be used."""
    with Recorder(domain_id=DOMAIN) as recorder:
        yield recorder


def test_a_client_finds_the_only_recorder_without_being_told_where_it_is(rec):
    """The whole point of discovery by service type: `Recorder()` with no arguments."""
    assert rec.name == NODE
    assert rec.status().recorder == NODE


def test_discover_lists_the_recorder_on_the_graph(running_recorder):
    """The fleet entry point: names first, then a client per name."""
    assert NODE in discover(domain_id=DOMAIN)


def test_status_reports_the_bag_it_was_started_with(rec, running_recorder):
    status = rec.status()
    assert status.recording
    assert status.uri.startswith(running_recorder)
    assert status.storage_id == 'mcap'


def test_add_and_remove_change_what_is_recorded(rec):
    rec.set_topics([TOPICS[0]])

    added = rec.add([TOPICS[1]])
    assert added.complete
    assert sorted(rec.topics()) == sorted(TOPICS[:2])

    removed = rec.remove([TOPICS[1]])
    assert removed.unsubscribed == [TOPICS[1]]
    assert rec.topics() == [TOPICS[0]]


def test_set_topics_replaces_the_whole_selection_and_reports_both_sides(rec):
    rec.set_topics([TOPICS[0]])
    change = rec.set_topics(TOPICS[1:])
    assert sorted(change.subscribed) == sorted(TOPICS[1:])
    assert change.unsubscribed == [TOPICS[0]]


def test_set_topics_with_nothing_records_nothing_without_closing_the_bag(rec):
    rec.set_topics([])
    assert rec.topics() == []
    assert rec.status().recording


def test_a_topic_that_is_not_there_is_reported_rather_than_raised(rec):
    """A partial success is a success to the service, so the result object is the only warning."""
    change = rec.set_topics([TOPICS[0], '/dynrec_lib_test/not_a_topic'])
    assert change.subscribed == [TOPICS[0]]
    assert change.unavailable == ['/dynrec_lib_test/not_a_topic']
    assert not change.complete


def test_a_pattern_selects_topics_without_enumerating_them(rec):
    rec.set_topics([], regex='dynrec_lib_test')
    assert sorted(rec.topics()) == sorted(TOPICS)

    rec.set_topics([], regex='dynrec_lib_test', exclude_regex='gamma')
    assert sorted(rec.topics()) == sorted(TOPICS[:2])


def test_an_empty_add_is_refused_before_it_reaches_the_recorder(rec):
    """A no-op reported as success is what a supervisor script cannot notice."""
    with pytest.raises(InvalidRequest):
        rec.add([])


def test_profiles_switch_the_selection_and_the_active_one_is_derived(rec):
    rec.profile('large')
    assert sorted(rec.topics()) == sorted(TOPICS[:2])

    profiles = rec.profiles()
    assert profiles.active == 'large'
    assert profiles['large'] == TOPICS[:2]

    # Derived from the live topic set on every reply, so one manual change moves the claim.
    rec.remove([TOPICS[1]])
    assert rec.profiles().active == 'small'


def test_an_unknown_profile_raises_with_the_recorder_own_reason(rec):
    with pytest.raises(CallFailed) as excinfo:
        rec.profile('no_such_profile')
    assert excinfo.value.return_code != 0
    assert excinfo.value.error_string


def test_pause_resume_and_toggle_track_the_recorder_state(rec):
    rec.set_topics([TOPICS[0]])
    rec.pause()
    assert rec.is_paused()
    assert rec.status().paused

    assert rec.toggle() is False
    assert not rec.is_paused()

    rec.pause()
    assert rec.resume() is None
    assert not rec.is_paused()


def test_a_scheduled_resume_returns_when_it_will_happen_and_has_not_happened_yet(rec):
    rec.pause()
    try:
        at = rec.resume(at='+30s')
        assert at is not None and at > time.time()
        # Asserted in both directions matters here: a scheduled call that fired immediately would
        # pass a one-sided test that only checked the return value.
        assert rec.is_paused()
    finally:
        # Cancel by resuming now, or the schedule outlives this test.
        rec.resume()


def test_snapshot_explains_itself_when_there_is_no_buffer_to_flush(rec):
    """This recorder was not started in snapshot mode, and the service reports only a bare false."""
    with pytest.raises(CallFailed, match='snapshot_mode'):
        rec.snapshot()


def test_split_rolls_the_bag_over(rec):
    rec.set_topics([TOPICS[0]])
    before = rec.status().bag_splits
    assert rec.split() is None
    assert rec.status().bag_splits == before + 1


def test_events_reach_a_watcher_while_the_same_client_makes_calls(rec):
    """The reason to prefer a library over shelling out: react to a change, do not poll for it."""
    seen = []
    arrived = threading.Event()

    def collect(event):
        seen.append(event)
        if event.topic == TOPICS[2]:
            arrived.set()

    rec.set_topics([TOPICS[0]])
    with rec.on_event(collect):
        rec.add([TOPICS[2]])
        assert arrived.wait(10.0), 'no subscription event arrived for the topic we just added'

    subscribed = [e for e in seen if e.topic == TOPICS[2] and e.action == 'subscribed']
    assert subscribed, 'the added topic produced no subscribed event'
    assert subscribed[0].kind == 'subscription'
    assert subscribed[0].reason, 'the event carries no reason, so it explains nothing'


def test_a_pause_reaches_the_same_watcher_as_a_topic_change(rec):
    """Both streams, flattened, because a bag that stops and a channel that stops are both gaps."""
    paused = threading.Event()

    def collect(event):
        if event.kind == 'pause' and event.action == 'paused':
            paused.set()

    with rec.on_event(collect):
        rec.pause()
        try:
            assert paused.wait(10.0), 'no pause event arrived'
        finally:
            rec.resume()


def test_a_callback_that_raises_does_not_stop_the_others(rec):
    """One bad callback on the executor thread would otherwise take the status stream with it."""
    survived = threading.Event()

    def explode(status):
        raise RuntimeError('deliberate')

    def keep_going(status):
        survived.set()

    with rec.on_status(explode), rec.on_status(keep_going):
        assert survived.wait(10.0), 'a raising callback silenced the stream'


def test_wait_for_returns_at_once_when_the_state_is_already_true(rec):
    """Written against the latched topic, so the current state counts rather than the next change."""
    started = time.monotonic()
    status = rec.wait_for(lambda s: s.recording, timeout=10.0)
    assert status.recording
    assert time.monotonic() - started < 5.0


def test_stop_and_record_bracket_a_bag_and_wait_for_sees_the_transition(rec):
    rec.set_topics([TOPICS[0]])
    rec.stop()
    assert not rec.wait_for(lambda s: not s.recording, timeout=10.0).recording

    # Stopping twice is refused, and the recorder's own words say why.
    with pytest.raises(CallFailed, match='already stopped'):
        rec.stop()

    assert rec.record() is None
    assert rec.wait_for(lambda s: s.recording, timeout=10.0).recording
    # ~/record restores the previous selection rather than opening an empty bag.
    assert rec.topics() == [TOPICS[0]]


def test_a_named_recorder_that_does_not_exist_fails_on_the_call_not_the_connect(
        running_recorder):
    """Discovery lags, so a name is trusted; the timeout is where being wrong shows up.

    The message names the recorder that *is* running, because a typo is the usual cause.
    """
    with Recorder('/no_such_recorder', timeout=2.0, discovery_timeout=2.0,
                  domain_id=DOMAIN) as absent:
        with pytest.raises(ServiceUnavailable, match=NODE):
            absent.status()


def test_many_calls_on_one_client_keep_being_answered(rec):
    """A long-lived script makes thousands of calls, and a client that stops replying is silent.

    A single rclpy node has been observed ceasing to receive service responses after roughly
    7,000 calls, against a client that created and destroyed a client per call. This library keeps
    one per service instead. Three hundred is a floor, not a reproduction of that ceiling -- it is
    what fits in a test suite -- but it fails loudly if reuse breaks outright.
    """
    for _ in range(300):
        rec.is_paused()
    assert rec.status().recording
