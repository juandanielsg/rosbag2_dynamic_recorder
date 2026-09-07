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

"""`describe()` against a real bag whose shape is known by construction.

One set of publishers feeds three topics from a single timer, and the fixture then makes one of
them stop, one start late, and pauses the recorder in the middle. That gives a bag containing every
shape that distinguishes the three ways of computing a rate.

The control is the channel recorded from end to end. Because all three topics share one timer they
were produced at the same rate, whatever rate the host actually managed -- so a sparse channel
reporting the *same* rate as the intact one is the claim, and it holds on a loaded machine where a
nominal 20 Hz does not.

Every assertion is two-sided on purpose. Checking only that the honest rate is right would pass on
a bag with no holes in it at all, where every method agrees -- so each test also pins that the
averaged rate is wrong, which is what makes the bag a real test case.
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

from dynrec import Recorder
from dynrec.bag import describe, summary_dict

# A domain of its own, passed explicitly to every participant rather than exported into the
# environment. Setting ROS_DOMAIN_ID at module scope would be a global write at import time, and
# pytest imports every test module before running any of them -- so whichever integration module
# was imported last would silently decide the domain for all of them.
TEST_DOMAIN_ID = os.environ.get('DYNREC_BAG_TEST_DOMAIN_ID', '74')
DOMAIN = int(TEST_DOMAIN_ID)

#: The publishers' nominal rate. Only used for loose sanity bounds and to set the timer -- the
#: measurements below are compared against the intact channel, not against this, because a loaded
#: host does not always achieve it.
TRUE_HZ = 20.0
PERIOD = 1.0 / TRUE_HZ

TOPICS = ['/dynrec_bag_test/steady', '/dynrec_bag_test/dropped', '/dynrec_bag_test/late']


class Publishers(Node):
    def __init__(self, context):
        super().__init__('dynrec_bag_test_publishers', context=context)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = [self.create_publisher(String, topic, qos) for topic in TOPICS]
        self._seq = 0
        self.create_timer(PERIOD, self._tick)

    def _tick(self):
        self._seq += 1
        for pub in self._pubs:
            pub.publish(String(data=str(self._seq)))


@pytest.fixture(scope='module')
def publishers():
    """One set of publishers for every bag in this module.

    On a context of its own rather than rclpy's global one. The global context cannot be
    initialised a second time once it has been shut down, so two test modules that both call
    rclpy.init() cannot coexist in one pytest session -- and neither can two fixtures in this
    module. Owning the context is the same move `dynrec.client` makes, for the same reason.
    """
    context = rclpy.Context()
    context.init(domain_id=DOMAIN)
    node = Publishers(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        yield node
    finally:
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()


def stop_recorder(proc):
    proc.terminate()
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()


def run_recorder(bag, node_name, extra_params=()):
    """Start a recorder under its own node name.

    The name matters: `~/stop` closes the bag but leaves the node running, so a recorder from an
    earlier fixture is still on the graph when the next one starts. Two nodes sharing the default
    name collapse into one entry during discovery, and calls then land on whichever answers --
    which showed up here as the second bag's `stop` being refused by the first recorder.
    """
    exe = os.path.join(
        get_package_prefix('rosbag2_dynamic_recorder'),
        'lib', 'rosbag2_dynamic_recorder', 'dynamic_recorder')
    assert os.path.exists(exe), 'recorder executable not found at {}'.format(exe)
    return subprocess.Popen(
        [exe, '--ros-args', '-r', '__node:={}'.format(node_name),
         '-p', 'uri:={}'.format(bag),
         '-p', 'topics:=[{},{}]'.format(TOPICS[0], TOPICS[1]), *extra_params],
        env=dict(os.environ, ROS_DOMAIN_ID=TEST_DOMAIN_ID),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def check_alive(proc):
    """Fail with the recorder's own output rather than with a discovery timeout.

    A recorder that died at startup otherwise surfaces as 'no recorder found on the graph', which
    sends you looking at the network instead of at the reason."""
    if proc.poll() is not None:
        raise AssertionError('recorder exited early:\n' + proc.stdout.read())


@pytest.fixture(scope='module')
def shaped_bag(publishers):
    """One bag holding every shape: a steady channel, a dropped one, a late one, and a pause.

    Deliberately the same shapes as the worked example in `dynrec/bag.py`, because those are the
    four cases that distinguish the three ways of computing a rate.
    """
    tmp = tempfile.mkdtemp(prefix='dynrec_bag_test_')
    bag = os.path.join(tmp, 'bag')
    proc = run_recorder(bag, 'shaped_recorder')
    try:
        time.sleep(4.0)
        check_alive(proc)
        with Recorder('/shaped_recorder', domain_id=DOMAIN) as rec:
            time.sleep(4.0)
            rec.remove([TOPICS[1]])          # dropped: stops here
            rec.add([TOPICS[2]])             # late: starts here
            time.sleep(4.0)
            rec.pause()                      # a hole in steady and late at once
            time.sleep(4.0)
            rec.resume()
            time.sleep(4.0)
            rec.stop()
            time.sleep(1.5)
        yield bag
    finally:
        stop_recorder(proc)
        shutil.rmtree(tmp, ignore_errors=True)


@pytest.fixture(scope='module')
def summary(shaped_bag):
    return describe(shaped_bag)


@pytest.fixture(scope='module')
def control_rate(summary):
    """The rate of the channel that was recorded from end to end, used as ground truth.

    It shares its publisher's timer with the other two, so whatever rate the host actually
    achieved, all three were producing it.
    """
    return channel(summary, TOPICS[0]).rate


def assert_matches_control(stats, control):
    """The claim this whole module exists to check, stated once.

    Measured against the continuously-recorded channel rather than against the nominal 20 Hz. All
    three topics are published from a single timer, so their true rates are identical by
    construction -- but on a loaded machine the timer does not always achieve 20 Hz, and a suite
    that asserted the nominal figure would fail for a reason that has nothing to do with the code
    under test. The sparse channels having the *same* rate as the intact one is the real claim.

    Two-sided: the honest rate has to match the control, *and* it has to be dramatically closer to
    it than the averaged number every other tool reports. Checking only the first would pass on a
    bag with no holes in it, where the two agree and nothing is being tested.

    The band on the first check is wide, and the second check is the real assertion. A sparse
    channel's window is only a few seconds long and covers recorder startup, where messages are
    genuinely lost to message-definition resolution and DDS matching -- 102 arriving where 123
    were sent has been measured here. That loss is in the bag, so it is in the rate, and no
    reading of the bag can undo it. What must hold regardless is that reading the events puts the
    answer near the truth while averaging over the bag does not.
    """
    assert stats.rate == pytest.approx(control, rel=0.25), (
        '{}: honest rate {:.2f} should match the intact channel at {:.2f} Hz '
        '(count={}, recorded={:.2f}s, active={:.2f}s, span={:.2f}s)'.format(
            stats.topic, stats.rate, control, stats.count,
            stats.recorded_seconds, stats.active_seconds, stats.last - stats.first))
    assert abs(stats.rate - control) < abs(stats.averaged_rate - control) / 3.0, (
        '{}: honest {:.2f} is not clearly better than averaged {:.2f}'.format(
            stats.topic, stats.rate, stats.averaged_rate))


def channel(summary, topic):
    for stats in summary.channels:
        if stats.topic == topic:
            return stats
    raise AssertionError('{} is not in the bag: {}'.format(
        topic, [c.topic for c in summary.channels]))


def test_the_bag_is_one_file_with_all_three_topics(summary):
    assert {c.topic for c in summary.channels} == set(TOPICS)


def test_a_channel_with_a_pause_shaped_hole_recovers_its_true_rate(summary):
    """The case span arithmetic cannot fix: the hole is interior, so the span is the whole bag."""
    steady = channel(summary, TOPICS[0])
    assert steady.basis == 'events'
    # The control channel itself, so this is the one place a nominal figure is checked -- loosely,
    # only to catch an implementation that is wrong by more than the host's own jitter.
    assert TRUE_HZ * 0.7 < steady.rate < TRUE_HZ * 1.1, steady.rate
    assert steady.averaged_rate < steady.rate * 0.9, (
        'averaged rate {:.2f} is not wrong enough for this bag to prove anything'.format(
            steady.averaged_rate))

    # And the span is no help here, because the hole is in the middle of it.
    span_rate = steady.count / (steady.last - steady.first)
    assert span_rate < steady.rate * 0.9, (
        'span rate {} should also be wrong for an interior hole'.format(span_rate))


def test_a_channel_that_stopped_recovers_its_true_rate(summary, control_rate):
    dropped = channel(summary, TOPICS[1])
    assert_matches_control(dropped, control_rate)
    assert dropped.averaged_rate < control_rate * 0.6, (
        'a channel recorded for a third of the bag should average far below its true rate')


def test_a_channel_that_started_late_recovers_its_true_rate(summary, control_rate):
    late = channel(summary, TOPICS[2])
    assert_matches_control(late, control_rate)
    assert late.averaged_rate < control_rate * 0.9


def test_active_time_never_exceeds_recorded_time(summary):
    """The invariant behind the two fields, and the reason the rate uses the second one.

    Subscribing is not instantaneous: the SUBSCRIBED event is stamped when the subscription is
    created, but the first message cannot arrive until the message definition has been resolved --
    ~0.4-0.6s by notes/spike-plan.md, plus DDS matching. How long that takes varies with whether
    discovery is already warm, so its size is not worth asserting; that it is never negative, and
    that the rate is measured over the trimmed window, is.
    """
    for stats in summary.channels:
        assert stats.active_seconds <= stats.recorded_seconds + 1e-9, stats.topic
        assert stats.rate == pytest.approx(stats.count / stats.active_seconds), stats.topic


def test_the_dropped_channel_is_not_charged_for_a_pause_it_missed(summary):
    """It was already unsubscribed when the pause happened, so none of it is its own downtime."""
    dropped = channel(summary, TOPICS[1])
    pause_start = summary.pause_windows[0][0]
    assert dropped.last < pause_start, 'the fixture should drop this topic before pausing'
    # Its active time is the span it was delivering over, undiminished by a pause it was not
    # present for -- which is the whole reason a pause cannot be subtracted from every channel.
    assert dropped.active_seconds == pytest.approx(dropped.last - dropped.first, abs=0.2)


def test_the_pause_is_found_and_is_roughly_the_length_it_was(summary):
    assert len(summary.pause_windows) == 1, summary.pause_windows
    begin, finish = summary.pause_windows[0]
    assert 3.0 < finish - begin < 6.0, 'expected a ~4s pause, got {:.2f}s'.format(finish - begin)


def test_a_pause_that_is_explained_is_not_reported_as_unexplained(summary):
    """The whole point of writing the events: this hole has an account of itself."""
    assert summary.unexplained_gaps == []
    assert not [w for w in summary.warnings if 'no pause event explains it' in w]


def test_channels_are_marked_sparse_when_they_were_not_live_throughout(summary):
    assert channel(summary, TOPICS[1]).sparse
    assert channel(summary, TOPICS[2]).sparse


def test_the_report_shows_both_numbers_so_they_can_be_compared(summary):
    from dynrec.bag import format_summary
    text = '\n'.join(format_summary(summary))
    assert 'averaged' in text
    assert TOPICS[0] in text


def test_the_json_shape_keeps_unknown_as_null(summary):
    payload = summary_dict(summary)
    assert payload['channels']
    for entry in payload['channels']:
        assert entry['rate'] is None or entry['rate'] > 0
        assert 'averaged_rate' in entry and 'basis' in entry


class TestWithoutEvents:
    """A bag recorded with the provenance turned off, which is the case that must not lie."""

    @pytest.fixture(scope='class')
    def bare_bag(self, publishers):
        tmp = tempfile.mkdtemp(prefix='dynrec_bag_bare_')
        bag = os.path.join(tmp, 'bag')
        proc = run_recorder(bag, 'bare_recorder', extra_params=[
            '-p', 'record_subscription_events:=false',
            '-p', 'record_pause_events:=false'])
        try:
            time.sleep(4.0)
            check_alive(proc)
            with Recorder('/bare_recorder', domain_id=DOMAIN) as rec:
                time.sleep(3.0)
                rec.pause()
                time.sleep(4.0)
                rec.resume()
                time.sleep(3.0)
                rec.stop()
                time.sleep(1.5)
            yield bag
        finally:
            stop_recorder(proc)
            shutil.rmtree(tmp, ignore_errors=True)

    def test_it_says_the_provenance_is_missing_rather_than_inventing_it(self, bare_bag):
        summary = describe(bare_bag)
        assert any('no subscription events' in w for w in summary.warnings), summary.warnings
        assert any('no pause events' in w for w in summary.warnings), summary.warnings
        for stats in summary.channels:
            assert stats.basis == 'span', (
                'without events there is nothing better than the span to measure over')

    def test_an_unexplained_hole_across_every_channel_is_surfaced(self, bare_bag):
        """This is the bag the project exists to make impossible, so the reader must flag it.

        A pause recorded with the events off leaves exactly the signature of a crash or a stall.
        The tool cannot say which -- and says so, rather than quietly averaging over it.
        """
        summary = describe(bare_bag)
        assert summary.unexplained_gaps, 'a 4s hole in every channel went unreported'
        # *A* pause-sized gap, not the first one. A shared hole of about a second also turns up
        # from time to time -- the environmental stall characterised in notes/recording-stall.md,
        # which is not ours and which this reader is right to surface alongside the pause.
        pause_sized = [
            (begin, finish) for begin, finish in summary.unexplained_gaps
            if 3.0 < finish - begin < 6.0
        ]
        assert pause_sized, 'no pause-sized hole among {}'.format(
            [round(finish - begin, 2) for begin, finish in summary.unexplained_gaps])
        assert any('no pause event explains it' in w for w in summary.warnings)
