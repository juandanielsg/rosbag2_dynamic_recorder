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

"""`ros2 dynrec` driven as a real subprocess against a real recorder.

The unit tests cover the parts with judgement in them, but they all import the code directly, so
every one of them would still pass if the entry points in setup.py were wrong and `ros2 dynrec`
did not exist at all. These tests run the actual command, which is the only way to find that out.

They also pin the promise the package is for: a script can drive the recorder and rely on the
exit code alone.

Runs its own publishers on a private ROS_DOMAIN_ID, so it needs no simulator and cannot collide
with anything else on the machine.
"""

import json
import os
import shutil
import subprocess
import tempfile
import threading

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# A domain of its own, for the same reason the recorder's tests take one: a developer's own nodes
# must not join this graph, and 72 is only "probably unused" by anyone else.
# Set before any rclpy.init(): the publishers, the recorder subprocess and every `ros2 dynrec`
# call have to land on the SAME domain or they never see each other.
TEST_DOMAIN_ID = os.environ.get('DYNREC_CLI_TEST_DOMAIN_ID', '72')
os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN_ID

TOPICS = ['/dynrec_cli_test/alpha', '/dynrec_cli_test/beta', '/dynrec_cli_test/gamma']
NODE = '/rosbag2_dynamic_recorder'


class Publishers(Node):
    """Traffic on the test topics, so the recorder has something to resolve and record."""

    def __init__(self):
        super().__init__('dynrec_cli_test_publishers')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = [self.create_publisher(String, topic, qos) for topic in TOPICS]
        self._seq = 0
        self.create_timer(0.05, self._tick)

    def _tick(self):
        self._seq += 1
        for pub in self._pubs:
            pub.publish(String(data=str(self._seq)))


def dynrec(*args, timeout=60):
    """Run `ros2 dynrec ...` and return the CompletedProcess.

    --spin-time is shortened from the default because the recorder is already up by the time any
    of these run, and a full second of discovery per invocation dominates the test's runtime. It
    is only meaningful on a verb, so it is left off when the first argument is an option such as
    --help.
    """
    ros2 = shutil.which('ros2')
    assert ros2, 'ros2 is not on PATH; is the workspace sourced?'
    tuning = [] if args[0].startswith('-') else ['--spin-time', '0.5']
    return subprocess.run(
        [ros2, 'dynrec', *args, *tuning],
        env=dict(os.environ, ROS_DOMAIN_ID=TEST_DOMAIN_ID),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout,
    )


def topics_now():
    result = dynrec('topics')
    assert result.returncode == 0, result.stderr
    return result.stdout.split()


@pytest.fixture(scope='module')
def recorder():
    """One recorder and one set of publishers for the whole module.

    Module-scoped deliberately: each `ros2 dynrec` invocation costs a couple of seconds of
    process startup, and restarting the recorder for every test would triple a suite whose
    subject is a thin client. The tests below therefore state the topic set they need rather
    than assuming what the previous one left behind.
    """
    env = dict(os.environ, ROS_DOMAIN_ID=TEST_DOMAIN_ID)
    tmp = tempfile.mkdtemp(prefix='dynrec_cli_test_')
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

    rclpy.init(args=None)
    publishers = Publishers()
    executor = SingleThreadedExecutor()
    executor.add_node(publishers)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    try:
        # Let the recorder come up and the publishers reach the graph before the first call.
        threading.Event().wait(5.0)
        if proc.poll() is not None:
            raise AssertionError('recorder exited early:\n' + proc.stdout.read())
        yield bag
    finally:
        executor.shutdown()
        publishers.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)


def test_every_verb_is_registered(recorder):
    """A verb missing from setup.py's entry points is invisible to every other test here."""
    result = dynrec('--help')
    assert result.returncode == 0, result.stderr
    for verb in ('add', 'pause', 'profile', 'profiles', 'record', 'remove', 'resume', 'set',
                 'snapshot', 'split', 'status', 'stop', 'toggle', 'topics'):
        assert verb in result.stdout, 'verb {} is not registered'.format(verb)


def test_status_finds_the_recorder_without_being_told_where_it_is(recorder):
    """The whole point of discovery by service type: `ros2 dynrec status` with no arguments."""
    result = dynrec('status')
    assert result.returncode == 0, result.stderr
    assert NODE in result.stdout
    assert recorder in result.stdout


def test_add_and_remove_change_what_is_recorded(recorder):
    dynrec('set', TOPICS[0])
    assert dynrec('add', TOPICS[1]).returncode == 0
    assert sorted(topics_now()) == sorted(TOPICS[:2])

    assert dynrec('remove', TOPICS[1]).returncode == 0
    assert topics_now() == [TOPICS[0]]


def test_set_replaces_the_whole_selection(recorder):
    dynrec('set', TOPICS[0])
    result = dynrec('set', TOPICS[1], TOPICS[2])
    assert result.returncode == 0, result.stderr
    assert sorted(topics_now()) == sorted(TOPICS[1:])
    # The topic that went away is reported, not silently dropped.
    assert TOPICS[0] in result.stdout


def test_set_with_no_topics_records_nothing(recorder):
    assert dynrec('set').returncode == 0
    assert topics_now() == []


def test_profile_switches_the_selection_and_is_then_reported_active(recorder):
    assert dynrec('profile', 'large').returncode == 0
    assert sorted(topics_now()) == sorted(TOPICS[:2])

    result = dynrec('profiles')
    assert result.returncode == 0, result.stderr
    assert '* large' in result.stdout

    # Derived from the live topic set, so one manual change must drop the claim entirely.
    dynrec('remove', TOPICS[1])
    assert '* small' in dynrec('profiles').stdout


def test_pause_and_resume_are_visible_in_the_status(recorder):
    dynrec('set', TOPICS[0])
    assert dynrec('pause').returncode == 0
    assert json.loads(dynrec('status', '--json').stdout)['paused'] is True
    assert dynrec('resume').returncode == 0
    assert json.loads(dynrec('status', '--json').stdout)['paused'] is False


def test_json_status_never_reports_a_count_it_cannot_measure(recorder):
    """The project's rule, at the boundary where it is easiest to pipe a wrong number onward."""
    payload = json.loads(dynrec('status', '--json').stdout)
    assert payload['recorder'] == NODE
    # Either a real count or an explicit null -- never a zero standing in for "no idea".
    assert payload['messages_missed'] is None or isinstance(payload['messages_missed'], int)


def test_a_refused_call_exits_non_zero_and_explains_itself(recorder):
    """The promise the package is for: a supervisor can trust the exit code alone."""
    result = dynrec('profile', 'nosuch')
    assert result.returncode == 1
    assert 'nosuch' in result.stderr
    # And it says what IS configured, so the fix does not need a second command.
    assert 'small' in result.stderr and 'large' in result.stderr


def test_an_unavailable_topic_is_reported_as_such(recorder):
    result = dynrec('add', '/dynrec_cli_test/nobody_publishes_this')
    assert result.returncode == 1
    assert 'unavailable' in result.stdout


def test_a_wrong_node_name_names_the_recorder_that_is_actually_running(recorder):
    """A typo in --node is the common cause, and a bare timeout would not say so."""
    result = dynrec('status', '--node', '/not_the_recorder', '--timeout', '2')
    assert result.returncode == 1
    assert NODE in result.stderr


def test_stop_then_record_reopens_the_bag(recorder):
    dynrec('set', TOPICS[0])
    assert dynrec('stop').returncode == 0
    assert json.loads(dynrec('status', '--json').stdout)['recording'] is False

    # A second stop is refused rather than silently accepted, and the CLI passes that on.
    assert dynrec('stop').returncode == 1

    assert dynrec('record').returncode == 0
    payload = json.loads(dynrec('status', '--json').stdout)
    assert payload['recording'] is True
    # ~/record restores the selection from before the stop rather than starting empty.
    assert payload['subscribed_topics'] == [TOPICS[0]]
