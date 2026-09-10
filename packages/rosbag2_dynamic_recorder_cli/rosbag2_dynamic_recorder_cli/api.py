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

"""Shared plumbing for the `ros2 dynrec` verbs.

Everything here that can be a plain function is one, so the rules with actual judgement in them
-- how a recorder is identified, and what counts as unknown rather than zero -- can be tested
without a ROS graph. That is the same split the UI package made for `build_state()`, for the
same reason.
"""

import sys
from contextlib import contextmanager

import rclpy
from ros2cli.node.direct import DirectNode

#: The service every recorder offers, used to identify one on the graph.
GET_STATUS_TYPE = 'rosbag2_dynamic_recorder_interfaces/srv/GetStatus'
GET_STATUS_SUFFIX = '/get_status'

#: Seconds to wait for a service to appear and then to reply. Generous because adding a topic
#: costs ~0.5s inside the recorder and a set_topics over a dozen topics is slower still.
DEFAULT_TIMEOUT = 10.0


class RecorderError(Exception):
    """A failure the user can act on: no recorder, several recorders, or a refused call."""


def recorder_nodes(service_names_and_types):
    """Fully qualified names of the recorders visible on the graph, sorted.

    A recorder is identified by offering `~/get_status` of our own type, not by node name. A
    recorder launched under a different name or pushed into a namespace is therefore still found,
    and an unrelated node that happens to be called `rosbag2_dynamic_recorder` is not.
    """
    found = set()
    for name, types in service_names_and_types:
        if name.endswith(GET_STATUS_SUFFIX) and GET_STATUS_TYPE in types:
            found.add(name[:-len(GET_STATUS_SUFFIX)])
    return sorted(found)


def choose_recorder(discovered, requested=None):
    """Pick the recorder to talk to, or explain why that is not possible.

    A requested name is trusted even when discovery did not turn it up: graph discovery can lag,
    and refusing a name that is about to appear would be worse than trying it and timing out.
    """
    if requested:
        return '/' + requested.strip('/')
    if not discovered:
        raise RecorderError(
            'no rosbag2_dynamic_recorder found on the graph.\n'
            'Is one running, and is ROS_DOMAIN_ID the same in both shells?')
    if len(discovered) > 1:
        listed = '\n  '.join(discovered)
        raise RecorderError(
            'several recorders are running; pick one with --node:\n  ' + listed)
    return discovered[0]


class Recorder:
    """A resolved recorder, and the one way this package talks to it."""

    def __init__(self, node, name, discovered, timeout=DEFAULT_TIMEOUT):
        self.node = node
        self.name = name
        self.discovered = discovered
        self.timeout = timeout

    def call(self, srv_type, verb, request=None):
        """Call `<recorder>/<verb>` and return the response, or raise RecorderError."""
        service = '{}/{}'.format(self.name, verb)
        client = self.node.create_client(srv_type, service)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise RecorderError(self.unreachable_message(service))
            future = client.call_async(request if request is not None else srv_type.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
            if not future.done():
                raise RecorderError(
                    '{} did not reply within {:g}s. '
                    'The recorder is up but busy or wedged.'.format(service, self.timeout))
            return future.result()
        finally:
            self.node.destroy_client(client)

    def unreachable_message(self, service):
        """Explain a missing service, naming the recorders that *are* there.

        A typo in --node is the common cause, so listing what was found turns a bare timeout into
        an answer.
        """
        message = '{} is not available after {:g}s.'.format(service, self.timeout)
        others = [name for name in self.discovered if name != self.name]
        if others:
            return message + '\nRecorders that are running:\n  ' + '\n  '.join(others)
        if not self.discovered:
            return message + '\nNo recorder was found on the graph at all.'
        return message


def add_recorder_arguments(parser):
    """Arguments every verb takes."""
    parser.add_argument(
        '-n', '--node', default=None,
        help='Recorder node name, e.g. /rosbag2_dynamic_recorder. Only needed when more than '
             'one recorder is running; otherwise the single one on the graph is used')
    parser.add_argument(
        '--timeout', type=float, default=DEFAULT_TIMEOUT, metavar='N',
        help='Seconds to wait for the service and for its reply (default: %(default)s)')
    parser.add_argument(
        '--spin-time', type=float, default=1.0, metavar='N',
        help='Seconds to spin for graph discovery before looking for the recorder '
             '(default: %(default)s)')


def add_pattern_arguments(parser, candidates):
    """The -e/--exclude-regex pair shared by `add`, `remove` and `set`.

    Flag names follow stock `ros2 bag record -e`, since anyone reaching for a pattern here has
    almost certainly used that one.
    """
    parser.add_argument(
        '-e', '--regex', default='', metavar='PATTERN',
        help='Select topics by regular expression instead of, or as well as, naming them. '
             'Matched against {}. A search rather than a full match, so "camera" finds '
             '"/robot/camera/image"; anchor it (^/robot/) when you mean a prefix'.format(
                 candidates))
    parser.add_argument(
        '--exclude-regex', default='', metavar='PATTERN',
        help='Drop topics matching this from the selection. Applied after names and --regex are '
             'combined, so it filters explicitly named topics too')


def require_a_selection(args):
    """Refuse a call that names nothing and matches nothing.

    Without this an empty invocation would be a silent no-op reported as success, which is the
    kind of thing a supervisor script cannot notice.
    """
    if not args.topics and not args.regex:
        raise RecorderError('name at least one topic, or select some with --regex')


@contextmanager
def recorder_from_args(args):
    """Yield a Recorder resolved from the command line, or raise RecorderError."""
    with DirectNode(args) as direct:
        node = direct.node
        discovered = recorder_nodes(node.get_service_names_and_types())
        name = choose_recorder(discovered, getattr(args, 'node', None))
        yield Recorder(node, name, discovered, timeout=args.timeout)


def with_recorder(args, body):
    """Run `body(recorder)`, turning any RecorderError into a message and a non-zero exit.

    Every verb goes through this so a script driving the CLI can rely on the exit code alone, and
    so failures land on stderr where a supervisor's log will not mix them into the output.
    """
    try:
        with recorder_from_args(args) as recorder:
            return body(recorder)
    except RecorderError as exc:
        print(str(exc), file=sys.stderr)
        return 1


def report(response, success_line=None):
    """Render the `return_code`/`error_string` pair shared by most of the services."""
    if response.return_code != 0:
        detail = response.error_string or 'return_code {}'.format(response.return_code)
        print(detail, file=sys.stderr)
        return 1
    if success_line:
        print(success_line)
    return 0


def parse_topic_args(values):
    """Split `TOPIC[:TYPE]` arguments into the parallel arrays the services take.

    An explicit type is not decoration: the recorder only consults the graph when the type is
    empty, so naming it is the way to record a topic whose publisher has not started yet.
    """
    topics = []
    types = []
    for value in values:
        topic, separator, type_name = value.partition(':')
        if not topic:
            raise RecorderError("'{}' has no topic name".format(value))
        if separator and not type_name:
            raise RecorderError("'{}' has a trailing ':' but no type after it".format(value))
        topics.append(topic)
        types.append(type_name)
    # The services require topic_types to be empty or exactly as long as topics, so an all-empty
    # list is sent as empty rather than as a list of empty strings.
    return topics, ([] if not any(types) else types)
