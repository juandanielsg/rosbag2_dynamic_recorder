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

The rules with actual judgement in them -- how a recorder is identified, how `TOPIC[:TYPE]` is
read, what counts as unknown rather than zero, that an empty selection is refused -- live in
dynrec and are tested there. This module is what is specific to a command line: argument
definitions, the exit code, and stderr.
"""

import sys

from dynrec.errors import AmbiguousRecorder, CallFailed, DynrecError
from dynrec.results import TopicChange

from rosbag2_dynamic_recorder_cli.format import format_topic_group


def ambiguous_message(exc):
    """dynrec refuses to guess between recorders; a command line says which flag resolves it."""
    return ('several recorders are running; pick one with --node:\n  '
            + '\n  '.join(exc.recorders))


def add_recorder_arguments(parser):
    """Arguments every verb takes."""
    # Imported here so this module, and the tests of its ROS-free parts, need no rclpy. The client
    # is the only thing here that does, and only a verb that actually talks to a recorder does.
    from dynrec.client import DEFAULT_TIMEOUT
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


def report_change(call, *groups, empty=None):
    """Run `call()` and print the :class:`TopicChange` it returns, one labelled group per line.

    `groups` are (label, field) pairs; empty groups are omitted so a change stays one line per
    thing that happened, and `empty` is printed when nothing did. Printed even when the call is
    refused: a total failure -- every topic unavailable, a profile that does not exist -- raises
    CallFailed, but the response behind it still says which topics were touched, and printing
    those before the error reaches `with_recorder` is what keeps stdout useful on a failure.
    """
    try:
        change = call()
    except CallFailed as exc:
        _print_change(TopicChange.from_msg(exc.response), groups, empty)
        raise
    _print_change(change, groups, empty)


def _print_change(change, groups, empty):
    lines = [
        line for label, field in groups
        for line in format_topic_group(label, getattr(change, field))
    ]
    if lines or empty:
        print('\n'.join(lines) or empty)


def with_recorder(args, body):
    """Run `body(recorder)` against the recorder the command line names, and return the exit code.

    Every verb goes through this so a script driving the CLI can rely on the exit code alone, and
    so failures land on stderr where a supervisor's log will not mix them into the output. The
    recorder owns its own context and node; the verb then speaks to it through the same typed
    methods a script would, so the CLI and the library cannot disagree about how a request is
    built or a refusal is reported.
    """
    from dynrec.client import Recorder
    try:
        with Recorder(
            recorder=getattr(args, 'node', None),
            timeout=args.timeout,
            discovery_timeout=args.spin_time,
        ) as recorder:
            body(recorder)
        return 0
    except AmbiguousRecorder as exc:
        print(ambiguous_message(exc), file=sys.stderr)
        return 1
    except DynrecError as exc:
        print(str(exc), file=sys.stderr)
        return 1
