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

import time

from rosbag2_dynamic_recorder_cli.api import add_recorder_arguments, report, with_recorder
from rosbag2_dynamic_recorder_cli.format import format_duration, parse_time
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_interfaces.srv import Record


class RecordVerb(VerbExtension):
    """Open a new bag after a stop, restoring the previous topic selection."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)
        parser.add_argument(
            '--uri', default='', metavar='PATH',
            help='Where to create the new bag directory. Default: the uri the recorder was '
                 'started with')
        parser.add_argument(
            '--at', metavar='TIME', default=None,
            help='Start recording at a future time instead of now. Accepts +30s, 14:05, '
                 '2026-09-03T14:05, or epoch seconds. Compared against the node clock -- Record '
                 'takes no mode field, so it is node time by definition and fires on a timer '
                 'whether or not any messages are arriving')

    def main(self, *, args):
        def body(recorder):
            request = Record.Request()
            request.uri = args.uri
            at = None
            if args.at is not None:
                seconds, nanoseconds = parse_time(args.at)
                request.start_time.sec = seconds
                request.start_time.nanosec = nanoseconds
                at = seconds + nanoseconds / 1e9
            response = recorder.call(Record, 'record', request)
            if at is None:
                return report(response, 'recording')
            return report(
                response,
                'recording scheduled in {}'.format(format_duration(at - time.time())))

        return with_recorder(args, body)
