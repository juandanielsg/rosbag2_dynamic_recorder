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

from rosbag2_dynamic_recorder_cli.api import add_schedule_arguments
from rosbag2_dynamic_recorder_cli.format import format_scheduled
from rosbag2_dynamic_recorder_cli.verb import RecorderVerb


class RecordVerb(RecorderVerb):
    """Open a new bag after a stop, restoring the previous topic selection."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        parser.add_argument(
            '--uri', default='', metavar='PATH',
            help='Where to create the new bag directory. Default: the uri the recorder was '
                 'started with')
        add_schedule_arguments(parser, 'start', modes=False)

    def run(self, recorder, args):
        at = recorder.record(uri=args.uri, at=args.at)
        print(format_scheduled('recording', 'recording', at))
