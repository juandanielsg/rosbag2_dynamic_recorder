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

import json

from rosbag2_dynamic_recorder_cli.format import format_status
from rosbag2_dynamic_recorder_cli.verb import RecorderVerb


class StatusVerb(RecorderVerb):
    """Show what the recorder is doing."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        parser.add_argument(
            '--json', action='store_true',
            help='Emit the status as JSON. Fields the recorder cannot vouch for are null '
                 'rather than zero')

    def run(self, recorder, args):
        # ~/get_status rather than the latched ~/status topic: this is the one-shot call that
        # service exists for, and one round trip beats waiting out a publication period.
        status = recorder.status()
        if args.json:
            print(json.dumps(status.as_dict(), indent=2))
        else:
            print('\n'.join(format_status(status)))
