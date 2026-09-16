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

from rosbag2_dynamic_recorder_cli.api import report_change
from rosbag2_dynamic_recorder_cli.verb import RecorderVerb


class ProfileVerb(RecorderVerb):
    """Apply a named profile: record exactly the topics it lists."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        parser.add_argument(
            'name', metavar='NAME',
            help='Profile name, as declared in the profile_names parameter. '
                 "'ros2 dynrec profiles' lists them")

    def run(self, recorder, args):
        # Routed through the same code as set_topics inside the recorder, so switching profiles
        # inherits the guarantee: topics common to both are never torn down.
        report_change(
            lambda: recorder.profile(args.name),
            ('now recording', 'subscribed'),
            ('no longer recording', 'unsubscribed'),
            ('unavailable', 'unavailable'))
