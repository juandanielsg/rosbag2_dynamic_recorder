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

from rosbag2_dynamic_recorder_cli.api import add_pattern_arguments, report_change
from rosbag2_dynamic_recorder_cli.verb import RecorderVerb

TOPIC_HELP = (
    "Topic to record, e.g. /scan. Append ':<type>' to name the type explicitly "
    '(e.g. /scan:sensor_msgs/msg/LaserScan), which is how you record a topic whose publisher '
    'has not started yet; without it the type is discovered from the graph'
)


class AddVerb(RecorderVerb):
    """Start recording one or more topics, leaving the rest alone."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        # nargs='*', not '+': --regex alone is a complete request.
        parser.add_argument('topics', nargs='*', metavar='TOPIC', help=TOPIC_HELP)
        add_pattern_arguments(parser, 'topic names on the graph')

    def run(self, recorder, args):
        # A partial success -- two topics added, one unavailable -- is reported as success, so the
        # unavailable list is the only thing that says so.
        report_change(
            lambda: recorder.add(args.topics, regex=args.regex, exclude_regex=args.exclude_regex),
            ('now recording', 'subscribed'),
            ('unavailable', 'unavailable'))
