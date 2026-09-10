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

from rosbag2_dynamic_recorder_cli.api import (
    add_pattern_arguments,
    add_recorder_arguments,
    parse_topic_args,
    report,
    require_a_selection,
    with_recorder,
)
from rosbag2_dynamic_recorder_cli.format import format_topic_group
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_dynamic_recorder_interfaces.srv import SubscribeTopics

TOPIC_HELP = (
    "Topic to record, e.g. /scan. Append ':<type>' to name the type explicitly "
    '(e.g. /scan:sensor_msgs/msg/LaserScan), which is how you record a topic whose publisher '
    'has not started yet; without it the type is discovered from the graph'
)


class AddVerb(VerbExtension):
    """Start recording one or more topics, leaving the rest alone."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)
        # nargs='*', not '+': --regex alone is a complete request.
        parser.add_argument('topics', nargs='*', metavar='TOPIC', help=TOPIC_HELP)
        add_pattern_arguments(parser, 'topic names on the graph')

    def main(self, *, args):
        def body(recorder):
            require_a_selection(args)
            request = SubscribeTopics.Request()
            request.topics, request.topic_types = parse_topic_args(args.topics)
            request.regex = args.regex
            request.exclude_regex = args.exclude_regex
            response = recorder.call(SubscribeTopics, 'subscribe_topics', request)
            # Printed before the return code is judged, because a partial success -- two topics
            # added, one unavailable -- reports return_code 0 and the unavailable list is then
            # the only thing that says so.
            lines = format_topic_group('now recording', response.subscribed_topics)
            lines += format_topic_group('unavailable', response.unavailable_topics)
            if lines:
                print('\n'.join(lines))
            return report(response)

        return with_recorder(args, body)
