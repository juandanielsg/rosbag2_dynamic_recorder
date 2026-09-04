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

from rosbag2_dynamic_recorder_cli.api import (
    add_pattern_arguments,
    add_recorder_arguments,
    report,
    require_a_selection,
    with_recorder,
)
from rosbag2_dynamic_recorder_cli.format import format_topic_group
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_dynamic_recorder_interfaces.srv import UnsubscribeTopics


class RemoveVerb(VerbExtension):
    """Stop recording one or more topics, leaving the rest alone."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)
        parser.add_argument(
            'topics', nargs='*', metavar='TOPIC',
            help='Topic to stop recording. Messages already written are kept; the bag simply '
                 'stops gaining new ones')
        add_pattern_arguments(parser, 'the topics currently being recorded, not the graph')

    def main(self, *, args):
        def body(recorder):
            require_a_selection(args)
            request = UnsubscribeTopics.Request()
            request.topics = list(args.topics)
            request.regex = args.regex
            request.exclude_regex = args.exclude_regex
            response = recorder.call(UnsubscribeTopics, 'unsubscribe_topics', request)
            lines = format_topic_group('no longer recording', response.unsubscribed_topics)
            lines += format_topic_group('was not being recorded', response.not_subscribed_topics)
            if lines:
                print('\n'.join(lines))
            return report(response)

        return with_recorder(args, body)
