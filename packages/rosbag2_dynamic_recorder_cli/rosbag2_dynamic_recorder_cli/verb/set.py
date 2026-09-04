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
    parse_topic_args,
    report,
    with_recorder,
)
from rosbag2_dynamic_recorder_cli.format import format_topic_group
from rosbag2_dynamic_recorder_cli.verb import VerbExtension
from rosbag2_dynamic_recorder_cli.verb.add import TOPIC_HELP

from rosbag2_dynamic_recorder_interfaces.srv import SetTopics


class SetVerb(VerbExtension):
    """Record exactly these topics and nothing else."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)
        parser.add_argument(
            'topics', nargs='*', metavar='TOPIC',
            help=TOPIC_HELP + '. Passing none is a valid request that stops recording every '
                              'topic without closing the bag')
        add_pattern_arguments(parser, 'topic names on the graph')

    def main(self, *, args):
        def body(recorder):
            request = SetTopics.Request()
            request.topics, request.topic_types = parse_topic_args(args.topics)
            request.regex = args.regex
            request.exclude_regex = args.exclude_regex
            # Topics present in both the old and new sets are never torn down, so this is the
            # verb to reach for when switching modes: the topics that carry over keep recording
            # without a gap.
            response = recorder.call(SetTopics, 'set_topics', request)
            lines = format_topic_group('now recording', response.subscribed_topics)
            lines += format_topic_group('no longer recording', response.unsubscribed_topics)
            lines += format_topic_group('unavailable', response.unavailable_topics)
            print('\n'.join(lines) if lines else 'recording nothing')
            return report(response)

        return with_recorder(args, body)
