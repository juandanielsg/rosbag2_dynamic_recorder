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
from rosbag2_dynamic_recorder_cli.verb.add import TOPIC_HELP


class SetVerb(RecorderVerb):
    """Record exactly these topics and nothing else."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        parser.add_argument(
            'topics', nargs='*', metavar='TOPIC',
            help=TOPIC_HELP + '. Passing none is a valid request that stops recording every '
                              'topic without closing the bag')
        add_pattern_arguments(parser, 'topic names on the graph')

    def run(self, recorder, args):
        # Topics present in both the old and new sets are never torn down, so this is the verb to
        # reach for when switching modes: the topics that carry over keep recording without a gap.
        report_change(
            lambda: recorder.set_topics(
                args.topics, regex=args.regex, exclude_regex=args.exclude_regex),
            ('now recording', 'subscribed'),
            ('no longer recording', 'unsubscribed'),
            ('unavailable', 'unavailable'),
            empty='recording nothing')
