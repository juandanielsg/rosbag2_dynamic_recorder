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


class RemoveVerb(RecorderVerb):
    """Stop recording one or more topics, leaving the rest alone."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        parser.add_argument(
            'topics', nargs='*', metavar='TOPIC',
            help="Topic to stop recording, e.g. /scan. Messages already written are kept; the "
                 "bag simply stops gaining new ones. As in 'add', a ':<type>' suffix is accepted "
                 'and ignored -- only the topic name identifies what to drop')
        add_pattern_arguments(parser, 'the topics currently being recorded, not the graph')

    def run(self, recorder, args):
        # Types are stripped by dynrec before the request is sent: dropping is keyed on the topic
        # name, so a ':<type>' suffix would otherwise match nothing.
        report_change(
            lambda: recorder.remove(
                args.topics, regex=args.regex, exclude_regex=args.exclude_regex),
            ('no longer recording', 'unsubscribed'),
            ('was not being recorded', 'not_subscribed'))
