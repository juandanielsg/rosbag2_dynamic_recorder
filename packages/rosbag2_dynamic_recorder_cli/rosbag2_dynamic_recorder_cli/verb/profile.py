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

from rosbag2_dynamic_recorder_cli.api import add_recorder_arguments, report, with_recorder
from rosbag2_dynamic_recorder_cli.format import format_topic_group
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_dynamic_recorder_interfaces.srv import SetProfile


class ProfileVerb(VerbExtension):
    """Apply a named profile: record exactly the topics it lists."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)
        parser.add_argument(
            'name', metavar='NAME',
            help='Profile name, as declared in the profile_names parameter. '
                 "'ros2 dynrec profiles' lists them")

    def main(self, *, args):
        def body(recorder):
            request = SetProfile.Request()
            request.name = args.name
            # Routed through the same code as set_topics inside the recorder, so switching
            # profiles inherits the guarantee: topics common to both are never torn down.
            response = recorder.call(SetProfile, 'set_profile', request)
            lines = format_topic_group('now recording', response.subscribed_topics)
            lines += format_topic_group('no longer recording', response.unsubscribed_topics)
            lines += format_topic_group('unavailable', response.unavailable_topics)
            if lines:
                print('\n'.join(lines))
            return report(response)

        return with_recorder(args, body)
