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

from rosbag2_dynamic_recorder_cli.api import add_recorder_arguments, with_recorder
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_dynamic_recorder_interfaces.srv import GetProfiles


class ProfilesVerb(VerbExtension):
    """List the configured profiles and mark the active one."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)
        parser.add_argument(
            '--names', action='store_true',
            help='Print only the profile names, one per line, for scripting')

    def main(self, *, args):
        def body(recorder):
            response = recorder.call(GetProfiles, 'get_profiles')
            if args.names:
                for profile in response.profiles:
                    print(profile.name)
                return 0
            if not response.profiles:
                print('no profiles configured (declare them with the profile_names parameter)')
                return 0
            for profile in response.profiles:
                # The recorder derives active_profile from the live topic set on every
                # publication rather than remembering it, so this marker cannot go stale and
                # claim a profile that a manual topic change has since broken.
                marker = '*' if profile.name == response.active_profile else ' '
                print('{} {}'.format(marker, profile.name))
                for topic in profile.topics:
                    print('    ' + topic)
            if not response.active_profile:
                print('\nNo profile matches what is being recorded right now.')
            return 0

        return with_recorder(args, body)
