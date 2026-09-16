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

from rosbag2_dynamic_recorder_cli.verb import RecorderVerb


class ProfilesVerb(RecorderVerb):
    """List the configured profiles and mark the active one."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        parser.add_argument(
            '--names', action='store_true',
            help='Print only the profile names, one per line, for scripting')

    def run(self, recorder, args):
        profiles = recorder.profiles()
        if args.names:
            for name in profiles.names:
                print(name)
            return
        if not profiles.profiles:
            print('no profiles configured (declare them with the profile_names parameter)')
            return
        for name, topics in profiles.profiles.items():
            # The recorder derives active_profile from the live topic set on every publication
            # rather than remembering it, so this marker cannot go stale and claim a profile
            # that a manual topic change has since broken.
            marker = '*' if name == profiles.active else ' '
            print('{} {}'.format(marker, name))
            for topic in topics:
                print('    ' + topic)
        if not profiles.active:
            print('\nNo profile matches what is being recorded right now.')
