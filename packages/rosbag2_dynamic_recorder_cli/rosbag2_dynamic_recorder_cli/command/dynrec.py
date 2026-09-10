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

from ros2cli.command import CommandExtension, add_subparsers_on_demand


class DynrecCommand(CommandExtension):
    """Drive a running rosbag2_dynamic_recorder."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'rosbag2_dynamic_recorder_cli.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            self._subparser.print_help()
            return 0
        return getattr(args, '_verb').main(args=args)
