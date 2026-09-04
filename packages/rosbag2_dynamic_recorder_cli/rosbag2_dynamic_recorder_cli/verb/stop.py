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
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_interfaces.srv import Stop


class StopVerb(VerbExtension):
    """Close the bag. The node keeps running; 'record' opens a new one."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)

    def main(self, *, args):
        def body(recorder):
            response = recorder.call(Stop, 'stop')
            # Any queued schedule is cleared by the recorder here, so a split or resume set up
            # before this cannot fire against whatever bag comes next.
            return report(response, 'stopped; the bag is closed')

        return with_recorder(args, body)
