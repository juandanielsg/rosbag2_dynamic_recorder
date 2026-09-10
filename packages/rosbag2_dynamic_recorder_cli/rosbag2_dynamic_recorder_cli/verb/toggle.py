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

from rosbag2_dynamic_recorder_cli.api import add_recorder_arguments, with_recorder
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_interfaces.srv import IsPaused, TogglePaused


class ToggleVerb(VerbExtension):
    """Flip between paused and recording."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)

    def main(self, *, args):
        def body(recorder):
            recorder.call(TogglePaused, 'toggle_paused')
            # TogglePaused has an empty response, and a toggle whose result you have to guess is
            # not much use, so ask. The second round trip is the price of being able to print
            # what actually happened rather than what was requested.
            paused = recorder.call(IsPaused, 'is_paused').paused
            print('paused' if paused else 'recording')
            return 0

        return with_recorder(args, body)
