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

import sys

from rosbag2_dynamic_recorder_cli.api import add_recorder_arguments, with_recorder
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_interfaces.srv import Snapshot


class SnapshotVerb(VerbExtension):
    """Flush the in-memory buffer to disk. Only does anything in snapshot mode."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)

    def main(self, *, args):
        def body(recorder):
            if recorder.call(Snapshot, 'snapshot').success:
                print('snapshot written')
                return 0
            # Snapshot carries no error string, so the near-certain cause is worth naming here
            # rather than leaving the user with a bare false.
            print(
                'snapshot failed. The recorder logs the reason; the usual one is that it was '
                'not started with snapshot_mode:=true, in which case there is no buffer to '
                'flush because messages are already being written as they arrive.',
                file=sys.stderr)
            return 1

        return with_recorder(args, body)
