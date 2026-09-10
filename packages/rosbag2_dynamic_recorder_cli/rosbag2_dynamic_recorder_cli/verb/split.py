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

import time

from rosbag2_dynamic_recorder_cli.api import add_recorder_arguments, report, with_recorder
from rosbag2_dynamic_recorder_cli.format import (
    add_schedule_arguments,
    apply_schedule,
    format_duration,
)
from rosbag2_dynamic_recorder_cli.verb import VerbExtension

from rosbag2_interfaces.srv import SplitBagfile


class SplitVerb(VerbExtension):
    """Roll over to a new bag file, now or at a scheduled time."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)
        add_schedule_arguments(parser, 'split')

    def main(self, *, args):
        def body(recorder):
            request = SplitBagfile.Request()
            at = apply_schedule(request, args, 'split_time', 'split_mode')
            response = recorder.call(SplitBagfile, 'split_bagfile', request)
            if at is None:
                return report(response, 'split')
            # Schedules are cleared on stop, so one queued here cannot fire against a later
            # recording -- worth knowing if you queue a split and then stop the bag.
            return report(
                response,
                'split scheduled in {} ({} time)'.format(
                    format_duration(at - time.time()), args.mode))

        return with_recorder(args, body)
