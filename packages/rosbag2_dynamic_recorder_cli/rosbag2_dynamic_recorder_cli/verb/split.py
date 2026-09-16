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

from rosbag2_dynamic_recorder_cli.format import add_schedule_arguments, format_duration
from rosbag2_dynamic_recorder_cli.verb import RecorderVerb


class SplitVerb(RecorderVerb):
    """Roll over to a new bag file, now or at a scheduled time."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        add_schedule_arguments(parser, 'split')

    def run(self, recorder, args):
        at = recorder.split(at=args.at, mode=args.mode, topic=args.topic)
        # Schedules are cleared on stop, so one queued here cannot fire against a later
        # recording -- worth knowing if you queue a split and then stop the bag.
        if at is None:
            print('split')
        else:
            print('split scheduled in {} ({} time)'.format(
                format_duration(at - time.time()), args.mode))
