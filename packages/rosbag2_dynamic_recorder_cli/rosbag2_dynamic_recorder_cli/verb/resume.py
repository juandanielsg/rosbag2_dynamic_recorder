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

from rosbag2_dynamic_recorder_cli.api import add_schedule_arguments
from rosbag2_dynamic_recorder_cli.format import format_scheduled
from rosbag2_dynamic_recorder_cli.verb import RecorderVerb


class ResumeVerb(RecorderVerb):
    """Start writing messages again, now or at a scheduled time."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        add_schedule_arguments(parser, 'resume')

    def run(self, recorder, args):
        at = recorder.resume(at=args.at, mode=args.mode, topic=args.topic)
        print(format_scheduled('recording', 'resume', at, args.mode))
