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


class StopVerb(RecorderVerb):
    """Close the bag. The node keeps running; 'record' opens a new one."""

    def run(self, recorder, args):
        # dynrec raises CallFailed when the recorder refuses (already stopped), and any queued
        # schedule is cleared by the recorder here, so a split or resume set up before cannot
        # fire against whatever bag comes next.
        recorder.stop()
        print('stopped; the bag is closed')
