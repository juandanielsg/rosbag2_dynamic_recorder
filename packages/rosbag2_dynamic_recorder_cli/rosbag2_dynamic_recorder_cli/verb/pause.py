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


class PauseVerb(RecorderVerb):
    """Stop writing messages, without unsubscribing from anything."""

    def run(self, recorder, args):
        # Pause has an empty response, so the only failure this can report is not reaching the
        # service at all -- which recorder.pause() raises for.
        recorder.pause()
        print('paused; subscriptions stay up, arriving messages are discarded')
