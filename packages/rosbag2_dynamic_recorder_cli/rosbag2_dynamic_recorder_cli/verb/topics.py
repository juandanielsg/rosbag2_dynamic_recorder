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


class TopicsVerb(RecorderVerb):
    """List the topics being recorded, one per line."""

    def run(self, recorder, args):
        # Bare names and nothing else, so this pipes into xargs without any parsing. `status` is
        # where the decorated version lives.
        for topic in recorder.topics():
            print(topic)
