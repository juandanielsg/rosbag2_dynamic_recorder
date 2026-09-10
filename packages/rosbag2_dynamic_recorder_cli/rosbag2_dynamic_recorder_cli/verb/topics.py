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

from rosbag2_dynamic_recorder_interfaces.srv import GetSubscribedTopics


class TopicsVerb(VerbExtension):
    """List the topics being recorded, one per line."""

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)

    def main(self, *, args):
        def body(recorder):
            # Bare names and nothing else, so this pipes into xargs without any parsing. `status`
            # is where the decorated version lives.
            response = recorder.call(GetSubscribedTopics, 'get_subscribed_topics')
            for topic in response.topics:
                print(topic)
            return 0

        return with_recorder(args, body)
