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

"""What the CLI adds on top of dynrec's discovery.

Finding the recorder, reading `TOPIC[:TYPE]`, and refusing an empty selection are dynrec's rules
and are tested there. What is pinned here is the command-line wording layered on them: the flag an
ambiguity names.

No ROS graph needed.
"""

from dynrec.errors import AmbiguousRecorder
from rosbag2_dynamic_recorder_cli.api import ambiguous_message


def test_several_recorders_refuses_and_names_the_flag():
    """Guessing between two recorders could stop the wrong recording, so the CLI does not guess --
    and the message says which flag resolves it, which the library cannot know."""
    message = ambiguous_message(AmbiguousRecorder('several', ['/rec_a', '/rec_b']))
    assert '--node' in message
    assert '/rec_a' in message and '/rec_b' in message

