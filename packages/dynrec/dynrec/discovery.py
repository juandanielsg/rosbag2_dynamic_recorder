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

"""Finding the recorder on the graph, and deciding which one is meant.

Takes a graph listing as an argument rather than fetching one, so the rule can be tested without
a ROS graph -- the same split `ros2 dynrec` made, and for the same reason: this is the part with
judgement in it.
"""

from dynrec.errors import AmbiguousRecorder, RecorderNotFound

#: The service every recorder offers, used to identify one on the graph.
GET_STATUS_TYPE = 'rosbag2_dynamic_recorder_interfaces/srv/GetStatus'
GET_STATUS_SUFFIX = '/get_status'


def recorder_nodes(service_names_and_types):
    """Fully qualified names of the recorders visible on the graph, sorted.

    A recorder is identified by offering `~/get_status` of our own type, not by node name. A
    recorder launched under a different name or pushed into a namespace is therefore still found,
    and an unrelated node that happens to be called `rosbag2_dynamic_recorder` is not.
    """
    found = set()
    for name, types in service_names_and_types:
        if name.endswith(GET_STATUS_SUFFIX) and GET_STATUS_TYPE in types:
            found.add(name[:-len(GET_STATUS_SUFFIX)])
    return sorted(found)


def choose_recorder(discovered, requested=None):
    """Pick the recorder to talk to, or raise explaining why that is not possible.

    A requested name is trusted even when discovery did not turn it up: graph discovery can lag,
    and refusing a name that is about to appear would be worse than trying it and timing out.
    """
    if requested:
        return '/' + requested.strip('/')
    if not discovered:
        raise RecorderNotFound(
            'no rosbag2_dynamic_recorder found on the graph. '
            'Is one running, and is ROS_DOMAIN_ID the same for both?')
    if len(discovered) > 1:
        raise AmbiguousRecorder(
            'several recorders are running; name the one you mean: '
            + ', '.join(discovered),
            discovered)
    return discovered[0]
