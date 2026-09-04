# Copyright 2026 juandanielsg
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

"""How the CLI decides which node it is talking to, and what it sends it.

Finding the recorder by service type rather than by node name is what makes `ros2 dynrec status`
work with no arguments, so it is worth pinning: a recorder under any name must be found, and a
node that merely borrows the name must not be.

No ROS graph needed -- these take the graph listing as an argument for exactly that reason.
"""

from types import SimpleNamespace

import pytest

from rosbag2_dynamic_recorder_cli.api import (
    GET_STATUS_TYPE,
    RecorderError,
    choose_recorder,
    parse_topic_args,
    recorder_nodes,
    require_a_selection,
)

OTHER_TYPE = 'rosbag2_interfaces/srv/IsPaused'


def test_recorder_found_under_any_name_or_namespace():
    graph = [
        ('/robot/left_recorder/get_status', [GET_STATUS_TYPE]),
        ('/rosbag2_dynamic_recorder/get_status', [GET_STATUS_TYPE]),
    ]
    assert recorder_nodes(graph) == [
        '/robot/left_recorder',
        '/rosbag2_dynamic_recorder',
    ]


def test_impostor_is_not_a_recorder():
    """A node called get_status of some other type is not ours, whatever it is named."""
    graph = [
        ('/rosbag2_dynamic_recorder/get_status', [OTHER_TYPE]),
        ('/some_node/is_paused', [OTHER_TYPE]),
    ]
    assert recorder_nodes(graph) == []


def test_other_services_on_the_same_node_do_not_duplicate_it():
    graph = [
        ('/rec/get_status', [GET_STATUS_TYPE]),
        ('/rec/pause', ['rosbag2_interfaces/srv/Pause']),
        ('/rec/set_topics', ['rosbag2_dynamic_recorder_interfaces/srv/SetTopics']),
    ]
    assert recorder_nodes(graph) == ['/rec']


def test_the_single_recorder_needs_no_argument():
    assert choose_recorder(['/rec']) == '/rec'


def test_no_recorder_says_so():
    with pytest.raises(RecorderError, match='no rosbag2_dynamic_recorder found'):
        choose_recorder([])


def test_several_recorders_refuses_and_lists_them():
    """Guessing between two recorders could stop the wrong recording, so it does not guess."""
    with pytest.raises(RecorderError) as excinfo:
        choose_recorder(['/rec_a', '/rec_b'])
    message = str(excinfo.value)
    assert '--node' in message
    assert '/rec_a' in message and '/rec_b' in message


def test_a_requested_name_is_trusted_even_if_discovery_missed_it():
    """Discovery can lag. Refusing a name that is about to appear would be worse than trying."""
    assert choose_recorder([], requested='rec') == '/rec'
    assert choose_recorder(['/other'], requested='/rec/') == '/rec'


def test_topics_without_types_send_an_empty_type_array():
    """The services require topic_types to be empty or exactly as long as topics."""
    topics, types = parse_topic_args(['/a', '/b'])
    assert topics == ['/a', '/b']
    assert types == []


def test_a_single_explicit_type_still_fills_the_whole_array():
    topics, types = parse_topic_args(['/a', '/b:sensor_msgs/msg/Imu'])
    assert topics == ['/a', '/b']
    assert types == ['', 'sensor_msgs/msg/Imu']
    assert len(types) == len(topics)


def test_a_trailing_colon_is_rejected_rather_than_read_as_discover():
    """'/a:' looks like an explicit type and is not one; silently discovering would hide a typo."""
    with pytest.raises(RecorderError, match='trailing'):
        parse_topic_args(['/a:'])


def test_a_call_that_selects_nothing_is_refused():
    """An empty invocation would be a silent no-op reported as success.

    That is precisely what a supervisor script cannot notice, and the exit code is the only thing
    it has to go on.
    """
    with pytest.raises(RecorderError, match="at least one topic"):
        require_a_selection(SimpleNamespace(topics=[], regex=""))


def test_a_pattern_alone_is_a_complete_selection():
    """`ros2 dynrec add -e '^/camera/'` names no topics and is still a full request."""
    require_a_selection(SimpleNamespace(topics=[], regex="^/camera/"))
    require_a_selection(SimpleNamespace(topics=["/a"], regex=""))
