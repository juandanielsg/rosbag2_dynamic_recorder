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

"""How the library decides which node it is talking to, and what it will build a request from.

Finding the recorder by service type rather than by node name is what makes `Recorder()` with no
arguments work, so it is worth pinning in both directions: a recorder under any name must be
found, and a node that merely borrows the name must not be.

No ROS graph needed -- these take the graph listing as an argument for exactly that reason.
"""

import pytest

from dynrec.discovery import GET_STATUS_TYPE, choose_recorder, recorder_nodes
from dynrec.errors import AmbiguousRecorder, InvalidRequest, RecorderNotFound
from dynrec.topics import parse_topic_specs, require_a_selection

OTHER_TYPE = 'rosbag2_interfaces/srv/IsPaused'


def test_recorder_found_under_any_name_or_namespace():
    graph = [
        ('/robot/left_recorder/get_status', [GET_STATUS_TYPE]),
        ('/rosbag2_dynamic_recorder/get_status', [GET_STATUS_TYPE]),
    ]
    assert recorder_nodes(graph) == ['/robot/left_recorder', '/rosbag2_dynamic_recorder']


def test_impostor_is_not_a_recorder():
    """A get_status of some other type is not ours, whatever the node is called."""
    graph = [
        ('/rosbag2_dynamic_recorder/get_status', [OTHER_TYPE]),
        ('/some_node/is_paused', [OTHER_TYPE]),
    ]
    assert recorder_nodes(graph) == []


def test_the_only_recorder_needs_no_naming():
    assert choose_recorder(['/rosbag2_dynamic_recorder']) == '/rosbag2_dynamic_recorder'


def test_two_recorders_refuse_to_be_guessed_between():
    """And the exception carries the names, so the caller can choose without re-discovering."""
    with pytest.raises(AmbiguousRecorder) as excinfo:
        choose_recorder(['/left', '/right'])
    assert excinfo.value.recorders == ['/left', '/right']


def test_no_recorder_says_what_to_check():
    with pytest.raises(RecorderNotFound, match='ROS_DOMAIN_ID'):
        choose_recorder([])


def test_a_named_recorder_is_trusted_even_when_undiscovered():
    """Discovery lags. Timing out on the first call says more than refusing a name outright."""
    assert choose_recorder([], 'not_yet_up') == '/not_yet_up'


def test_a_named_recorder_is_normalised_to_a_fully_qualified_name():
    assert choose_recorder(['/rosbag2_dynamic_recorder'], 'rosbag2_dynamic_recorder') == (
        '/rosbag2_dynamic_recorder')


def test_topics_without_types_send_no_types_at_all():
    """The services require topic_types empty or exactly as long as topics."""
    assert parse_topic_specs(['/scan', '/odom']) == (['/scan', '/odom'], [])


def test_an_explicit_type_is_carried_through_by_index():
    topics, types = parse_topic_specs(['/scan:sensor_msgs/msg/LaserScan', '/odom'])
    assert topics == ['/scan', '/odom']
    assert types == ['sensor_msgs/msg/LaserScan', '']


def test_a_bare_string_is_one_topic_not_fourteen():
    """`add('/scan')` is what anyone writes first, and iterating it would fail bafflingly."""
    assert parse_topic_specs('/scan') == (['/scan'], [])


def test_a_trailing_colon_is_refused_rather_than_read_as_no_type():
    with pytest.raises(InvalidRequest, match='trailing'):
        parse_topic_specs(['/scan:'])


def test_a_selection_that_names_and_matches_nothing_is_refused():
    """A silent no-op reported as success is the one thing a supervisor script cannot notice."""
    with pytest.raises(InvalidRequest):
        require_a_selection([], '')
    require_a_selection([], 'camera')
    require_a_selection(['/scan'], '')
