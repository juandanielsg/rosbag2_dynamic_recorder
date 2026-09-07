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

"""Turning what a caller writes into the parallel arrays the services take.

No ROS import: this is string handling, and keeping it that way is what makes the awkward case
below testable in isolation.
"""

from dynrec.errors import InvalidRequest


def parse_topic_specs(values):
    """Split `TOPIC` or `TOPIC:TYPE` items into `(topics, topic_types)`.

    An explicit type is not decoration: the recorder only consults the graph when the type is
    empty, so naming it is the way to record a topic whose publisher has not started yet -- which
    from a script is the common case, since the script often starts the publisher itself.

    A plain string is accepted as well as a list, because `add('/scan')` is what anyone writes
    first and iterating it into fourteen single-character topics would be a baffling way to fail.
    """
    if isinstance(values, str):
        values = [values]
    topics = []
    types = []
    for value in values or ():
        topic, separator, type_name = value.partition(':')
        if not topic:
            raise InvalidRequest("'{}' has no topic name".format(value))
        if separator and not type_name:
            raise InvalidRequest("'{}' has a trailing ':' but no type after it".format(value))
        topics.append(topic)
        types.append(type_name)
    # The services require topic_types to be empty or exactly as long as topics, so an all-empty
    # list is sent as empty rather than as a list of empty strings.
    return topics, ([] if not any(types) else types)


def require_a_selection(topics, regex):
    """Refuse a call that names nothing and matches nothing.

    Without this an empty `add()` would be a silent no-op reported as success, and a supervisor
    script has no way to notice that -- the same reason `ros2 dynrec add` refuses it.
    """
    if not topics and not regex:
        raise InvalidRequest('name at least one topic, or select some with regex=')
