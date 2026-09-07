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

"""Drive a running rosbag2_dynamic_recorder from a Python script.

    from dynrec import Recorder

    with Recorder() as rec:
        rec.set_topics(['/scan', '/odom'])
        rec.add(['/camera/image:sensor_msgs/msg/Image'])
        rec.pause()
        rec.resume(at='+30s')
        print(rec.status().subscribed_topics)

`Recorder` and `discover` are resolved on first use rather than at import, so `import dynrec` costs
nothing and, more usefully, works without rclpy present. That is not a micro-optimisation: it keeps
the modules holding the judgement -- `discovery`, `schedule`, `topics`, `results` -- importable and
testable with no ROS installation at all, which is the same split `build_state()` and
`recorder_nodes()` already make in the UI and CLI packages.
"""

from dynrec.errors import (
    AmbiguousRecorder,
    CallFailed,
    CallTimeout,
    DynrecError,
    InvalidRequest,
    RecorderNotFound,
    ServiceUnavailable,
)
from dynrec.results import Event, Profiles, Status, TopicChange
from dynrec.schedule import TIME_MODES

__all__ = [
    'AmbiguousRecorder',
    'CallFailed',
    'CallTimeout',
    'DynrecError',
    'Event',
    'InvalidRequest',
    'Profiles',
    'Recorder',
    'RecorderNotFound',
    'ServiceUnavailable',
    'Status',
    'TIME_MODES',
    'TopicChange',
    'discover',
]


def __getattr__(name):
    """Pull in the rclpy-dependent half only when something actually asks for it."""
    if name in ('Recorder', 'discover'):
        from dynrec import client
        return getattr(client, name)
    raise AttributeError('module {!r} has no attribute {!r}'.format(__name__, name))


def __dir__():
    return sorted(__all__)
