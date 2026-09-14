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

"""The service and message types the recorder offers, resolved for the installed rosbag2.

The recorder reuses `rosbag2_interfaces` types wherever it can, so that a client written against
`ros2 bag record` can drive it. But those definitions moved between distros: rosbag2 0.34 (Rolling)
gave `Record`, `Resume` and `SplitBagfile` a scheduled form, gave `Stop` a return code, and added
`MessagesLostEvent`; Jazzy (0.26) and Kilted (0.32) have the earlier shapes, and Kilted has no
`Record` at all. `rosbag2_dynamic_recorder_interfaces` carries a field-identical copy of each,
and the recorder's CMake offers the stock type where the installed one matches and the copy
where it does not.

This module makes the same decision, from the same evidence, on the client side: a stock type is
used when it has the field that distinguishes the 0.34 shape, and the copy otherwise. Import the
names from here rather than from `rosbag2_interfaces.srv` directly, and they agree with whatever
recorder was built against the same install.
"""

import rosbag2_dynamic_recorder_interfaces.msg as _own_msg
import rosbag2_dynamic_recorder_interfaces.srv as _own_srv
import rosbag2_interfaces.msg as _stock_msg
import rosbag2_interfaces.srv as _stock_srv


def _resolve(stock_module, own_module, name, part, field):
    """Return the stock type when `part` (`Request`/`Response`/`None`) has `field`, else ours."""
    stock = getattr(stock_module, name, None)
    if stock is None:
        return getattr(own_module, name)
    subject = stock if part is None else getattr(stock, part)
    if field is None or hasattr(subject, field):
        return stock
    return getattr(own_module, name)


# Identical on every distro this project builds on.
IsPaused = _stock_srv.IsPaused
Pause = _stock_srv.Pause
Snapshot = _stock_srv.Snapshot
TogglePaused = _stock_srv.TogglePaused

# Reshaped in rosbag2 0.34; the field named is the one that tells the shapes apart.
Record = _resolve(_stock_srv, _own_srv, 'Record', 'Request', 'start_time')
Resume = _resolve(_stock_srv, _own_srv, 'Resume', 'Request', 'resume_mode')
SplitBagfile = _resolve(_stock_srv, _own_srv, 'SplitBagfile', 'Request', 'split_mode')
Stop = _resolve(_stock_srv, _own_srv, 'Stop', 'Response', 'return_code')
MessagesLostEvent = _resolve(_stock_msg, _own_msg, 'MessagesLostEvent', None, None)


def type_name(interface):
    """The `package/kind/Name` string for one of the types above, as `ros2 service call` wants."""
    module = interface.__module__.split('.')  # e.g. rosbag2_interfaces.srv._record
    return '/'.join((module[0], module[1], interface.__name__))


__all__ = [
    'IsPaused',
    'MessagesLostEvent',
    'Pause',
    'Record',
    'Resume',
    'Snapshot',
    'SplitBagfile',
    'Stop',
    'TogglePaused',
    'type_name',
]
