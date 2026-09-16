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

"""Defaults shared by the UI node and its launch file, so neither can drift from the other.

Kept free of imports: the launch file reads these from the launch process, which has no reason
to pull in rclpy or an HTTP server to learn a port number.
"""

#: Where the page is served. Not 8080: too often already taken on a developer's machine.
DEFAULT_PORT = 8088

#: The recorder this UI controls unless told otherwise -- the node name the recorder's own launch
#: file gives it.
DEFAULT_RECORDER_NODE = '/rosbag2_dynamic_recorder'
