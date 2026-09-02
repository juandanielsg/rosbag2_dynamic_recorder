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

"""Recorder plus browser UI, in one command.

    ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag

Then open http://localhost:8088. This is the intended entry point for anyone who does not want to
learn service call syntax to record a bag.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARGUMENTS = [
    ('uri', 'dynamic_bag', 'Output bag path.'),
    ('topics', '[]', 'Topics to record at startup, as a YAML list. Can be changed in the UI.'),
    ('storage_id', 'mcap', 'Storage plugin.'),
    ('start_paused', 'false', 'Start with recording paused.'),
    ('snapshot_mode', 'false', 'Buffer in memory and only write on request.'),
    ('port', '8088', 'Port for the browser UI.'),
    ('bind', '0.0.0.0', 'Address the UI listens on. Use 127.0.0.1 to allow local access only.'),
]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, default, desc in ARGUMENTS]
        + [
            Node(
                package='rosbag2_dynamic_recorder',
                executable='dynamic_recorder',
                name='rosbag2_dynamic_recorder',
                output='screen',
                parameters=[{
                    'uri': LaunchConfiguration('uri'),
                    'storage_id': LaunchConfiguration('storage_id'),
                    # Parsed so a YAML list on the command line arrives as a string array
                    # rather than as one long string.
                    'topics': PythonExpression(LaunchConfiguration('topics')),
                    'start_paused': LaunchConfiguration('start_paused'),
                    'snapshot_mode': LaunchConfiguration('snapshot_mode'),
                }],
            ),
            Node(
                package='rosbag2_dynamic_recorder_ui',
                executable='ui_node',
                name='rosbag2_dynamic_recorder_ui',
                output='screen',
                parameters=[{
                    'port': LaunchConfiguration('port'),
                    'bind': LaunchConfiguration('bind'),
                    'recorder_node': '/rosbag2_dynamic_recorder',
                }],
            ),
        ]
    )
