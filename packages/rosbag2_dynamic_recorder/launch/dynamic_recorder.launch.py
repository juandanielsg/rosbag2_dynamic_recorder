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

"""Launch the dynamic recorder.

Example:
    ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \\
        uri:=/tmp/mybag topics:="['/scan','/odom']"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARGUMENTS = [
    ('uri', 'dynamic_bag', 'Output bag path.'),
    ('storage_id', 'mcap', 'Storage plugin.'),
    ('serialization_format', 'cdr', 'Message serialization format.'),
    ('topics', '[]', 'Topics to subscribe at startup, as a YAML list.'),
    ('start_paused', 'false', 'Start with recording paused.'),
    ('snapshot_mode', 'false', 'Buffer in memory and only write on ~/snapshot.'),
    ('max_cache_size', '104857600', 'Writer cache size in bytes. Required by snapshot_mode.'),
    ('record_subscription_events', 'true',
     'Record SubscriptionChangeEvent into the bag, so sparse channels explain themselves.'),
    ('messages_lost_report_period', '5.0',
     'Seconds between MessagesLostEvent publications. 0 disables reporting.'),
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
                    'serialization_format': LaunchConfiguration('serialization_format'),
                    # Parsed so a YAML list on the command line arrives as a string array
                    # rather than as one long string.
                    'topics': PythonExpression(LaunchConfiguration('topics')),
                    'start_paused': LaunchConfiguration('start_paused'),
                    'snapshot_mode': LaunchConfiguration('snapshot_mode'),
                    'max_cache_size': LaunchConfiguration('max_cache_size'),
                    'record_subscription_events':
                        LaunchConfiguration('record_subscription_events'),
                    'messages_lost_report_period':
                        LaunchConfiguration('messages_lost_report_period'),
                }],
            ),
        ]
    )
