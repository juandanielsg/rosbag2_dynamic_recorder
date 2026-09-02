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

    ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \\
        uri:=/tmp/mybag topics:="['/scan','/odom']"

Starting with no topics is fine and normal -- add them later over the services or the UI.
"""

import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    ('uri', 'dynamic_bag', 'Output bag path.'),
    ('storage_id', 'mcap', 'Storage plugin.'),
    ('serialization_format', 'cdr', 'Message serialization format.'),
    ('topics', '[]', 'Topics to record at startup, as a YAML list. May be empty.'),
    ('start_paused', 'false', 'Start with recording paused.'),
    ('snapshot_mode', 'false', 'Buffer in memory and only write on ~/snapshot.'),
    ('max_cache_size', '104857600', 'Writer cache in bytes. Required by snapshot_mode.'),
    ('record_subscription_events', 'true',
     'Record SubscriptionChangeEvent into the bag, so sparse channels explain themselves.'),
    ('messages_lost_report_period', '5.0',
     'Seconds between MessagesLostEvent publications. 0 disables reporting.'),
    ('params_file', '', 'Optional YAML of extra parameters, e.g. recording profiles.'),
]


def topic_list(context):
    """Parse the `topics` argument into a real list.

    ast.literal_eval rather than eval: this is a command-line string and there is no reason to
    execute it.
    """
    raw = LaunchConfiguration('topics').perform(context).strip()
    if not raw:
        return []
    try:
        value = ast.literal_eval(raw)
    except (ValueError, SyntaxError) as exc:
        raise RuntimeError(f"could not parse topics:={raw!r} as a list: {exc}") from exc
    if isinstance(value, str):
        value = [value]
    return [str(v) for v in value]


def recorder_parameters(context):
    """Build the parameter dict, omitting `topics` when empty.

    An empty list cannot survive the round trip through a launch parameters file -- it arrives at
    rcl as "contains no value" and the node aborts. Since the node already defaults to recording
    nothing, the fix is simply not to pass the key.
    """
    params = {
        'uri': LaunchConfiguration('uri').perform(context),
        'storage_id': LaunchConfiguration('storage_id').perform(context),
        'serialization_format': LaunchConfiguration('serialization_format').perform(context),
        'start_paused': LaunchConfiguration('start_paused').perform(context) == 'true',
        'snapshot_mode': LaunchConfiguration('snapshot_mode').perform(context) == 'true',
        'max_cache_size': int(LaunchConfiguration('max_cache_size').perform(context)),
        'record_subscription_events':
            LaunchConfiguration('record_subscription_events').perform(context) == 'true',
        'messages_lost_report_period':
            float(LaunchConfiguration('messages_lost_report_period').perform(context)),
    }
    topics = topic_list(context)
    if topics:
        params['topics'] = topics
    return params


def _setup(context, *_args, **_kwargs):
    parameters = [recorder_parameters(context)]
    params_file = LaunchConfiguration('params_file').perform(context).strip()
    if params_file:
        parameters.append(params_file)
    return [
        Node(
            package='rosbag2_dynamic_recorder',
            executable='dynamic_recorder',
            name='rosbag2_dynamic_recorder',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, default, desc in ARGUMENTS]
        + [OpaqueFunction(function=_setup)]
    )
