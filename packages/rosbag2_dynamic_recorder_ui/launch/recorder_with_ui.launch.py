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

"""Recorder plus browser UI, in one command.

    ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag

Then open http://localhost:8088. This is the intended entry point for anyone who does not want to
learn service call syntax to record a bag: start it with no topics at all and pick them in the
browser.

To offer named profiles, point params_file at a YAML like
rosbag2_dynamic_recorder/config/profiles.example.yaml.
"""

import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    ('uri', 'dynamic_bag', 'Output bag path.'),
    ('topics', '[]', 'Topics to record at startup, as a YAML list. May be empty; pick in the UI.'),
    ('storage_id', 'mcap', 'Storage plugin.'),
    ('serialization_format', 'cdr', 'Message serialization format.'),
    ('start_paused', 'false', 'Start with recording paused.'),
    ('snapshot_mode', 'false', 'Buffer in memory and only write on request.'),
    ('max_cache_size', '104857600', 'Writer cache in bytes. Required by snapshot_mode.'),
    ('record_subscription_events', 'true',
     'Record SubscriptionChangeEvent into the bag, so sparse channels explain themselves.'),
    ('messages_lost_report_period', '5.0',
     'Seconds between MessagesLostEvent publications. 0 disables reporting.'),
    ('params_file', '', 'Optional YAML of extra recorder parameters, e.g. recording profiles.'),
    ('port', '8088', 'Port for the browser UI.'),
    ('bind', '127.0.0.1',
     'Address the UI listens on. Loopback by default because the UI has no authentication; use 0.0.0.0 only on a trusted network.'),
]


def recorder_parameters(context):
    """Build the recorder's parameter dict, omitting `topics` when it is empty.

    An empty list cannot survive the round trip through a launch parameters file: it reaches rcl
    as "contains no value" and the node aborts with InvalidParameterValueException. Since the node
    already defaults to recording nothing, the fix is not to pass the key at all -- and starting
    with no topics is the normal case when they will be picked in the UI.

    Duplicated from the recorder package's own launch file rather than imported: that package is
    ament_cmake and exposes no Python module to import from.
    """
    raw = LaunchConfiguration('topics').perform(context).strip()
    topics = []
    if raw:
        try:
            # literal_eval, not eval: this is a command-line string with no reason to execute it.
            value = ast.literal_eval(raw)
        except (ValueError, SyntaxError) as exc:
            raise RuntimeError(f"could not parse topics:={raw!r} as a list: {exc}") from exc
        topics = [str(v) for v in ([value] if isinstance(value, str) else value)]

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
        Node(
            package='rosbag2_dynamic_recorder_ui',
            executable='ui_node',
            name='rosbag2_dynamic_recorder_ui',
            output='screen',
            parameters=[{
                'port': int(LaunchConfiguration('port').perform(context)),
                'bind': LaunchConfiguration('bind').perform(context),
                'recorder_node': '/rosbag2_dynamic_recorder',
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, default, desc in ARGUMENTS]
        + [OpaqueFunction(function=_setup)]
    )
