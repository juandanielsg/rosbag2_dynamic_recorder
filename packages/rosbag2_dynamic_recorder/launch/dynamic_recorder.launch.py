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


def _bool(text):
    return text == 'true'


#: (name, default, type, description). Each typed argument becomes the recorder parameter of the
#: same name; `topics` and `params_file` are handled apart.
ARGUMENTS = [
    ('uri', 'dynamic_bag', str, 'Output bag path.'),
    ('storage_id', 'mcap', str, 'Storage plugin.'),
    ('serialization_format', 'cdr', str, 'Message serialization format.'),
    ('topics', '[]', None, 'Topics to record at startup, as a YAML list. May be empty.'),
    ('start_paused', 'false', _bool, 'Start with recording paused.'),
    ('use_sim_time', 'false', _bool,
     'Stamp on the /clock-driven node clock; wait for /clock to start.'),
    ('snapshot_mode', 'false', _bool, 'Buffer in memory and only write on ~/snapshot.'),
    ('max_cache_size', str(100 * 1024 * 1024), int,  # the node's own default, 100 MiB
     'Writer cache in bytes. snapshot_mode needs this or max_cache_duration to be > 0.'),
    ('max_cache_duration', '0', int,
     'Writer cache bound in seconds; 0 for none. Combines with max_cache_size. A bound an '
     'operator can reason about: at most this much recording is at risk if the process dies.'),
    ('max_bagfile_size', '0', int, 'Split the bag when a file reaches this many bytes; 0 never.'),
    ('max_bagfile_duration', '0', int, 'Split the bag every this many seconds; 0 never.'),
    ('storage_preset_profile', '', str,
     'Storage plugin preset. For mcap: none, fastwrite, zstd_fast, zstd_small. fastwrite is '
     'the one for a weak CPU; the zstd presets trade CPU for disk bandwidth.'),
    ('storage_config_uri', '', str, 'Path to a storage-plugin YAML, overlaid on the preset.'),
    ('record_subscription_events', 'true', _bool,
     'Record SubscriptionChangeEvent into the bag, so sparse channels explain themselves.'),
    ('min_free_space', '0', int,
     'Stop recording when free space on the bag filesystem falls below this many bytes; '
     '0 disables. Protects the disk, not the bag: logs or a second recorder can fill it.'),
    ('min_free_space_percent', '0.0', float,
     'Same, as a percentage of the filesystem; 0.0 disables. When both are set the stricter one '
     'applies.'),
    ('max_bag_size', '0', int,
     'Stop recording when the bag directory, across every split, grows past this many bytes; '
     '0 disables. A cap on the recording where max_bagfile_size only rolls to a new file.'),
    ('messages_lost_report_period', '5.0', float,
     'Seconds between MessagesLostEvent publications. 0 disables reporting.'),
    ('params_file', '', None, 'Optional YAML of extra parameters, e.g. recording profiles.'),
]


def topic_list(raw):
    """Parse the `topics` argument into a list. literal_eval, not eval: nothing here should run."""
    raw = raw.strip()
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
    """The typed arguments as parameters, plus `topics` when it is not empty.

    An empty list does not survive the launch parameters file -- rcl reads it as "no value" and
    the node aborts -- and the node records nothing by default anyway.
    """
    def value(name):
        return LaunchConfiguration(name).perform(context)

    params = {name: parse(value(name)) for name, _, parse, _ in ARGUMENTS if parse is not None}
    topics = topic_list(value('topics'))
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
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, default, _, description in ARGUMENTS]
        + [OpaqueFunction(function=_setup)]
    )
