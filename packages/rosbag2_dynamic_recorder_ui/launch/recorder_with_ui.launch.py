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

Every argument of the recorder's own launch file is accepted here unchanged -- it is included,
not copied, so a parameter added there reaches this entry point without a second table to keep
in step. To offer named profiles, point params_file at a YAML like
rosbag2_dynamic_recorder/config/profiles.example.yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from rosbag2_dynamic_recorder_ui import DEFAULT_PORT, DEFAULT_RECORDER_NODE


def generate_launch_description():
    recorder = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('rosbag2_dynamic_recorder'), 'launch', 'dynamic_recorder.launch.py'])))
    ui = Node(
        package='rosbag2_dynamic_recorder_ui',
        executable='ui_node',
        name='rosbag2_dynamic_recorder_ui',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'bind': ParameterValue(LaunchConfiguration('bind'), value_type=str),
            'recorder_node': DEFAULT_RECORDER_NODE,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value=str(DEFAULT_PORT), description='Port for the browser UI.'),
        DeclareLaunchArgument(
            'bind', default_value='127.0.0.1',
            description='Address the UI listens on. Loopback by default because the UI has no '
                        'authentication; use 0.0.0.0 only on a trusted network.'),
        recorder,
        ui,
    ])
