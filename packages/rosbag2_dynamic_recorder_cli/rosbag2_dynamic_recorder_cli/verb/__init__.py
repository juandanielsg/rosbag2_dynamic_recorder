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

from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION, satisfies_version

from rosbag2_dynamic_recorder_cli.api import add_recorder_arguments, with_recorder


class VerbExtension:
    """
    The extension point for 'dynrec' verb extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods must be defined:
    * `main`

    The following methods can be defined:
    * `add_arguments`
    """

    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(VerbExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        raise NotImplementedError()


class RecorderVerb(VerbExtension):
    """A verb that talks to one recorder.

    Subclasses add their own arguments after `super().add_arguments()` and implement `run`. A
    refusal raises out of `run`; `with_recorder` turns it into stderr and the exit code.
    """

    def add_arguments(self, parser, cli_name):
        add_recorder_arguments(parser)

    def main(self, *, args):
        return with_recorder(args, lambda recorder: self.run(recorder, args))

    def run(self, recorder, args):
        raise NotImplementedError()
