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

import json
import os
import sys

from rosbag2_dynamic_recorder_cli.verb import VerbExtension


class InfoVerb(VerbExtension):
    """Describe a recorded bag: what each channel really did, and at what rate."""

    def add_arguments(self, parser, cli_name):
        # Deliberately none of the recorder arguments. Every other verb here talks to a running
        # recorder; this one reads a finished bag and needs no graph at all.
        parser.add_argument(
            'bag', metavar='BAG',
            help='Path to the bag directory, the same one you would hand to `ros2 bag play`')
        parser.add_argument(
            '--storage-id', default='', metavar='ID',
            help='Storage plugin, e.g. mcap. Detected from the bag by default')
        parser.add_argument(
            '--json', action='store_true',
            help='Emit the report as JSON. Values the bag cannot establish are null, never zero')

    def main(self, *, args):
        # Imported here rather than at module scope so that `ros2 dynrec --help` still works, and
        # every other verb still runs, in an installation that lacks the library.
        try:
            from dynrec.bag import describe, format_summary, summary_dict
        except ImportError as exc:
            print('this verb needs the dynrec package: {}'.format(exc), file=sys.stderr)
            return 1

        if not os.path.isdir(args.bag):
            print("'{}' is not a directory. Pass the bag directory, not the .mcap file "
                  'inside it.'.format(args.bag), file=sys.stderr)
            return 1

        try:
            summary = describe(args.bag, storage_id=args.storage_id)
        except Exception as exc:
            print('could not read {}: {}'.format(args.bag, exc), file=sys.stderr)
            return 1

        if args.json:
            print(json.dumps(summary_dict(summary), indent=2))
        else:
            print('\n'.join(format_summary(summary)))

        # Non-zero when the bag holds a hole nothing accounts for, so a script checking a
        # recording can act on it. A sparse channel is normal and is not an error; a simultaneous
        # unexplained gap across every channel is the one shape that should not be there.
        return 2 if summary.unexplained_gaps else 0
