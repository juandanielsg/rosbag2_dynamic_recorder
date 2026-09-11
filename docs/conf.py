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

"""Sphinx configuration.

Two choices are explained here because they keep the build self-contained.

**ROS is mocked, not installed.** Building these docs needs no ROS: `rclpy` and the two interface
packages are replaced by `autodoc_mock_imports` stand-ins. That works because only one of the
library's seven modules imports ROS at module scope, and `dynrec/__init__.py` resolves `Recorder`
lazily, the same property that lets 55 of its tests run without a graph. Installing ROS Rolling in
CI to build a page would turn a thirty-second job into a ten-minute one.

**Nothing is fetched at build time.** No intersphinx, no remote inventories, no CDN. A docs build
that fails because someone else's server is down will eventually block a commit, and this project's
install story is about working on a robot with no internet.
"""

import os
import sys
from datetime import date

# The library itself, so autodoc reads it from the working tree rather than an installed copy. The
# source, not a build directory, so the docs describe the code in front of you.
sys.path.insert(0, os.path.abspath('../packages/dynrec'))

project = 'rosbag2_dynamic_recorder'
author = 'Juan Daniel Suárez González'
copyright = '{}, {}'.format(date.today().year, author)
release = '0.1.0'
version = '0.1.0'

extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    # Writes .nojekyll into the output. Without it GitHub Pages may run the build through Jekyll,
    # which ignores directories beginning with an underscore; _static/ holds the theme CSS and the
    # logo, so the site would come out unstyled.
    'sphinx.ext.githubpages',
]

# Markdown throughout: every other document in the repository is Markdown, and a reStructuredText
# tree would mean maintaining two dialects.
source_suffix = {'.md': 'markdown', '.rst': 'restructuredtext'}

myst_enable_extensions = [
    'colon_fence',
    'deflist',
]
# Anchor sub-headings down to h3 so the longer pages can be deep-linked.
myst_heading_anchors = 3

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'README.md']

# -- autodoc ----------------------------------------------------------------------------------

# Everything ROS. `dynrec.client` imports all of these at module scope, and mocking them is what
# lets this build run on a plain Python image.
autodoc_mock_imports = [
    'rclpy',
    'rosbag2_interfaces',
    'rosbag2_dynamic_recorder_interfaces',
]

autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'undoc-members': False,
    'show-inheritance': True,
}
# Signatures carry defaults that are meaningful (DEFAULT_TIMEOUT, DEFAULT_DISCOVERY_TIMEOUT), so
# they are preserved rather than collapsed.
autodoc_preserve_defaults = True
autodoc_typehints = 'description'

# -- HTML -------------------------------------------------------------------------------------

html_theme = 'furo'
html_title = 'rosbag2_dynamic_recorder'
html_logo = '_static/dynrec_logo.png'

# _static holds the logo. Listed because an empty _static/ is not tracked by git: it exists
# locally, vanishes in a fresh clone, and fails the build under -W.
html_static_path = ['_static']

REPO = 'https://github.com/juandanielsg/rosbag2_dynamic_recorder'
BRANCH = 'main'

html_theme_options = {
    'source_repository': REPO + '/',
    'source_branch': BRANCH,
    'source_directory': 'docs/',
}
