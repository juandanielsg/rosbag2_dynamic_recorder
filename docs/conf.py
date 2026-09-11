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

Two choices here are worth explaining, because both were made to keep the build honest.

**ROS is mocked, not installed.** Building these docs needs no ROS at all: `rclpy` and the two
interface packages are replaced by `autodoc_mock_imports` stand-ins. That is possible only because
the library was written so that only one of its seven modules imports ROS at module scope, and
`dynrec/__init__.py` resolves `Recorder` lazily -- the same property that lets 55 of its tests run
without a graph. The alternative, installing ROS Rolling in CI to build a docs page, would turn a
thirty-second job into a ten-minute one and make the docs unbuildable on a laptop.

**Nothing is fetched at build time.** No intersphinx, no remote inventories, no CDN. A docs build
that fails because someone else's server is down is a docs build that will eventually block a
commit, and this project's whole install story is about working on a robot with no internet.
"""

import os
import sys
from datetime import date

# The library itself, so autodoc can read it straight from the working tree rather than from an
# installed copy. Deliberately the source and not a build directory: the docs then describe the
# code in front of you, which is the only version a contributor can act on.
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
    # which ignores directories beginning with an underscore -- and _static/ is where the theme's
    # CSS and this project's logo live, so the site would come out unstyled and logo-less.
    'sphinx.ext.githubpages',
]

# Markdown throughout, because every existing document in this repository is Markdown. A
# reStructuredText docs tree would have meant maintaining two dialects or rewriting all of them.
source_suffix = {'.md': 'markdown', '.rst': 'restructuredtext'}

myst_enable_extensions = [
    'colon_fence',
    'deflist',
]
# Give sub-headings anchors down to h3, so the longer pages can be deep-linked and their
# in-page navigation works rather than listing only top-level sections.
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

# _static carries the logo and nothing else. Listed here only because it now has real content: an
# empty _static/ is a directory git will not track, so it exists locally, vanishes in a fresh
# clone, and fails the build under -W -- which is exactly how the first CI docs build failed.
html_static_path = ['_static']

REPO = 'https://github.com/juandanielsg/rosbag2_dynamic_recorder'
BRANCH = 'main'

html_theme_options = {
    'source_repository': REPO + '/',
    'source_branch': BRANCH,
    'source_directory': 'docs/',
}
