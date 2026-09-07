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

"""Sphinx configuration.

Two choices here are worth explaining, because both were made to keep the build honest.

**ROS is mocked, not installed.** Building these docs needs no ROS at all: `rclpy` and the two
interface packages are replaced by `autodoc_mock_imports` stand-ins. That is possible only because
the library was written so that four of its five modules import no ROS, and `dynrec/__init__.py`
resolves `Recorder` lazily -- the same property that lets 36 of its tests run without a graph. The
alternative, installing ROS Rolling in CI to build a docs page, would turn a thirty-second job into
a ten-minute one and make the docs unbuildable on a laptop.

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
author = 'juandanielsg'
copyright = '{}, {}'.format(date.today().year, author)
release = '0.1.0'
version = '0.1.0'

extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
]

# Markdown throughout, because every existing document in this repository is Markdown and the
# design notes below are included verbatim from notes/ rather than copied. A reStructuredText
# docs tree would have meant maintaining two dialects or rewriting eight working notes.
source_suffix = {'.md': 'markdown', '.rst': 'restructuredtext'}

myst_enable_extensions = [
    'colon_fence',
    'deflist',
]
# The notes use ## and ### freely; without this, their sub-headings produce no anchors and the
# in-page navigation for a 28,000-word roadmap is useless.
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
html_static_path = ['_static']

REPO = 'https://github.com/juandanielsg/rosbag2_dynamic_recorder'
BRANCH = 'main'

html_theme_options = {
    'source_repository': REPO + '/',
    'source_branch': BRANCH,
    'source_directory': 'docs/',
}

# -- links out of the docs tree ----------------------------------------------------------------


def _repo_url(path):
    """A link to `path` in the repository, as a file or as a directory."""
    kind = 'tree' if path.endswith('/') or '.' not in path.rsplit('/', 1)[-1] else 'blob'
    return '{}/{}/{}/{}'.format(REPO, kind, BRANCH, path.rstrip('/'))


def _rewrite_repo_links(app, doctree):
    """Point the design notes' links at the repository when they are not documentation pages.

    The notes under `design/` are included verbatim from `notes/`, so their relative links are
    written relative to `notes/` -- `../spike/verify_events.py` means the repository's `spike/`
    directory. Sibling links like `architecture.md` are other notes and resolve to real pages here;
    the rest point at a smoke test, a probe script, a source directory, none of which is a
    documentation page.

    Rewriting those rather than editing the notes keeps one source for them and keeps both
    renderings working: relative on GitHub, absolute here. Suppressing the warning instead would
    have hidden genuinely broken cross-references along with these.

    Runs on `doctree-read`, before the reference resolution that would otherwise warn.
    """
    import posixpath

    from docutils import nodes as docutils_nodes
    from sphinx import addnodes

    docname = app.env.docname
    if not docname.startswith('design/'):
        return

    for node in list(doctree.findall(addnodes.pending_xref)):
        # MyST marks its own local links with reftype 'myst' and leaves refdomain None, so the
        # reftype is what identifies them.
        if node.get('reftype') != 'myst':
            continue
        target = node.get('reftarget', '')
        path, _, anchor = target.partition('#')
        if not path:
            continue

        # MyST hands over a docname ('design/architecture') for a link it could resolve inside the
        # source tree, and the raw path ('../spike/verify_events.py') for one it could not. So a
        # target that names a real page is left alone, and only the leftovers are rewritten.
        as_doc = path[:-3] if path.endswith('.md') else path
        if as_doc in app.env.found_docs:
            continue

        url = _repo_url(posixpath.normpath(posixpath.join('notes', path)))
        if anchor:
            url += '#' + anchor
        reference = docutils_nodes.reference('', '', refuri=url, internal=False)
        reference.extend(node.children)
        node.replace_self(reference)


def setup(app):
    app.connect('doctree-read', _rewrite_repo_links)
    return {'parallel_read_safe': True, 'parallel_write_safe': True}
