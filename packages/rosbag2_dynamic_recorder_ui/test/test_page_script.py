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

"""The page's own JavaScript, parsed and run.

No other test executes it: `/api/state` answers whether or not the page parses, so a syntax error
would leave the UI showing only its placeholder text behind a green test suite.

So: parse web/app.js, and run its render path against sample state through a DOM stub in
[page_harness.js](page_harness.js). Not a browser. It answers three questions: does rendering
throw, which is the failure that leaves the page frozen; does text from the ROS graph reach the
page escaped; and does a poll that brings nothing new leave the page's elements alone.

Needs node. The dev container image and CI install it; elsewhere these skip rather than fail.
"""

import json
import shutil
import subprocess
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
PAGE = HERE.parent / "web" / "index.html"
SCRIPT = HERE.parent / "web" / "app.js"
HARNESS = HERE / "page_harness.js"

NODE = shutil.which("node") or shutil.which("nodejs")
needs_node = pytest.mark.skipif(
    NODE is None, reason="node is not installed; add nodejs to the image to run these")

#: Mirrors what `build_state` returns for a connected recorder.
#: adds. Written out rather than imported so these tests need only node, not a ROS install --
#: `test_build_state.py` asserts the two stay in step, which is where the drift would show.
SAMPLE_STATE = {
    "connected": True,
    "stale": False,
    "recorder": "/rosbag2_dynamic_recorder",
    "uri": "/tmp/bag",
    "storage_id": "mcap",
    "recording": True,
    "paused": False,
    "snapshot_mode": False,
    "elapsed_seconds": 62.5,
    "recording_started": 1788500000.0,
    "stamp": 1788500062.5,
    "subscribed_topics": ["/spike/a", "/spike/c"],
    "active_profile": "",
    "available_topics": ["/spike/a", "/spike/b", "/spike/c"],
    "stale_after_seconds": 5.0,
    "messages_written": 1234,
    # None on purpose: the page must render this as unknown, and that branch has to be walked.
    "messages_missed": None,
    "messages_lost_in_transport": 0,
    "messages_lost_in_recorder": 0,
    "messages_lost_reported": 0,
    "write_errors": 0,
    "bag_splits": 1,
    "bag_size_bytes": 4096,
    "sequence_numbers_available": False,
    "free_space_bytes": 0,
    "total_space_bytes": 0,
    "min_free_space": 0,
    "min_free_space_percent": 0.0,
    "stopped_for_low_disk": False,
    "max_bag_size": 0,
    "stopped_for_max_bag_size": False,
    "use_sim_time": False,
    "waiting_for_clock": False,
    "recorder_now": 1788500062.5,
    "schedules": [
        {"action": "resume", "at": 1788500090.0, "mode": "node", "topic": ""},
        {"action": "split", "at": 1788500120.0, "mode": "publish", "topic": "/spike/a"},
    ],
    "topic_losses": [
        # missed None on purpose, like messages_missed above: the tag must not count it as zero.
        {"topic": "/spike/c", "missed": None, "lost_in_transport": 2, "lost_in_recorder": 1},
    ],
    "profiles": [
        {"name": "light", "topics": ["/spike/a"]},
        {"name": "everything", "topics": ["/spike/a", "/spike/b", "/spike/c"]},
    ],
    "history": [
        {"kind": "subscription", "topic": "/spike/a", "action": "subscribed",
         "reason": "startup", "stamp": 1788500000.4},
        {"kind": "subscription", "topic": "/spike/b", "action": "subscribed",
         "reason": "startup", "stamp": 1788500001.1},
        {"kind": "subscription", "topic": "/spike/c", "action": "subscribed",
         "reason": "service:subscribe_topics", "stamp": 1788500011.3},
        {"kind": "subscription", "topic": "/spike/b", "action": "unsubscribed",
         "reason": "service:unsubscribe_topics", "stamp": 1788500030.0},
        {"kind": "pause", "topic": "", "action": "paused",
         "reason": "service:pause", "stamp": 1788500041.0},
        {"kind": "pause", "topic": "", "action": "resumed",
         "reason": "service:resume", "stamp": 1788500055.0},
        {"kind": "split", "topic": "", "action": "split", "reason": "service:split_bagfile",
         "stamp": 1788500058.0, "closed_file": "/tmp/bag/bag_0.mcap",
         "opened_file": "/tmp/bag/bag_1.mcap"},
    ],
}


def test_the_page_loads_its_script_and_stylesheet():
    """The page refers to the files the UI node serves, and needs no node to check it."""
    html = PAGE.read_text(encoding="utf-8")
    assert '<script src="app.js"></script>' in html
    assert '<link rel="stylesheet" href="style.css">' in html
    assert SCRIPT.read_text(encoding="utf-8").strip(), "the script is empty"


@needs_node
def test_the_page_script_parses():
    """If the script does not parse, nothing on the page runs."""
    result = subprocess.run([NODE, "--check", str(SCRIPT)],
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, "the page's JavaScript does not parse:\n" + result.stderr


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    """One run of the harness over the page's script and SAMPLE_STATE, shared by the tests below."""
    if NODE is None:
        pytest.skip("node is not installed; add nodejs to the image to run these")
    state = tmp_path_factory.mktemp("page") / "state.json"
    state.write_text(json.dumps(SAMPLE_STATE), encoding="utf-8")
    return subprocess.run([NODE, str(HARNESS), str(SCRIPT), str(state)],
                          capture_output=True, text=True, timeout=120)


def passed(run, label):
    """Whether the harness ran the check called `label` and it passed."""
    return f"  ok    {label}" in run.stdout


@needs_node
def test_rendering_does_not_throw_on_any_branch(harness):
    """Parsing is not enough: a render that throws leaves the page on its placeholder too."""
    assert harness.returncode == 0, (
        "a check in the harness failed:\n" + harness.stdout + harness.stderr)
    # Non-vacuous: the harness must actually have exercised the branches, not just loaded.
    assert passed(harness, 'filter regex "["'), (
        "the harness did not run the invalid-pattern branch:\n" + harness.stdout)


@needs_node
def test_text_from_the_graph_is_escaped(harness):
    """~/record takes any uri a ROS client sends. It, and every event and profile name, used to
    reach innerHTML raw, so whoever could call the recorder could run script in the operator's
    browser."""
    assert passed(harness, "text from the graph is escaped"), harness.stdout + harness.stderr


@needs_node
def test_a_poll_with_nothing_new_rebuilds_nothing(harness):
    """The page re-rendered its lists every second, which took keyboard focus off a checkbox
    and closed any tooltip, within a second of either arriving."""
    assert passed(harness, "an unchanged state rebuilds nothing"), harness.stdout + harness.stderr
    assert passed(harness, "a ticked box survives the next poll"), harness.stdout + harness.stderr
