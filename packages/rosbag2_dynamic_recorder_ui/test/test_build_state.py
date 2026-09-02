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

"""The UI must never present a number the recorder cannot vouch for.

This is the project's rigor rule, and it is easy to break by accident: someone tidying the state
builder replaces `None` with `0` and the page starts cheerfully reporting "no messages missing" on
a middleware that cannot detect missing messages at all. These tests exist to make that change
fail loudly.

No ROS graph needed -- build_state is deliberately a plain function.
"""

from types import SimpleNamespace

from rosbag2_dynamic_recorder_ui.ui_node import build_state


def _status(**overrides):
    base = dict(
        uri="/tmp/bag",
        storage_id="mcap",
        recording=True,
        paused=False,
        snapshot_mode=False,
        elapsed_seconds=12.5,
        subscribed_topics=["/a", "/b"],
        messages_written=100,
        messages_missed=0,
        sequence_numbers_available=True,
        messages_lost=0,
        bag_splits=0,
        bag_size_bytes=2048,
    )
    base.update(overrides)
    return SimpleNamespace(**base)


def _state(status, age=0.0):
    return build_state(status, age, "/rec", ["/a", "/b", "/c"], [])


def test_missing_is_unknown_when_sequence_numbers_unavailable():
    """The whole point: unmeasurable must not render as zero."""
    state = _state(_status(sequence_numbers_available=False, messages_missed=0))
    assert state["messages_missed"] is None


def test_missing_is_unknown_even_if_a_count_leaked_through():
    """Guards against trusting a stale count when the flag says it is meaningless."""
    state = _state(_status(sequence_numbers_available=False, messages_missed=999))
    assert state["messages_missed"] is None


def test_missing_is_reported_when_measurable():
    assert _state(_status(messages_missed=7))["messages_missed"] == 7
    assert _state(_status(messages_missed=0))["messages_missed"] == 0


def test_transport_loss_is_kept_separate():
    """messages_lost has been observed reading 0 while messages were genuinely absent, so it must
    never be conflated with the detected count."""
    state = _state(_status(messages_missed=42, messages_lost=0))
    assert state["messages_missed"] == 42
    assert state["messages_lost_reported"] == 0
    assert "messages_lost" not in state, "reported loss must stay under its qualified name"


def test_disconnected_state_claims_nothing():
    state = build_state(None, None, "/rec", ["/a"], [])
    assert state["connected"] is False
    for field in ("messages_written", "messages_missed", "recording", "uri"):
        assert field not in state, f"{field} must not be invented while disconnected"


def test_stale_is_flagged_but_only_when_old():
    assert _state(_status(), age=0.5)["stale"] is False
    assert _state(_status(), age=30.0)["stale"] is True


def test_fields_are_plain_json_types():
    """The page consumes this as JSON; ROS array types would not survive serialization."""
    state = _state(_status())
    assert isinstance(state["subscribed_topics"], list)
    assert isinstance(state["recording"], bool)
    assert isinstance(state["paused"], bool)
