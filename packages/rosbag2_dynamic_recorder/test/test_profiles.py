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

"""Named topic sets: applying one, and an active_profile derived from what is recorded."""

from rosbag2_dynamic_recorder_interfaces.srv import (
    GetProfiles,
    SetProfile,
    SubscribeTopics,
    UnsubscribeTopics,
)

from recorder_harness import TOPICS


def test_profiles_are_offered_as_configured(profiled_recorder):
    harness, _ = profiled_recorder
    response = harness.call(GetProfiles, "get_profiles")
    names = [p.name for p in response.profiles]
    assert names == ["small", "large", "ghost"], (
        "declaration order should be preserved for a UI to list")


def test_switching_profiles_does_not_touch_shared_topics(profiled_recorder):
    """The fleet case: small and large share a topic, and switching must not interrupt it."""
    harness, _ = profiled_recorder
    harness.call(SetProfile, "set_profile", name="small")
    response = harness.call(SetProfile, "set_profile", name="large")

    assert response.return_code == 0, response.error_string
    assert TOPICS[0] not in response.unsubscribed_topics, (
        "the topic shared by both profiles was torn down"
    )
    assert set(response.subscribed_topics) == {TOPICS[0], TOPICS[1]}


def test_active_profile_is_derived_not_remembered(profiled_recorder):
    """A remembered name would still read 'large' after someone changed a topic by hand, which
    would be a lie in the status."""
    harness, _ = profiled_recorder
    harness.call(SetProfile, "set_profile", name="large")
    assert harness.status().active_profile == "large"

    # Add a topic no profile lists, so the selection matches nothing.
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[2]])
    assert harness.status().active_profile == "", (
        "the selection no longer matches any profile, so none is active"
    )

    # Returning to exactly a profile's set makes it active again, with no set_profile call.
    harness.call(UnsubscribeTopics, "unsubscribe_topics", topics=[TOPICS[2]])
    assert harness.status().active_profile == "large"


def test_active_profile_follows_the_topics_not_the_last_command(profiled_recorder):
    """Derived means derived: dropping a topic from `large` lands exactly on `small`, and the
    status says so even though set_profile was never called with that name.

    This is the behaviour a remembered label could not produce, and it is the honest one -- the
    recorder really is recording the small profile at that point.
    """
    harness, _ = profiled_recorder
    harness.call(SetProfile, "set_profile", name="large")
    harness.call(UnsubscribeTopics, "unsubscribe_topics", topics=[TOPICS[1]])
    assert harness.status().active_profile == "small"


def test_unknown_profile_says_what_is_configured(profiled_recorder):
    harness, _ = profiled_recorder
    response = harness.call(SetProfile, "set_profile", name="nonexistent")
    assert response.return_code != 0
    assert "small" in response.error_string and "large" in response.error_string, (
        "the error should tell the caller what they could have asked for"
    )


def test_profile_reports_total_failure(profiled_recorder):
    """Mirror of the set_topics rule: a profile none of whose topics could be subscribed is a
    failure the caller has to see, not a success with an empty list."""
    harness, _ = profiled_recorder
    harness.call(SetProfile, "set_profile", name="small")

    response = harness.call(SetProfile, "set_profile", name="ghost")
    assert response.return_code != 0, "an all-unavailable profile is a failure"
    assert "/rdr_test/nobody_publishes_this" in response.unavailable_topics
    assert list(response.subscribed_topics) == []


def test_profiles_absent_when_none_configured(recorder):
    harness, _ = recorder
    assert harness.call(GetProfiles, "get_profiles").profiles == []
    assert harness.status().active_profile == ""
