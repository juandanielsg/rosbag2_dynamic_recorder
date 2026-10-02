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

"""Adding, removing and replacing topics: selection, patterns, and refusals with their reasons."""

from rosbag2_dynamic_recorder_interfaces.srv import (
    GetSubscribedTopics,
    SetTopics,
    SubscribeTopics,
    UnsubscribeTopics,
)

from recorder_harness import NODE, TOPICS


def test_subscribe_then_status_reports_it(recorder):
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    assert response.return_code == 0, response.error_string
    assert list(response.subscribed_topics) == [TOPICS[0]]
    assert list(harness.status().subscribed_topics) == [TOPICS[0]]


def test_set_topics_leaves_shared_topics_alone(recorder):
    """The project's core claim, at the API level: a topic present before and after a change is
    never reported as touched."""
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=TOPICS[:2])
    response = harness.call(SetTopics, "set_topics", topics=[TOPICS[1], TOPICS[2]])

    assert response.return_code == 0, response.error_string
    assert TOPICS[0] in response.unsubscribed_topics, "the dropped topic should be reported"
    assert TOPICS[1] not in response.unsubscribed_topics, (
        "a topic in both the old and new set must not be torn down"
    )
    assert set(harness.status().subscribed_topics) == {TOPICS[1], TOPICS[2]}


def test_regex_selects_topics_without_naming_them(recorder):
    """The point of the feature: "record all of these" without enumerating them."""
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", regex="/rdr_test/")
    assert response.return_code == 0, response.error_string
    assert sorted(response.subscribed_topics) == sorted(TOPICS)


def test_regex_and_explicit_topics_combine(recorder):
    harness, _ = recorder
    response = harness.call(
        SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]], regex="gamma")
    assert response.return_code == 0, response.error_string
    assert sorted(response.subscribed_topics) == sorted([TOPICS[0], TOPICS[2]])


def test_exclude_regex_filters_explicitly_named_topics_too(recorder):
    """Applied to the combined set, which is what makes "all of X except Y" one call."""
    harness, _ = recorder
    response = harness.call(
        SubscribeTopics, "subscribe_topics",
        topics=[TOPICS[1]], regex="/rdr_test/", exclude_regex="beta")
    assert response.return_code == 0, response.error_string
    assert TOPICS[1] not in response.subscribed_topics, (
        "exclude_regex should filter a topic even when it was named explicitly"
    )
    assert sorted(response.subscribed_topics) == sorted([TOPICS[0], TOPICS[2]])


def test_a_pattern_never_matches_the_recorders_own_topics(recorder):
    """Its event channels are written into the bag directly; subscribing would double-record."""
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", regex=".*")
    assert response.return_code == 0, response.error_string
    ours = [t for t in response.subscribed_topics if t.startswith(NODE + "/")]
    assert ours == [], f"a pattern pulled in our own topics: {ours}"
    # Non-vacuous: '.*' really did match a great deal else.
    assert set(TOPICS).issubset(set(response.subscribed_topics))


def test_invalid_regex_is_refused_with_the_reason(recorder):
    """A typo must not look identical to a pattern that legitimately matched nothing."""
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", regex="/rdr_test/[")
    assert response.return_code != 0
    assert "invalid regular expression" in response.error_string
    assert harness.call(GetSubscribedTopics, "get_subscribed_topics").topics == [], (
        "a refused request should not have subscribed anything"
    )


def test_regex_matching_nothing_is_refused_on_subscribe(recorder):
    """Subscribe exists to add topics, so adding none is a request that was not met."""
    harness, _ = recorder
    response = harness.call(SubscribeTopics, "subscribe_topics", regex="^/nothing_matches_this")
    assert response.return_code != 0
    assert "matched no topics" in response.error_string


def test_set_topics_regex_replaces_the_selection(recorder):
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=[TOPICS[1]])
    response = harness.call(SetTopics, "set_topics", regex="alpha")
    assert response.return_code == 0, response.error_string
    assert response.subscribed_topics == [TOPICS[0]]
    assert TOPICS[1] in response.unsubscribed_topics


def test_set_topics_reports_total_failure(recorder):
    """Regression: a set_topics whose every topic failed used to report success, so a caller
    could not tell "all eight missing" from "all eight recorded" without parsing the lists."""
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=TOPICS[:2])

    response = harness.call(
        SetTopics, "set_topics", topics=["/rdr_test/nobody_publishes_this"])
    assert response.return_code != 0, "an all-unavailable set_topics is a failure"
    assert "/rdr_test/nobody_publishes_this" in response.unavailable_topics
    # The old selection was dropped before the add failed, and the add did not land.
    assert list(response.subscribed_topics) == []
    assert not harness.status().subscribed_topics


def test_unsubscribe_regex_matches_what_is_recorded_not_the_graph(recorder):
    """Only the recorded subset can be dropped, so that is the pool a pattern searches."""
    harness, _ = recorder
    harness.call(SetTopics, "set_topics", topics=TOPICS[:2])
    response = harness.call(UnsubscribeTopics, "unsubscribe_topics", regex="/rdr_test/")
    assert response.return_code == 0, response.error_string
    # gamma is on the graph and matches the pattern, but was never recorded, so it is not
    # reported as dropped and not reported as an error either.
    assert sorted(response.unsubscribed_topics) == sorted(TOPICS[:2])
    assert TOPICS[2] not in response.unsubscribed_topics
    assert response.not_subscribed_topics == []


def test_unsubscribe_reports_topics_it_was_not_recording(recorder):
    harness, _ = recorder
    harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    response = harness.call(UnsubscribeTopics, "unsubscribe_topics", topics=TOPICS[:2])
    assert list(response.unsubscribed_topics) == [TOPICS[0]]
    assert list(response.not_subscribed_topics) == [TOPICS[1]]


def test_invalid_topic_name_is_refused_not_fatal(recorder):
    """Regression: this used to terminate the process.

    Supplying topic_types skips the graph lookup, so a bad name reaches
    create_generic_subscription directly. An exception there escapes the service callback and
    takes the whole node down, losing the recording along with it.
    """
    harness, _ = recorder
    response = harness.call(
        SubscribeTopics, "subscribe_topics",
        topics=["not a valid topic name!"], topic_types=["std_msgs/msg/String"],
    )
    assert response.return_code != 0
    assert response.error_string, "the failure should say what went wrong"

    # The point of the test: the recorder is still alive and usable afterwards.
    assert harness.status().recording is True
    ok = harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
    assert ok.return_code == 0, "the node should still work after a rejected request"


def test_unloadable_type_is_refused_not_fatal(recorder):
    """Same path, reached through a type that has no message library to load."""
    harness, _ = recorder
    response = harness.call(
        SubscribeTopics, "subscribe_topics",
        topics=[TOPICS[0]], topic_types=["no_such_pkg/msg/NoSuchType"],
    )
    assert response.return_code != 0
    assert harness.status().recording is True


def test_changed_type_on_a_known_topic_is_refused(recorder):
    """A bag channel is bound to one type. Writing a different type into it would silently
    corrupt the recording, so the subscribe is refused instead."""
    harness, _ = recorder
    assert harness.call(
        SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]]).return_code == 0
    harness.call(UnsubscribeTopics, "unsubscribe_topics", topics=[TOPICS[0]])

    response = harness.call(
        SubscribeTopics, "subscribe_topics",
        topics=[TOPICS[0]], topic_types=["sensor_msgs/msg/Imu"],
    )
    assert response.return_code != 0
    assert "already in this bag" in response.error_string, response.error_string
    assert harness.status().recording is True
