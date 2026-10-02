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

"""Recorder fixtures: each starts a recorder process and a harness, and stops both."""

import pytest
from rosbag2_dynamic_recorder_interfaces.srv import SubscribeTopics

from recorder_harness import BAG_SIZE_LIMIT, TOPICS, start_recorder


@pytest.fixture()
def recorder():
    """A freshly started recorder plus a harness publishing on the test topics."""
    harness, bag, cleanup = start_recorder()
    try:
        yield harness, bag
    finally:
        cleanup()


@pytest.fixture()
def paused_recorder():
    """A recorder that opened its bag already paused, so the bag begins with a hole."""
    harness, bag, cleanup = start_recorder(("-p", "start_paused:=true"))
    try:
        yield harness, bag
    finally:
        cleanup()


@pytest.fixture()
def unexplained_recorder():
    """A recorder with pause events switched off, for the opt-out."""
    harness, bag, cleanup = start_recorder(("-p", "record_pause_events:=false"))
    try:
        yield harness, bag
    finally:
        cleanup()


@pytest.fixture()
def low_disk_recorder():
    """A recorder whose free-space guard fires at the first check, so it stops itself.

    The minimum is larger than any real disk, so the condition is certain without having to fill
    one. A percentage probe is not used because a fresh tmpfs can legitimately be ~100% free.
    """
    harness, bag, cleanup = start_recorder((
        "-p", "min_free_space:=9223372036854775807",
        "-p", "storage_check_period:=0.2"))
    try:
        yield harness, bag
    finally:
        cleanup()


@pytest.fixture()
def size_limited_recorder():
    """A recorder with a bag size cap, fed messages fat enough to reach it within seconds.

    Size on disk is what the guard measures, and MCAP only writes a chunk once it holds ~768 KiB,
    so the harness's usual few-byte strings would take minutes to register. 50 KB per message on
    one topic at 20 Hz is ~1 MB/s: the first chunk lands within a second or two.
    """
    harness, bag, cleanup = start_recorder((
        "-p", f"max_bag_size:={BAG_SIZE_LIMIT}",
        "-p", "storage_check_period:=0.2"))
    try:
        harness.payload = "x" * 50_000
        harness.call(SubscribeTopics, "subscribe_topics", topics=[TOPICS[0]])
        yield harness, bag
    finally:
        cleanup()


@pytest.fixture()
def profiled_recorder():
    """A recorder configured with two deliberately overlapping profiles."""
    harness, bag, cleanup = start_recorder((
        "-p", "profile_names:=[small,large,ghost]",
        "-p", f"profiles.small:=[{TOPICS[0]}]",
        "-p", f"profiles.large:=[{TOPICS[0]},{TOPICS[1]}]",
        "-p", "profiles.ghost:=[/rdr_test/nobody_publishes_this]",
    ))
    try:
        yield harness, bag
    finally:
        cleanup()
