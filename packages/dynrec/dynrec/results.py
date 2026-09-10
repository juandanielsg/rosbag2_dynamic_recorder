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

"""What the calls hand back: plain data, not ROS messages.

Returning the response message would be the least work and the worst interface. A script would
then have to know that `messages_missed` is only meaningful when `sequence_numbers_available` is
set, and the first script that forgets reports zero missing messages on a middleware that cannot
count them -- a wrong number, in the one place this project cares most about being honest.

So the rule the CLI's `--json` and the browser UI both encode is enforced here instead, once:
**a number the recorder cannot vouch for is None, never a convenient zero.**

No ROS import. The `from_msg` constructors only read attributes, so they can be exercised against
any object carrying the right fields.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

#: Maps the uint8 action constants onto words, because a script comparing against 0 and 1 is one
#: constant rename away from silently inverting.
SUBSCRIPTION_ACTIONS = {0: 'subscribed', 1: 'unsubscribed'}
PAUSE_ACTIONS = {0: 'paused', 1: 'resumed'}


def _stamp_seconds(stamp):
    """A builtin_interfaces/Time as epoch seconds on the recorder's clock."""
    return stamp.sec + stamp.nanosec / 1e9


@dataclass(frozen=True)
class Status:
    """What the recorder is doing, as of one moment.

    The clock caveat worth carrying into any script: `recording_started` and every event stamp are
    on the *recorder's* clock, not this process's. Compare them with each other, never with
    `time.time()` unless you know the two clocks agree.
    """

    recorder: str
    uri: str
    storage_id: str
    recording: bool
    paused: bool
    snapshot_mode: bool
    recording_started: float
    elapsed_seconds: float
    subscribed_topics: List[str]
    active_profile: str
    messages_written: int
    #: None when the middleware supplies no publication sequence numbers, i.e. when the recorder
    #: genuinely cannot tell. Never zero in that case -- zero would be a claim it cannot make.
    messages_missed: Optional[int]
    #: Losses the transport or writer *reported*. Observed reading 0 while ~3-4% of messages were
    #: absent from a bag, so it is a floor, not a total. `messages_missed` is the honest one.
    messages_lost_reported: int
    write_errors: int
    bag_splits: int
    #: Bytes flushed to disk, which lags what has been captured because the writer caches. Reads 0
    #: early in a healthy recording; use `messages_written` for liveness.
    bag_size_bytes: int
    sequence_numbers_available: bool

    @classmethod
    def from_msg(cls, msg, recorder=''):
        sequence_ok = bool(msg.sequence_numbers_available)
        return cls(
            recorder=recorder,
            uri=msg.uri,
            storage_id=msg.storage_id,
            recording=bool(msg.recording),
            paused=bool(msg.paused),
            snapshot_mode=bool(msg.snapshot_mode),
            recording_started=_stamp_seconds(msg.recording_started),
            elapsed_seconds=msg.elapsed_seconds,
            subscribed_topics=list(msg.subscribed_topics),
            active_profile=msg.active_profile,
            messages_written=msg.messages_written,
            messages_missed=msg.messages_missed if sequence_ok else None,
            messages_lost_reported=msg.messages_lost,
            write_errors=msg.write_errors,
            bag_splits=msg.bag_splits,
            bag_size_bytes=msg.bag_size_bytes,
            sequence_numbers_available=sequence_ok,
        )

    def as_dict(self):
        """Plain data for logging or JSON, with the unknown-versus-zero rule intact."""
        return {
            'recorder': self.recorder,
            'uri': self.uri,
            'storage_id': self.storage_id,
            'recording': self.recording,
            'paused': self.paused,
            'snapshot_mode': self.snapshot_mode,
            'elapsed_seconds': self.elapsed_seconds,
            'subscribed_topics': list(self.subscribed_topics),
            'active_profile': self.active_profile,
            'messages_written': self.messages_written,
            'messages_missed': self.messages_missed,
            'messages_lost_reported': self.messages_lost_reported,
            'write_errors': self.write_errors,
            'bag_splits': self.bag_splits,
            'bag_size_bytes': self.bag_size_bytes,
        }


@dataclass(frozen=True)
class TopicChange:
    """The outcome of a call that changed what is being recorded.

    One type for all four of them, because a caller that switches between `add` and `set_topics`
    should not have to switch result types too; the lists a given call cannot fill stay empty.

    `unavailable` is the field to actually read. The services report a partial success -- two
    topics added, one absent from the graph -- as success, so nothing raises, and this list is the
    only thing that says you are recording less than you asked for.
    """

    #: Topics being recorded after the call. `add` and `set_topics` fill it; the others do not.
    subscribed: List[str] = field(default_factory=list)
    #: Topics no longer being recorded because of this call.
    unsubscribed: List[str] = field(default_factory=list)
    #: Requested topics that could not be subscribed: absent from the graph, invalid, or of
    #: ambiguous type. Empty is the only reassuring value.
    unavailable: List[str] = field(default_factory=list)
    #: For `remove`: topics that were not being recorded in the first place, so nothing happened.
    not_subscribed: List[str] = field(default_factory=list)

    @classmethod
    def from_msg(cls, response):
        return cls(
            subscribed=list(getattr(response, 'subscribed_topics', ())),
            unsubscribed=list(getattr(response, 'unsubscribed_topics', ())),
            unavailable=list(getattr(response, 'unavailable_topics', ())),
            not_subscribed=list(getattr(response, 'not_subscribed_topics', ())),
        )

    @property
    def complete(self):
        """True when everything asked for happened -- nothing was unavailable."""
        return not self.unavailable


@dataclass(frozen=True)
class Profiles:
    """The configured profiles, and which one matches what is being recorded.

    `active` is derived by the recorder from the live topic set on every reply rather than
    remembered, so it cannot claim a profile that a manual topic change has since broken. An empty
    `active` after any hand-edited selection is the normal, truthful answer.
    """

    profiles: Dict[str, List[str]] = field(default_factory=dict)
    active: str = ''

    @classmethod
    def from_msg(cls, response):
        return cls(
            profiles={p.name: list(p.topics) for p in response.profiles},
            active=response.active_profile,
        )

    @property
    def names(self):
        return list(self.profiles)

    def __contains__(self, name):
        return name in self.profiles

    def __getitem__(self, name):
        return self.profiles[name]

    def __iter__(self):
        return iter(self.profiles)

    def __len__(self):
        return len(self.profiles)


@dataclass(frozen=True)
class Event:
    """One thing that happened to the recording, from either event stream.

    Both streams are flattened into one shape because a caller watching for "what changed" wants
    them interleaved, and because the two are only meaningful together: a subscription event
    explains a channel that stops, a pause event explains a bag that stops.

    `stamp` is on the recorder's clock. Sort by it rather than by arrival: the streams come in on
    separate subscriptions, so delivery order is not the order things happened.
    """

    #: 'subscription' or 'pause'.
    kind: str
    #: 'subscribed', 'unsubscribed', 'paused' or 'resumed'.
    action: str
    #: Epoch seconds on the recorder's clock.
    stamp: float
    #: What caused it, e.g. 'service:set_topics', 'schedule:resume', 'startup'.
    reason: str
    node_name: str
    #: Empty for a pause event, which affects every topic at once.
    topic: str = ''
    topic_type: str = ''

    @classmethod
    def from_subscription_msg(cls, msg):
        return cls(
            kind='subscription',
            action=SUBSCRIPTION_ACTIONS.get(msg.action, str(msg.action)),
            stamp=_stamp_seconds(msg.stamp),
            reason=msg.reason,
            node_name=msg.node_name,
            topic=msg.topic_name,
            topic_type=msg.topic_type,
        )

    @classmethod
    def from_pause_msg(cls, msg):
        return cls(
            kind='pause',
            action=PAUSE_ACTIONS.get(msg.action, str(msg.action)),
            stamp=_stamp_seconds(msg.stamp),
            reason=msg.reason,
            node_name=msg.node_name,
        )
