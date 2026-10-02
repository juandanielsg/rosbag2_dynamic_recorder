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

The rule the CLI's `--json` and the browser UI rely on is enforced here, once: **a number the
recorder cannot vouch for is None, never a convenient zero.** A raw `messages_missed` reads 0 on a
middleware that cannot count gaps at all.

No ROS import: the `from_msg` constructors only read attributes, so any object carrying the right
fields will do in a test.
"""

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

from dynrec.schedule import NANOSECONDS_PER_SECOND

#: Maps the uint8 action constants onto words, because a script comparing against 0 and 1 is one
#: constant rename away from silently inverting.
SUBSCRIPTION_ACTIONS = {0: 'subscribed', 1: 'unsubscribed'}
PAUSE_ACTIONS = {0: 'paused', 1: 'resumed'}
#: Shared by LowDiskEvent and BagSizeLimitEvent: a self-inflicted stop is their only action.
STOP_ACTIONS = {0: 'stopped'}
#: ScheduledAction's action and mode constants. The mode words are the ones `mode=` takes.
SCHEDULE_ACTIONS = {0: 'resume', 1: 'split', 2: 'record'}
SCHEDULE_MODES = {0: 'node', 1: 'publish', 2: 'receive'}


def _stamp_seconds(stamp):
    """A builtin_interfaces/Time as epoch seconds on the recorder's clock."""
    return stamp.sec + stamp.nanosec / NANOSECONDS_PER_SECOND


def _action_name(names, msg):
    """`msg.action` as a word, or as its number when the constant is one this library predates."""
    return names.get(msg.action, str(msg.action))


@dataclass(frozen=True)
class Scheduled:
    """An operation the recorder will perform later: one entry of :attr:`Status.schedules`."""

    #: 'resume', 'split' or 'record'.
    action: str
    #: When it fires, as epoch seconds on the clock `mode` names. For 'node' that is the
    #: recorder's clock, the one `Status.recording_started` is on.
    at: float
    #: 'node' fires on a timer; 'publish' and 'receive' wait for a message stamped that late.
    mode: str
    #: For 'publish' and 'receive': the topic whose messages are checked, or '' for any topic.
    topic: str = ''

    @classmethod
    def from_msg(cls, msg):
        return cls(
            action=_action_name(SCHEDULE_ACTIONS, msg),
            at=_stamp_seconds(msg.time),
            mode=SCHEDULE_MODES.get(msg.mode, str(msg.mode)),
            topic=msg.tracking_topic,
        )


@dataclass(frozen=True)
class TopicLoss:
    """What one topic has lost in the current bag: one entry of :attr:`Status.topic_losses`."""

    topic: str
    #: From sequence-number gaps. None where the middleware supplies no sequence numbers, as for
    #: :attr:`Status.messages_missed`.
    missed: Optional[int]
    lost_in_transport: int
    lost_in_recorder: int

    @classmethod
    def from_msg(cls, msg, sequence_ok):
        return cls(
            topic=msg.topic_name,
            missed=msg.messages_missed if sequence_ok else None,
            lost_in_transport=msg.messages_lost_in_transport,
            lost_in_recorder=msg.messages_lost_in_recorder,
        )


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
    #: Losses the transport *reported* before delivery: a floor, not a total, since the transport
    #: can drop messages without reporting them. When this climbs, look at the publisher, its QoS,
    #: or the network.
    messages_lost_in_transport: int
    #: Messages that reached the recorder and were then dropped by the writer: cache overflow
    #: because the disk could not keep up, or a failed storage write. Known-lost with certainty,
    #: and the remedy is local -- cache size or duration, storage preset, topic set, or the disk.
    messages_lost_in_recorder: int
    #: The two above summed, for callers that only want a total. Inherits the transport caveat.
    messages_lost_reported: int
    write_errors: int
    bag_splits: int
    #: Bytes flushed to disk, which lags what has been captured because the writer caches. Reads 0
    #: early in a healthy recording; use `messages_written` for liveness.
    bag_size_bytes: int
    sequence_numbers_available: bool
    #: Bytes available to the recorder on the filesystem holding the bag, or 0 when it could not be
    #: determined. This is what the free-space guard compares against.
    free_space_bytes: int = 0
    #: Total size of that filesystem in bytes, or 0 when it could not be determined.
    total_space_bytes: int = 0
    #: Configured minimum free bytes that stops recording. 0 when not set.
    min_free_space: int = 0
    #: Configured minimum free space as a percentage of the filesystem. 0.0 when not set.
    min_free_space_percent: float = 0.0
    #: True when the recorder stopped itself because free space fell below the configured minimum.
    #: Cleared when `~/record` opens a new bag.
    stopped_for_low_disk: bool = False
    #: Configured cap on the bag directory in bytes, across every split. 0 when not set.
    max_bag_size: int = 0
    #: True when the recorder stopped itself because the bag grew past `max_bag_size`. Cleared when
    #: `~/record` opens a new bag.
    stopped_for_max_bag_size: bool = False
    #: True when messages are stamped with the node clock driven by `/clock`, so the bag's
    #: timeline is the simulation's rather than the wall clock's.
    use_sim_time: bool = False
    #: True under `use_sim_time` until the first `/clock` message: no bag is open yet, and every
    #: call that needs one is refused until then.
    waiting_for_clock: bool = False
    #: Resumes, splits and records queued for later, at most one of each, in that order. Includes
    #: schedules any client set. There is no call to cancel one; scheduling again replaces it.
    schedules: List[Scheduled] = field(default_factory=list)
    #: The loss figures broken down by topic, for each topic that has lost anything in this bag,
    #: sorted by name. Empty when nothing has been lost.
    topic_losses: List[TopicLoss] = field(default_factory=list)
    #: When the recorder built this status, as epoch seconds on its clock: that clock's reading as
    #: of now, give or take delivery. `elapsed_seconds` stops counting at a stop; this does not.
    stamp: float = 0.0

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
            messages_lost_in_transport=msg.messages_lost_in_transport,
            messages_lost_in_recorder=msg.messages_lost_in_recorder,
            messages_lost_reported=msg.messages_lost,
            write_errors=msg.write_errors,
            bag_splits=msg.bag_splits,
            bag_size_bytes=msg.bag_size_bytes,
            sequence_numbers_available=sequence_ok,
            free_space_bytes=msg.free_space_bytes,
            total_space_bytes=msg.total_space_bytes,
            min_free_space=msg.min_free_space,
            min_free_space_percent=msg.min_free_space_percent,
            stopped_for_low_disk=bool(msg.stopped_for_low_disk),
            max_bag_size=msg.max_bag_size,
            stopped_for_max_bag_size=bool(msg.stopped_for_max_bag_size),
            use_sim_time=bool(msg.use_sim_time),
            waiting_for_clock=bool(msg.waiting_for_clock),
            schedules=[Scheduled.from_msg(s) for s in msg.schedules],
            topic_losses=[TopicLoss.from_msg(t, sequence_ok) for t in msg.topic_losses],
            stamp=_stamp_seconds(msg.stamp),
        )

    def as_dict(self):
        """Plain data for logging or JSON, with the unknown-versus-zero rule intact."""
        return asdict(self)


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
    """One thing that happened to the recording, from one of five event streams.

    All five are flattened into one shape because a caller watching for "what changed" wants them
    interleaved, and because they are only meaningful together: a subscription event explains a
    channel that stops, a pause event explains a bag that stops, a split event explains where one
    file ends and the next begins, and a low-disk or bag-size-limit event explains a recording the
    recorder ended itself, to protect the filesystem or to honour a cap on the bag.

    `stamp` is on the recorder's clock. Sort by it rather than by arrival: the streams come in on
    separate subscriptions, so delivery order is not the order things happened.
    """

    #: 'subscription', 'pause', 'split', 'low_disk' or 'bag_size_limit'.
    kind: str
    #: 'subscribed', 'unsubscribed', 'paused', 'resumed', 'split' or 'stopped'.
    action: str
    #: Epoch seconds on the recorder's clock.
    stamp: float
    #: What caused it, e.g. 'service:set_topics', 'schedule:resume', 'startup', 'timer'.
    reason: str
    node_name: str
    #: Empty for a pause, low-disk or bag-size-limit event, which affect the whole recording rather
    #: than one topic.
    topic: str = ''
    topic_type: str = ''
    #: For a low-disk event: bytes available on the recorder's filesystem at the check that fired,
    #: and its total size. Zero on the other kinds, which have no filesystem figure to carry.
    free_space_bytes: int = 0
    total_space_bytes: int = 0
    #: For a bag-size-limit event: the bag's size on disk at the check that fired, and the limit it
    #: exceeded. Zero on the other kinds.
    bag_size_bytes: int = 0
    max_bag_size: int = 0
    #: For a split event: the file that was closed and the file recording continues in. Empty on
    #: the other kinds.
    closed_file: str = ''
    opened_file: str = ''

    @classmethod
    def _from_msg(cls, kind, action, msg, **fields):
        """The fields every event message carries, plus the kind-specific `fields`."""
        return cls(
            kind=kind, action=action, stamp=_stamp_seconds(msg.stamp), reason=msg.reason,
            node_name=msg.node_name, **fields)

    @classmethod
    def from_subscription_msg(cls, msg):
        return cls._from_msg(
            'subscription', _action_name(SUBSCRIPTION_ACTIONS, msg), msg,
            topic=msg.topic_name, topic_type=msg.topic_type)

    @classmethod
    def from_pause_msg(cls, msg):
        return cls._from_msg('pause', _action_name(PAUSE_ACTIONS, msg), msg)

    @classmethod
    def from_file_split_msg(cls, msg):
        return cls._from_msg(
            'split', 'split', msg, closed_file=msg.closed_file, opened_file=msg.opened_file)

    @classmethod
    def from_low_disk_msg(cls, msg):
        return cls._from_msg(
            'low_disk', _action_name(STOP_ACTIONS, msg), msg,
            free_space_bytes=msg.free_space_bytes, total_space_bytes=msg.total_space_bytes)

    @classmethod
    def from_bag_size_limit_msg(cls, msg):
        return cls._from_msg(
            'bag_size_limit', _action_name(STOP_ACTIONS, msg), msg,
            bag_size_bytes=msg.bag_size_bytes, max_bag_size=msg.max_bag_size)


#: The recorder's event streams: topic suffix under the recorder's name, the message type in
#: `rosbag2_dynamic_recorder_interfaces.msg`, and the Event constructor for it. The client
#: subscribes to every row and the bag reader decodes every row, so a new stream is one row here.
EVENT_STREAMS = {
    '/events/subscription_change': ('SubscriptionChangeEvent', Event.from_subscription_msg),
    '/events/pause': ('PauseEvent', Event.from_pause_msg),
    '/events/file_split': ('FileSplitEvent', Event.from_file_split_msg),
    '/events/low_disk': ('LowDiskEvent', Event.from_low_disk_msg),
    '/events/bag_size_limit': ('BagSizeLimitEvent', Event.from_bag_size_limit_msg),
}
