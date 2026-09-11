# The service API

Everything else in this project (the CLI, the Python library, the browser UI) is a client of these
services. A node that speaks ROS needs nothing else.

The names below are relative to the recorder node, so with the default name `~/pause` is
`/rosbag2_dynamic_recorder/pause`.

## Topic management

| Service | Type | Purpose |
|---|---|---|
| `~/subscribe_topics` | `SubscribeTopics` | Add topics. Types discovered from the graph if not given. |
| `~/unsubscribe_topics` | `UnsubscribeTopics` | Drop topics. Recorded messages are kept. |
| `~/set_topics` | `SetTopics` | Replace the whole set. |
| `~/get_subscribed_topics` | `GetSubscribedTopics` | The current set. |
| `~/set_profile` | `SetProfile` | Apply a named topic set. |
| `~/get_profiles` | `GetProfiles` | List the configured profiles. |

The guarantee that matters: on `set_topics` and `set_profile`, topics present in both the old and
the new set are never torn down. They keep recording without a gap, which is what makes a mode
switch mid-recording safe. Measured, the largest gap on such a topic across two changes is 0.05s,
one publish interval.

`return_code` is `0` on success and `1` when nothing was actioned or on error. An empty
`set_topics` is a valid request that unsubscribes everything, and is reported as success. The
reused `rosbag2_interfaces` services under Recording control keep their own return codes instead.

### Selecting by pattern

`subscribe_topics`, `unsubscribe_topics` and `set_topics` all take `regex` and `exclude_regex`.

The semantics are stock `ros2 bag record -e`'s: an ECMAScript search, not a full match, so `camera`
finds `/robot/camera/image` without anchors. Anchor it yourself (`^/robot/`) when you mean a prefix.
`exclude_regex` is applied to the combined selection, after names and matches are merged, so
"everything under `/robot` except the depth camera" takes one call rather than two.

Three rules:

- A pattern never matches the recorder's own topics. Its event channels are already written into
  the bag directly, so subscribing to them would record every event twice. Hidden topics are
  excluded too, as stock rosbag2 does.
- `unsubscribe_topics` matches what is being recorded, not the graph. Only a recorded topic can be
  dropped, and matching the graph would silently do nothing for one that has since left it.
- An invalid expression is refused with the reason. Quietly matching nothing would be
  indistinguishable from a pattern that legitimately found nothing, and the likely cause is a typo.

### Naming a type explicitly

`topic_types` lines up with `topics` by index. An empty entry requests discovery from the graph, which
succeeds only when the topic currently offers exactly one type, and cannot work before the publisher
exists. Naming the type is how you record a topic whose publisher has not started yet.

A pattern match has no index, so it cannot carry an explicit type; matched topics always take their
type from the graph.

## Recording control

These use the stock `rosbag2_interfaces` definitions, so a client written against standard rosbag2
drives this node unchanged.

| Service | Type | Purpose |
|---|---|---|
| `~/pause` `~/resume` `~/toggle_paused` `~/is_paused` | `Pause`, `Resume`, … | Stop writing without tearing down subscriptions. |
| `~/split_bagfile` | `SplitBagfile` | Close the current file and open the next. |
| `~/snapshot` | `Snapshot` | Flush the in-memory buffer. Requires `snapshot_mode`. |
| `~/stop` | `Stop` | Close the bag. |
| `~/record` | `Record` | Open a new bag, restoring the previous topic selection. |
| `~/get_status` | `GetStatus` | Everything needed for a status line, in one round trip. |

Stopping is not final: `~/record` opens a fresh bag and re-subscribes whatever was being recorded at
stop. Since rosbag2 will not open over an existing bag directory, the new one takes the next free
suffix: `mybag`, then `mybag(1)`.

### Scheduled operations

`resume`, `split_bagfile` and `record` accept a future timestamp instead of acting immediately. A
zero stamp means "now", so an untouched request is already the immediate request.

- Node time (`mode: 0`, and the only option for `record`) is driven by a timer, so it fires even if
  the robot has gone quiet.
- Publish time (`1`) and receive time (`2`) are compared against arriving messages, on any recorded
  topic or on `tracking_topic_name` alone. They cannot fire while no messages are arriving, which
  is useful for "when data resumes" and a trap for "in five minutes".

A mode outside 0–2, or a `tracking_topic_name` nobody is recording, is rejected: a schedule keyed
to a topic that is not being recorded would wait forever.

`~/stop` clears a queued `resume` or `split_bagfile`, so it cannot fire against the next bag. A
queued `record` is not cleared and will still fire.

## Status

`~/get_status` is the one-shot form; `~/status` is the same message published periodically, on every
change, and latched, so a client attaching mid-recording sees current state immediately rather than
waiting for the next tick. A UI or a long-lived script should prefer the topic.

A few fields need reading carefully:

`messages_missed`
: Detected by watching for gaps in each publisher's sequence numbers. It does not depend on the
  transport raising an event, so it catches samples a publisher overwrote in its own history before
  the recorder ever saw them. Meaningful only when `sequence_numbers_available` is true; treat it as
  unknown, not zero, when that is false.

`messages_lost`
: What the transport or writer reported. Observed reading 0 while roughly 3–4% of messages were
  absent from a bag. Treat it as a floor, not a total. A large `messages_missed` with this at 0 means
  the recorder was blocked long enough for publisher history to overflow.

`bag_size_bytes`
: Bytes flushed to disk, not bytes captured. The writer caches, so this reads 0 early in a perfectly
  healthy recording. Use `messages_written` to answer "is it recording?".

`write_errors`
: Messages that reached the recorder but could not be written, for example when the disk filled. Any
  non-zero value means the bag is incomplete, and the recorder kept running deliberately, since
  stopping would lose the rest of the recording too.

The remaining fields are literal: `recording`, `paused`, `snapshot_mode`, `elapsed_seconds`,
`recording_started`, `subscribed_topics`, `bag_splits` (times the file has rolled over) and
`messages_written` (which counts the recorder's own event messages too).

`active_profile` is derived from the live topic set on every publication, not remembered. Record one
extra topic and it goes empty, because no profile is in effect any more. Land exactly on another
profile's set and it reports that one, whatever command got you there.

## Events

| Topic | Message | |
|---|---|---|
| `~/events/subscription_change` | `SubscriptionChangeEvent` | also written into the bag |
| `~/events/pause` | `PauseEvent` | also written into the bag |
| `~/events/write_split` | `WriteSplitEvent` | |
| `~/events/messages_lost` | `MessagesLostEvent` | |

Writing the first two into the bag is what makes a gap legible: a gap mid-bag is otherwise
indistinguishable from a dropout, a crash or a network fault. The event records what changed, when and why, in-stream and
timestamped, and it survives bag splitting.

The two cover different shapes of gap, and you need both:

- `SubscriptionChangeEvent` explains a channel that stops: one topic goes sparse while the others
  carry on. Disable with `record_subscription_events:=false`.
- `PauseEvent` explains a bag that stops: a pause leaves a hole in every topic at once, which is
  what a crash or a network fault looks like too. The stock `rosbag2_interfaces/srv/Pause` has an
  empty request and response and so can carry no explanation. Disable with
  `record_pause_events:=false`.

While paused no messages are written, but these events still are. An event suppressed by the pause
it describes would leave the hole it exists to account for.

:::{warning}
Bag readers return messages in timestamp order, so the recorded order of events is only as monotonic
as the clock was. A backward clock step (NTP correcting, or a virtualised host, where ~120 ms steps
have been measured) reshuffles the events inside it, and a `PAUSED` can then appear after its
`RESUMED`. The recorder's own transition sequence is always correct and no event is lost; only the
recorded order follows the clock. If a sequence fails to alternate, suspect the clock before the
recorder.
:::

## Profiles

A named topic set, switched in one call. The fleet case it exists for: a supervisor changes the
robot's operating mode and the recorded topic set follows, without interrupting the topics the two
modes share.

```yaml
rosbag2_dynamic_recorder:
  ros__parameters:
    profile_names: ["idle", "navigation", "inspection"]
    profiles:
      idle: ["/tf", "/odom", "/imu"]
      navigation: ["/tf", "/odom", "/imu", "/scan"]
```

A worked example using the TurtleBot 4 simulator's topics ships at
`rosbag2_dynamic_recorder/config/profiles.example.yaml`.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `uri` | `dynamic_bag` | Output bag path. |
| `storage_id` | `mcap` | Storage plugin. |
| `serialization_format` | `cdr` | Message serialization format. |
| `topics` | `[]` | Topics to subscribe at startup. |
| `start_paused` | `false` | Start with recording paused. |
| `snapshot_mode` | `false` | Buffer in memory, write only on `~/snapshot`. |
| `max_cache_size` | `104857600` | Writer cache in bytes. Must be > 0 for `snapshot_mode`. |
| `record_subscription_events` | `true` | Write subscription changes into the bag. |
| `record_pause_events` | `true` | Write pauses and resumes into the bag. |
| `messages_lost_report_period` | `5.0` | Seconds between `MessagesLostEvent`. `0` disables. |
| `status_publish_period` | `1.0` | Seconds between `~/status` publications. |
| `profile_names` | `[]` | Names of the declared profiles. |

These are node parameters. The launch files forward only some of them: `uri`, `storage_id`,
`serialization_format`, `topics`, `start_paused`, `snapshot_mode`, `max_cache_size`,
`record_subscription_events` and `messages_lost_report_period`. Set `record_pause_events`,
`status_publish_period`, `profile_names` and the profiles themselves through `params_file` (or
`-p name:=value` when running the node directly). The full argument list is in
[Install](install.md#launch-arguments).

## Calling them by hand

```bash
ros2 run rosbag2_dynamic_recorder dynamic_recorder --ros-args \
  -p uri:=/tmp/mybag -p "topics:=['/scan','/odom']"

ros2 service call /rosbag2_dynamic_recorder/subscribe_topics \
  rosbag2_dynamic_recorder_interfaces/srv/SubscribeTopics "{topics: ['/diagnostics']}"

ros2 service call /rosbag2_dynamic_recorder/set_topics \
  rosbag2_dynamic_recorder_interfaces/srv/SetTopics "{topics: ['/scan','/odom','/tf']}"
```

The other clients exist so you do not have to call these by hand; [`ros2 dynrec`](cli.md) is the
simplest of them.
