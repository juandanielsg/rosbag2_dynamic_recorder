# rosbag2_dynamic_recorder

A ROS 2 bag recorder whose **recorded topic set can change while it is recording** — add a topic,
drop a topic, or swap the whole set — without stopping the writer and without splitting the bag.

Recording continues uninterrupted on every topic you did not touch.

## Why

`rosbag2_transport::Recorder` fixes its topic set at construction from `RecordOptions`. Changing
it means `stop()` → reconfigure → `record()`, which:

- **tears down every subscription**, so unrelated topics lose messages every time you add or
  remove one; and
- **closes the bag**, so each change forces a new file.

For a robot that records different topics per operational mode, that means a gap on `/tf` and
`/odom` every time an unrelated diagnostic topic is toggled, plus a file set whose boundaries
encode two unrelated things at once.

This records into **one continuous MCAP stream**. A topic that gets dropped and re-added becomes a
sparse channel with a hole — which MCAP supports natively, and which `ros2 bag play`, `mcap info`
and Foxglove all handle.

## What it is not

Not a replacement for rosbag2. Bags are written by `rosbag2_cpp::Writer`, so the output is a
standard rosbag2 bag and the whole ecosystem keeps working. Only the *control plane* is different.

## Services

Topic management:

| Service | Purpose |
|---|---|
| `~/subscribe_topics` | Add topics. Types are discovered from the graph if not given. |
| `~/unsubscribe_topics` | Drop topics. Recorded messages are kept. |
| `~/set_topics` | Replace the whole set atomically. Topics in both old and new are untouched. |
| `~/get_subscribed_topics` | Current set. |

Recording control. These use the stock `rosbag2_interfaces` definitions, so a client written
against standard rosbag2 drives this node unchanged:

| Service | Purpose |
|---|---|
| `~/pause` `~/resume` `~/toggle_paused` `~/is_paused` | Stop writing without tearing down subscriptions. |
| `~/split_bagfile` | Close the current file and open the next. |
| `~/snapshot` | Flush the in-memory buffer. Requires `snapshot_mode`. |
| `~/stop` | Close the bag. |
| `~/record` | Open a new bag and start again, restoring the previous topic selection. |
| `~/get_status` | Everything a UI or CLI needs for a status line, in one round trip. |

`return_code` is `0` on success, `1` when nothing was actioned or on error. The
timestamp-scheduled forms of `record`, `resume` and `split_bagfile` are not implemented and return
an explicit error rather than silently acting immediately.

Stopping is not a dead end: `~/record` opens a fresh bag and re-subscribes whatever was being
recorded when you stopped. Since rosbag2 will not open over an existing bag directory, the new one
gets the next free suffix — `mybag`, then `mybag(1)`.

`~/get_status` reports the bag URI and storage id, recording/paused/snapshot state, start time and
elapsed seconds, the subscribed topics, messages written, messages lost, split count and bag size
on disk. Note that `bag_size_bytes` counts bytes *flushed*, not captured: the writer caches, so it
reads 0 early in a recording. Use `messages_written` to answer "is it recording?".

## Events

| Topic | Message |
|---|---|
| `~/events/subscription_change` | `SubscriptionChangeEvent` — **also written into the bag** |
| `~/events/write_split` | `WriteSplitEvent` |
| `~/events/messages_lost` | `MessagesLostEvent` |

Recording the subscription changes into the bag is the point: a channel that stops mid-bag is
otherwise indistinguishable from a dropout, a crash, or a network fault. The event says what
changed, when, and why — in-stream, timestamped, and surviving bag splits. Disable with
`record_subscription_events:=false`.

```bash
ros2 run rosbag2_dynamic_recorder dynamic_recorder --ros-args \
  -p uri:=/tmp/mybag -p "topics:=['/scan','/odom']"

ros2 service call /rosbag2_dynamic_recorder/subscribe_topics \
  rosbag2_dynamic_recorder_interfaces/srv/SubscribeTopics "{topics: ['/diagnostics']}"

ros2 service call /rosbag2_dynamic_recorder/set_topics \
  rosbag2_dynamic_recorder_interfaces/srv/SetTopics "{topics: ['/scan','/odom','/tf']}"
```

### Parameters

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
| `messages_lost_report_period` | `5.0` | Seconds between `MessagesLostEvent`. `0` disables. |

There is a launch file too:

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag topics:="['/scan','/odom']"
```

## Status

Working and verified end to end; not yet run on real hardware. See
[notes/roadmap.md](notes/roadmap.md).

- **M1 done** — topic management, verified by
  [`m1_smoke_test.sh`](packages/rosbag2_dynamic_recorder/test/m1_smoke_test.sh): topics added and
  removed mid-recording land in a single MCAP with no gap on untouched topics.
- **M2 done** — recording control, events and provenance, verified by
  [`m2_smoke_test.sh`](packages/rosbag2_dynamic_recorder/test/m2_smoke_test.sh).
- **M3 next** — named topic-set profiles switched atomically on mode change.

## Layout

```
packages/
  rosbag2_dynamic_recorder_interfaces/   service definitions
  rosbag2_dynamic_recorder/              the node
notes/                                   architecture, spike findings, roadmap
spike/                                   throwaway validation of the core premise
src/                                     optional upstream rosbag2 checkout (untracked)
```

`notes/` is worth reading before changing anything —
[architecture.md](notes/architecture.md) records *why* this is built on `Writer` rather than
wrapping `Recorder`, and [spike-plan.md](notes/spike-plan.md) has measured timings (adding a topic
costs ~0.4–0.6s, almost all of it message-definition resolution) plus one retracted conclusion
worth not rediscovering.

## Install

You need ROS 2 Rolling. Everything else comes from apt; there is no need to build rosbag2 from
source.

```bash
sudo apt install ros-rolling-rosbag2 ros-rolling-rosbag2-storage-mcap

mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/juandanielsg/rosbag2_dynamic_recorder.git
cd ~/ws && colcon build --symlink-install
source install/setup.bash
```

The build is two small packages and takes about a minute. Then:

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag topics:="['/scan','/odom']"
```

## The browser UI

The way to use this without learning service-call syntax:

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag
```

Then open **http://localhost:8088**.

Tick a topic to start recording it, untick to stop. Topics you did not touch keep recording
without a gap. Pause, starting a new file, and stopping are buttons.

The page is a single static file served by a ROS node **on your own machine** — no account, no
cloud, no separate application, and no internet, so it works on a robot with no network. It
depends on nothing beyond `rclpy` and the Python standard library: no web framework and no npm
build step, because every dependency there is an install barrier.

It also refuses to overstate what it knows. Where the recorder cannot actually measure something,
the UI says **unknown** rather than showing a reassuring zero — see
[notes/recording-stall.md](notes/recording-stall.md) for the case where `messages_lost` read 0
while 3–4% of messages were genuinely absent.

## Tests

```bash
colcon test --packages-select rosbag2_dynamic_recorder
colcon test --packages-select rosbag2_dynamic_recorder_ui --python-testing pytest
colcon test-result --verbose
```

**Fourteen integration tests** start a real recorder process and drive its services, on a private
`ROS_DOMAIN_ID` with their own publishers — no simulator needed, and they cannot collide with
anything else running on the machine.

Some check the service contract: that a topic present before and after a `set_topics` is never
torn down, that a stopped recorder refuses for the right reason, that `~/record` restores the
previous selection, that scheduled operations are refused rather than silently performed
immediately.

The rest open the resulting bag and check what was actually written — that a topic change does not
split the file, that the untouched topic spans the whole recording without a hole, that the
dropped and added topics appear as sparse channels, and that the bag carries the
`SubscriptionChangeEvent`s explaining them. Measured, the untouched topic's largest gap across two
changes is **0.05s — one publish interval**, which is the central claim of this project stated as
a number rather than a promise.

**Seven unit tests** cover the UI's rigor rule: unmeasurable values must render as *unknown*, never
as a convenient zero. `build_state()` is a plain function precisely so this is testable without a
ROS graph.

The `--python-testing pytest` flag on the second command is a colcon quirk, not ours: without it
colcon picks the deprecated setuptools test runner for `ament_python` packages, collects nothing,
and exits 5 as though something failed.

## Development

A Docker dev container is provided for working on the project itself, and for running the
recorder against a simulator without installing ROS on the host.

```bash
docker compose build
docker compose up -d
docker compose exec rosbag2-dev bash -l
```

Inside the container:

```bash
colcon build --packages-select \
  rosbag2_dynamic_recorder_interfaces rosbag2_dynamic_recorder --symlink-install
source install/setup.bash
bash src/packages/rosbag2_dynamic_recorder/test/m1_smoke_test.sh
```

The upstream rosbag2 sources are **optional** — useful for reading the code this builds against,
not required to build:

```bash
git clone https://github.com/ros2/rosbag2.git src && git -C src checkout ae42fb9
```

Notes:

- The container publishes port 8088, so the browser UI is reachable at `http://localhost:8088`
  from the host. 8088 rather than 8080 because 8080 is very often already taken. It does **not** use `network_mode: host`: on Docker Desktop that is the WSL2 VM's
  network, which the host OS cannot reach.
- Our packages live in `packages/` and `spike/`, mounted into the workspace separately, so the
  optional upstream checkout in `src/` stays pristine.
- The image carries the workspace dependencies. If you add a package with new dependencies, run
  `rosdep install -r -y --from-paths src --ignore-src` inside the container.

## License

Apache-2.0.
