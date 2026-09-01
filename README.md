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

`return_code` is `0` on success, `1` when nothing was actioned or on error. The
timestamp-scheduled forms of `resume` and `split_bagfile` are not implemented and return an
explicit error rather than silently acting immediately.

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
src/                                     upstream rosbag2 checkout (untracked)
```

`notes/` is worth reading before changing anything —
[architecture.md](notes/architecture.md) records *why* this is built on `Writer` rather than
wrapping `Recorder`, and [spike-plan.md](notes/spike-plan.md) has measured timings (adding a topic
costs ~0.4–0.6s, almost all of it message-definition resolution) plus one retracted conclusion
worth not rediscovering.

## Development

Everything builds and runs inside a Docker container, so nothing is installed on the host.

```bash
docker compose build
docker compose up -d

# Upstream rosbag2 sources are not tracked here; fetch them once:
git clone https://github.com/ros2/rosbag2.git src && git -C src checkout ae42fb9

docker compose exec rosbag2-dev bash -l
```

Then inside the container:

```bash
colcon build --packages-select \
  rosbag2_dynamic_recorder_interfaces rosbag2_dynamic_recorder --symlink-install
source install/setup.bash
bash src/packages/rosbag2_dynamic_recorder/test/m1_smoke_test.sh
```

Notes:

- **Host networking** is used because DDS discovery needs real network interfaces.
- Our packages live in `packages/` and `spike/`, mounted into the workspace separately, so the
  upstream checkout in `src/` stays pristine.
- The image carries the workspace dependencies. If you add a package with new dependencies, run
  `rosdep install -r -y --from-paths src --ignore-src` inside the container.

## License

Apache-2.0.
